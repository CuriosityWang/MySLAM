//
// Created by xin on 23-6-21.
//

#include "myslam/loop_closure.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "DBoW3/DBoW3.h"

namespace myslam {
    LoopClosure::LoopClosure() {
        database_ = KeyframeDatabase::Ptr(new KeyframeDatabase);
        loop_closure_running_.store(true);
        loop_closure_thread_ = std::thread(std::bind(&LoopClosure::LoopClosureLoop, this));
    }

    void LoopClosure::Stop() {
        loop_closure_running_.store(false);
        map_update_.notify_one();
        loop_closure_thread_.join();
    }

    void LoopClosure::UpdateMap() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    int LoopClosure::DetectLoop(){
        DBoW3::Database &db = database_->getDatabase();
        DBoW3::QueryResults ret;
        db.query(current_frame_->descriptors_, ret, 4);
        for (const auto& result : ret) {
            unsigned int imageId = result.Id; // 获取图像ID
            double score = result.Score; // 获取匹配分数
            if(score > min_loop_score && score < max_loop_score ){
                return imageId;
            }
        }
        return -1;
    }


    void LoopClosure::InsertKeyframe(Frame::Ptr frame) {
        current_frame_ = frame;
        database_->InsertKeyFrame(frame);
    }

    void LoopClosure::LoopClosureLoop() {

        while (loop_closure_running_.load()){
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            int loop_start_index = DetectLoop();
            LOG(INFO) <<"Loop Detecting";

            if(loop_start_index >= 0){
                frame_passed_++;
                //这里是由于重复的回环不会使得精度再有提升，因此一次回环检测之后的后五次检测不执行更新
                if(frame_passed_ == 1){
                    int loop_end_index = current_frame_->keyframe_id_;
                    LOG(INFO) << "\n\n\n\n\n" <<"Loop Detected at" << loop_start_index << "--------" << loop_end_index << "\n\n\n\n";

                    Map::KeyframesType loop_kfs = map_->GetLoopKeyFrames(loop_start_index, loop_end_index);
                    Map::LandmarksType loop_landmarks = map_->GetLoopMapPoints(loop_start_index, loop_end_index);

                    /// 回环检测线程优化 回环上的 frames 和 landmarks
                    Optimize(loop_kfs, loop_landmarks);

                }
                else if (frame_passed_ > 5)
                    frame_passed_ = 0;
                else {
                    frame_passed_++;
                }


            }


        }
    }

    void LoopClosure::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks) {

        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // pose 顶点，使用Keyframe id
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);
            if (kf->keyframe_id_ > max_kf_id) {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 路标顶点，使用路标id索引
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        // K 和左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // edges
        int index = 1;
        double chi2_th = 5.991;  // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for (auto &landmark : landmarks) {
            if (landmark.second->is_outlier_) continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations) {
                if (obs.lock() == nullptr) continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_) {
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                // 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end()) {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});

                optimizer.addEdge(edge);

                index++;
            }
        }

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }
        }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier;

        // Set pose and lanrmark position
        for (auto &v : vertices) {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }

    }

} // myslam