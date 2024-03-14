//
// Created by xin on 23-6-21.
//

#ifndef MYSLAM_LOOP_CLOSURE_H
#define MYSLAM_LOOP_CLOSURE_H

#include "common_include.h"
#include "keyframeDatabase.h"
#include "frame.h"
#include "map.h"

/*
 * 回环检测线程，在检测到回环时启动优化
 * Map更新由前端触发
 */

namespace myslam {
    class Map;

    class LoopClosure {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<LoopClosure> Ptr;

        /// 构造函数中启动回环检测优化线程并挂起
        LoopClosure();

        /// 设置左右目相机，用来获取相机内外参
        void SetCameras(Camera::Ptr left, Camera::Ptr right) {
            cam_left_ = left;
            cam_right_ = right;
        }

        /// 设置地图
        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        void InsertKeyframe(Frame::Ptr frame);

        /// 触发地图更新，启动优化
        void UpdateMap();

        /// 关闭回环检测线程
        void Stop();

    private:
        /// 回环检测线程
        void LoopClosureLoop();

        int DetectLoop();

        /// 对回环上的关键帧和路标点进行优化
        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    public:

    private:

        double min_loop_score = 0.4;
        double max_loop_score = 0.9;

        unsigned int frame_passed_ = 0;

        std::thread loop_closure_thread_; // 建立一个回环检测线程
        std::mutex data_mutex_; // 互斥锁，用于在多线程环境下对共享数据进行保护
        std::condition_variable map_update_; // 用于通知闭环检测线程有新的地图数据可用，从而触发闭环检测的执行。
        std::atomic<bool> loop_closure_running_; // 用来启动或停止闭环检测线程，以及检查线程是否正在运行。

        std::shared_ptr<KeyframeDatabase> database_;
        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
        std::shared_ptr<Map> map_;

        std::shared_ptr<Frame> current_frame_;

    };

} // myslam

#endif //MYSLAM_LOOP_CLOSURE_H
