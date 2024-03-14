//
// Created by xin on 23-6-21.
//

#include "myslam/keyframeDatabase.h"
#include "DBoW3/DBoW3.h"

namespace myslam {

    DBoW3::Database &KeyframeDatabase::getDatabase() {
        std::unique_lock<std::mutex> lck(mutex_);
        return keyframe_database_;
    }

    KeyframeDatabase::KeyframesType KeyframeDatabase::GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(mutex_);
        return keyframes_;
    }

    void KeyframeDatabase::InsertKeyFrame(Frame::Ptr frame) {
//        std::unique_lock<std::mutex> lck(mutex_);
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        } else {
            keyframes_[frame->keyframe_id_] = frame;
        }

        keyframe_database_.add(frame->descriptors_);
    }

} // myslam