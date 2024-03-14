//
// Created by xin on 23-6-21.
//

#ifndef MYSLAM_KEYFRAMEDATABASE_H
#define MYSLAM_KEYFRAMEDATABASE_H

#include "common_include.h"
#include "frame.h"
#include "DBoW3/DBoW3.h"

namespace myslam {
    class Frame;


    class KeyframeDatabase {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<KeyframeDatabase> Ptr;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        KeyframeDatabase() : keyframe_database_(DBoW3::Vocabulary("../vocabulary.yml.gz"), false, 0) {};

        void InsertKeyFrame(Frame::Ptr frame);

        // 由于访问 database 和 insertKeyframe 可能还冲突，所以需要对 database需要加锁
        DBoW3::Database& getDatabase();

        KeyframesType GetAllKeyFrames();

    public:

        DBoW3::Database keyframe_database_;
        std::mutex mutex_;  // 定义一个互斥锁
        KeyframesType keyframes_;  // all key-frames

    };

} // myslam

#endif //MYSLAM_KEYFRAMEDATABASE_H
