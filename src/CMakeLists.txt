add_library(myslam STATIC
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        keyframeDatabase.cpp
        loop_closure.cpp)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
