#include <iostream>

#include <ros/ros.h>

#include "src/path_recorder.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_recorder");

    PathRecorder path_recorder;
    path_recorder.Record();

    return 0;
}
