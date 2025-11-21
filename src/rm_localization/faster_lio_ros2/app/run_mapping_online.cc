//
// Created by xiang on 2021/10/8.
//
// #include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"

/// run the lidar mapping in online mode

// DEFINE_string(params_file, "", "path to traj log file");
// DEFINE_string(ros_args, "", "path to traj log file");
// DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    // ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // FLAGS_stderrthreshold = google::INFO;
    // FLAGS_colorlogtostderr = true;
    // google::InitGoogleLogging(argv[0]);
    // google::ParseCommandLineFlags(&argc, &argv, true);

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();

    rclcpp::spin(laser_mapping);
    

    // laser_mapping->InitROS(nh);

    signal(SIGINT, SigHandle);
    // ros::Rate rate(5000);

    // online, almost same with offline, just receive the messages from ros
    // while (ros::ok()) {
    //     if (faster_lio::options::FLAG_EXIT) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     laser_mapping->Run();
    //     rate.sleep();
    // }

    // LOG(INFO) << "finishing mapping";
    // laser_mapping->Finish();

    // faster_lio::Timer::PrintAll();
    // LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    // laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    return 0;
}
