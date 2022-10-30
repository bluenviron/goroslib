#include <ros/ros.h>

#include <std_msgs/String.h>

#include <shared_services/Mysrv.h>

bool test_srv(shared_services::Mysrv::Request &req,
    shared_services::Mysrv::Response &res) {
    res.c = 123;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodebusinfo1");
    ros::NodeHandle node;

    auto pub = node.advertise<std_msgs::String>("test_topic", 100);

    auto srv = node.advertiseService("test_srv", test_srv);

    ros::spin();
    return 0;
}
