#include <ros/ros.h>

#include <shared_services/Mysrv.h>

bool test_srv(shared_services::Mysrv::Request &req,
    shared_services::Mysrv::Response &res) {
    res.c = 123;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodeserviceprovider");
    ros::NodeHandle node;

    auto srv = node.advertiseService("test_srv", test_srv);

    ros::spin();
    return 0;
}
