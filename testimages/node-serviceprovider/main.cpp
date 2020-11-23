
#include <ros/ros.h>

#include <nodeservice_srv/Mysrv.h>

bool test_srv(nodeservice_srv::Mysrv::Request &req,
    nodeservice_srv::Mysrv::Response &res) {
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
