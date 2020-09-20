
#include <ros/ros.h>

#include <nodeservice_srv/Mysrv.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "nodeserviceclient");
    ros::NodeHandle node("~");

    auto client = node.serviceClient<nodeservice_srv::Mysrv>("/test_srv");

    nodeservice_srv::Mysrv::Request req;
    req.a = 123;
    req.b = "456";
    nodeservice_srv::Mysrv::Response res;
    client.call(req, res);

    ros::spin();
    return 0;
}
