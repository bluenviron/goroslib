
#include <ros/ros.h>

#include <shared_services/Mysrv.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodeserviceclient");
    ros::NodeHandle node;

    auto client = node.serviceClient<shared_services::Mysrv>("test_srv");

    shared_services::Mysrv::Request req;
    req.a = 123;
    req.b = "456";
    shared_services::Mysrv::Response res;
    client.call(req, res);
    printf("%f\n", res.c);

    return 0;
}
