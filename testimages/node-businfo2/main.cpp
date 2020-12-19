
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <shared_services/Mysrv.h>

void on_message(const std_msgs::String::ConstPtr& in) {
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodebusinfo2");
    ros::NodeHandle node;

    auto sub = node.subscribe("test_topic", 100, on_message,
        ros::TransportHints().udp());

    auto client = node.serviceClient<shared_services::Mysrv>("test_srv");

    shared_services::Mysrv::Request req;
    req.a = 123;
    req.b = "456";
    shared_services::Mysrv::Response res;
    client.call(req, res);

    ros::spin();
    return 0;
}
