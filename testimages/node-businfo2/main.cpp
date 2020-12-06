
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <nodeservice_srv/Mysrv.h>

void on_message(const std_msgs::String::ConstPtr& in) {
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodebusinfo2");
    ros::NodeHandle node;

    auto sub = node.subscribe("test_topic", 100, on_message,
        ros::TransportHints().udp());

    auto client = node.serviceClient<nodeservice_srv::Mysrv>("test_srv");

    nodeservice_srv::Mysrv::Request req;
    req.a = 123;
    req.b = "456";
    nodeservice_srv::Mysrv::Response res;
    client.call(req, res);

    ros::spin();
    return 0;
}
