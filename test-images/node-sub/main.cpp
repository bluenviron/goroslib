
#include <ros/ros.h>

#include <nodesub/Mymsg.h>


void on_message(const nodesub::Mymsg::ConstPtr& in) {
    ROS_INFO("in: %d", in->a);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodesub");
    ros::NodeHandle node("~");

    auto sub = node.subscribe("/test_pub", 100, on_message);

    ros::spin();
    return 0;
}
