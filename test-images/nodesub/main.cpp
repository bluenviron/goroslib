
#include <ros/ros.h>
#include <thread>

#include <nodesub/Mymsg.h>


void on_message(const nodesub::Mymsg::ConstPtr& in) {
    //ROS_INFO("in: %f %f %f", in->position.x, in->position.y, in->position.z);
    //ROS_INFO("%u", in->prova);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodesub");
    ros::NodeHandle node("~");

    auto sub = node.subscribe("/test_pub", 100, on_message);

    ros::spin();
    return 0;
}
