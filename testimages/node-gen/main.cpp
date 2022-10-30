#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodegen");
    ros::NodeHandle node;

    ros::spin();
    return 0;
}
