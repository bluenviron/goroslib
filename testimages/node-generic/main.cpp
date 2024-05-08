#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodegeneric");
    ros::NodeHandle node;

    ros::spin();
    return 0;
}
