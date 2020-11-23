
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodesetparam");
    ros::NodeHandle node;

    node.setParam("test_string", "ABC");
    node.setParam("/myns/test_int", 123);
    node.setParam("test_bool", true);

    ros::spin();
    return 0;
}
