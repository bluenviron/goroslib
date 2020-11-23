
#include <ros/ros.h>
#include <inttypes.h>

#include <std_msgs/Int64MultiArray.h>

void on_message(const std_msgs::Int64MultiArray::ConstPtr& in) {
    printf("%d ", in->data.size());

    for (auto it = in->data.begin(); it != in->data.end(); it++) {
        printf("%" PRId64 " ", *it);
    }

    printf("\n");
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodesub");
    ros::NodeHandle node;

    auto sub = node.subscribe("test_topic", 100, on_message, ros::TransportHints().udp());

    ros::spin();
    return 0;
}
