
#include <ros/ros.h>
#include <thread>

#include <std_msgs/Int64MultiArray.h>


void publisher_run(ros::Publisher* pub) {
    ros::Rate rate(1);

    while(true) {
        std_msgs::Int64MultiArray out;

        for (int64_t i = 0; i < 400; i++) {
            out.data.push_back(i);
        }

        pub->publish(out);
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodepub");
    ros::NodeHandle node("~");

    // advertise a publisher and publish with a fixed rate
    auto pub = node.advertise<std_msgs::Int64MultiArray>("/test_pub", 100);
    std::thread pub_thread(publisher_run, &pub);

    ros::spin();
    return 0;
}
