
#include <ros/ros.h>
#include <thread>

#include <nodepub/Mymsg.h>

void publisher_run(ros::Publisher* pub) {
    ros::Rate rate(1);

    while(true) {
        nodepub::Mymsg out;

        out.a = 1;

        nodepub::Parent a;
        a.a = "other test";
        a.b = ros::Time(1500, 1345);
        a.c = true;
        a.d = 27;
        a.e = 23;
        a.f = ros::Duration(2345.50); //secs
        out.b.push_back(a);

        nodepub::Parent ca;
        ca.a = "AA";
        out.c[0] = ca;

        nodepub::Parent cb;
        cb.a = "BB";
        out.c[1] = cb;

        out.d[0] = 222;
        out.d[1] = 333;

        pub->publish(out);
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodepub");
    ros::NodeHandle node;

    // advertise a publisher and publish with a fixed rate
    auto pub = node.advertise<nodepub::Mymsg>("test_topic", 100);
    std::thread pub_thread(publisher_run, &pub);

    ros::spin();
    return 0;
}
