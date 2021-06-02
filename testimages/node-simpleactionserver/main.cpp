
#include <actionlib/server/simple_action_server.h>

#include <shared_actions/DoSomethingAction.h>

actionlib::SimpleActionServer<shared_actions::DoSomethingAction>* server;

void onExecute(const shared_actions::DoSomethingGoalConstPtr& goal) {
    if (goal->input == 3) {
        ros::Duration(2.0).sleep();
        if (server->isPreemptRequested()) {
            server->setAborted();
            return;
        }
    }

    ros::Duration(0.5).sleep();

    shared_actions::DoSomethingFeedback fb;
    fb.percent_complete = 0.5;
    server->publishFeedback(fb);

    ros::Duration(0.5).sleep();

    if (goal->input == 2) {
        server->setAborted();
        return;
    }

    shared_actions::DoSomethingResult res;
    res.output = 123456;
    server->setSucceeded(res);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_actionserver");
    ros::NodeHandle node;

    server = new actionlib::SimpleActionServer<shared_actions::DoSomethingAction>(
        node, "test_action", onExecute, false);
    server->start();

    ros::spin();
    return 0;
}
