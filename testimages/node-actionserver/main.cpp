
#include <thread>

#include <actionlib/server/action_server.h>

#include <shared_actions/DoSomethingAction.h>

void handleGoal(actionlib::ServerGoalHandle<shared_actions::DoSomethingAction> gh) {
    if (gh.getGoal()->input == 1) {
        gh.setRejected();
        return;
    }
    gh.setAccepted();

    if (gh.getGoal()->input == 3) {
        return;
    }

    ros::Duration(0.5).sleep();

    shared_actions::DoSomethingFeedback fb;
    fb.percent_complete = 0.5;
    gh.publishFeedback(fb);

    ros::Duration(0.5).sleep();

    if (gh.getGoal()->input == 2) {
        gh.setAborted();
        return;
    }

    shared_actions::DoSomethingResult res;
    res.output = 123456;
    gh.setSucceeded(res);
}

std::list<std::thread> threads;

void onGoal(actionlib::ServerGoalHandle<shared_actions::DoSomethingAction> gh) {
    threads.emplace_back(handleGoal, gh);
}

void onCancel(actionlib::ServerGoalHandle<shared_actions::DoSomethingAction> gh) {
    gh.setCanceled();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_actionserver");
    ros::NodeHandle node;

    actionlib::ActionServer<shared_actions::DoSomethingAction> server(
        node, "test_action", onGoal, onCancel, false);
    server.start();

    ros::spin();
    return 0;
}
