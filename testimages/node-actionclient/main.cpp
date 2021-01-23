
#include <actionlib/action_definition.h>
#include <actionlib/client/action_client.h>

#include <shared_actions/DoSomethingAction.h>

void onTransition(actionlib::ClientGoalHandle<shared_actions::DoSomethingAction> gh) {
    if (gh.getCommState() == actionlib::CommState::StateEnum::DONE) {
        printf("%s\n", gh.getTerminalState().toString().c_str());
        printf("%u\n", gh.getResult()->output);
        exit(0);
    }
}

void onFeedback(actionlib::ClientGoalHandle<shared_actions::DoSomethingAction> gh,
    const shared_actions::DoSomethingFeedback::ConstPtr& f) {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_actionclient");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionlib::ActionClient<shared_actions::DoSomethingAction> client(
        node, "test_action");

    client.waitForActionServerToStart();

    shared_actions::DoSomethingGoal gl;
    gl.input = 123456;
    auto gh = client.sendGoal(gl, onTransition, onFeedback);

    ros::waitForShutdown();
    return 0;
}
