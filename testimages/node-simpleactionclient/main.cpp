
#include <actionlib/client/simple_action_client.h>

#include <shared_actions/DoSomethingAction.h>

void onDone(const actionlib::SimpleClientGoalState& state, const shared_actions::DoSomethingResultConstPtr& result) {
    printf("%s\n", state.toString().c_str());
    printf("%u\n", result->output);
    exit(0);
}

void onActive() {
}

void onFeedback(const shared_actions::DoSomethingFeedbackConstPtr& fb) {
    printf("%.2f\n", fb->percent_complete);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_actionclient");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionlib::SimpleActionClient<shared_actions::DoSomethingAction> client(
        node, "test_action");

    client.waitForServer();

    shared_actions::DoSomethingGoal gl;
    gl.input = 123456;
    client.sendGoal(gl, onDone, onActive, onFeedback);

    ros::waitForShutdown();
    return 0;
}
