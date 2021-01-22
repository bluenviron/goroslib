package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/msgs/trajectory_msgs"
	"time"
)

type FollowJointTrajectoryActionGoal struct { //nolint:golint
	Trajectory        trajectory_msgs.JointTrajectory //nolint:golint
	PathTolerance     []JointTolerance                //nolint:golint
	GoalTolerance     []JointTolerance                //nolint:golint
	GoalTimeTolerance time.Duration                   //nolint:golint
}

const (
	FollowJointTrajectoryActionResult_SUCCESSFUL              int32 = 0  //nolint:golint
	FollowJointTrajectoryActionResult_INVALID_GOAL            int32 = -1 //nolint:golint
	FollowJointTrajectoryActionResult_INVALID_JOINTS          int32 = -2 //nolint:golint
	FollowJointTrajectoryActionResult_OLD_HEADER_TIMESTAMP    int32 = -3 //nolint:golint
	FollowJointTrajectoryActionResult_PATH_TOLERANCE_VIOLATED int32 = -4 //nolint:golint
	FollowJointTrajectoryActionResult_GOAL_TOLERANCE_VIOLATED int32 = -5 //nolint:golint
)

type FollowJointTrajectoryActionResult struct { //nolint:golint
	msg.Definitions `ros:"int32 SUCCESSFUL=0,int32 INVALID_GOAL=-1,int32 INVALID_JOINTS=-2,int32 OLD_HEADER_TIMESTAMP=-3,int32 PATH_TOLERANCE_VIOLATED=-4,int32 GOAL_TOLERANCE_VIOLATED=-5"`
	ErrorCode       int32  //nolint:golint
	ErrorString     string //nolint:golint
}

type FollowJointTrajectoryActionFeedback struct { //nolint:golint
	Header     std_msgs.Header                      //nolint:golint
	JointNames []string                             //nolint:golint
	Desired    trajectory_msgs.JointTrajectoryPoint //nolint:golint
	Actual     trajectory_msgs.JointTrajectoryPoint //nolint:golint
	Error      trajectory_msgs.JointTrajectoryPoint //nolint:golint
}

type FollowJointTrajectoryAction struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	FollowJointTrajectoryActionGoal
	FollowJointTrajectoryActionResult
	FollowJointTrajectoryActionFeedback
}
