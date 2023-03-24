//autogenerated:yes
//nolint:revive,lll
package actionlib

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type TestActionGoal struct {
	msg.Package `ros:"actionlib"`
	Goal        int32
}

type TestActionResult struct {
	msg.Package `ros:"actionlib"`
	Result      int32
}

type TestActionFeedback struct {
	msg.Package `ros:"actionlib"`
	Feedback    int32
}

type TestAction struct {
	msg.Package `ros:"actionlib"`
	TestActionGoal
	TestActionResult
	TestActionFeedback
}
