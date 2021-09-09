//nolint:golint,lll
package actionlib

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TestActionGoal struct {
	Goal int32
}

type TestActionResult struct {
	Result int32
}

type TestActionFeedback struct {
	Feedback int32
}

type TestAction struct {
	msg.Package `ros:"actionlib"`
	TestActionGoal
	TestActionResult
	TestActionFeedback
}
