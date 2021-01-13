package actionlib //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TestActionGoal struct { //nolint:golint
	Goal int32 //nolint:golint
}

type TestActionResult struct { //nolint:golint
	Result int32 //nolint:golint
}

type TestActionFeedback struct { //nolint:golint
	Feedback int32 //nolint:golint
}

type TestAction struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	TestActionGoal
	TestActionResult
	TestActionFeedback
}
