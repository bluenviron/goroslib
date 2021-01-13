package actionlib //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TwoIntsActionGoal struct { //nolint:golint
	A int64 //nolint:golint
	B int64 //nolint:golint
}

type TwoIntsActionResult struct { //nolint:golint
	Sum int64 //nolint:golint
}

type TwoIntsActionFeedback struct { //nolint:golint
}

type TwoIntsAction struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	TwoIntsActionGoal
	TwoIntsActionResult
	TwoIntsActionFeedback
}
