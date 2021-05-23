//nolint:golint
package actionlib

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TwoIntsActionGoal struct {
	A int64
	B int64
}

type TwoIntsActionResult struct {
	Sum int64
}

type TwoIntsActionFeedback struct{}

type TwoIntsAction struct {
	msg.Package `ros:"actionlib"`
	TwoIntsActionGoal
	TwoIntsActionResult
	TwoIntsActionFeedback
}
