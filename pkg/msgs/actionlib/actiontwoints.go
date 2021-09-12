//nolint:golint,lll
package actionlib

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TwoIntsActionGoal struct {
	msg.Package `ros:"actionlib"`
	A           int64
	B           int64
}

type TwoIntsActionResult struct {
	msg.Package `ros:"actionlib"`
	Sum         int64
}

type TwoIntsActionFeedback struct {
	msg.Package `ros:"actionlib"`
}

type TwoIntsAction struct {
	msg.Package `ros:"actionlib"`
	TwoIntsActionGoal
	TwoIntsActionResult
	TwoIntsActionFeedback
}
