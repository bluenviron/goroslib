package actionlib //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

const (
	TestRequestActionGoal_TERMINATE_SUCCESS   int32 = 0 //nolint:golint
	TestRequestActionGoal_TERMINATE_ABORTED   int32 = 1 //nolint:golint
	TestRequestActionGoal_TERMINATE_REJECTED  int32 = 2 //nolint:golint
	TestRequestActionGoal_TERMINATE_LOSE      int32 = 3 //nolint:golint
	TestRequestActionGoal_TERMINATE_DROP      int32 = 4 //nolint:golint
	TestRequestActionGoal_TERMINATE_EXCEPTION int32 = 5 //nolint:golint
)

type TestRequestActionGoal struct { //nolint:golint
	msg.Definitions `ros:"int32 TERMINATE_SUCCESS=0,int32 TERMINATE_ABORTED=1,int32 TERMINATE_REJECTED=2,int32 TERMINATE_LOSE=3,int32 TERMINATE_DROP=4,int32 TERMINATE_EXCEPTION=5"`
	TerminateStatus int32         //nolint:golint
	IgnoreCancel    bool          //nolint:golint
	ResultText      string        //nolint:golint
	TheResult       int32         //nolint:golint
	IsSimpleClient  bool          //nolint:golint
	DelayAccept     time.Duration //nolint:golint
	DelayTerminate  time.Duration //nolint:golint
	PauseStatus     time.Duration //nolint:golint
}

type TestRequestActionResult struct { //nolint:golint
	TheResult      int32 //nolint:golint
	IsSimpleServer bool  //nolint:golint
}

type TestRequestActionFeedback struct { //nolint:golint
}

type TestRequestAction struct { //nolint:golint
	msg.Package `ros:"actionlib"`
	TestRequestActionGoal
	TestRequestActionResult
	TestRequestActionFeedback
}
