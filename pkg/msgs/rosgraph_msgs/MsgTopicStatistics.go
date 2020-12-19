package rosgraph_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type TopicStatistics struct { //nolint:golint
	msg.Package    `ros:"rosgraph_msgs"`
	Topic          string        //nolint:golint
	NodePub        string        //nolint:golint
	NodeSub        string        //nolint:golint
	WindowStart    time.Time     //nolint:golint
	WindowStop     time.Time     //nolint:golint
	DeliveredMsgs  int32         //nolint:golint
	DroppedMsgs    int32         //nolint:golint
	Traffic        int32         //nolint:golint
	PeriodMean     time.Duration //nolint:golint
	PeriodStddev   time.Duration //nolint:golint
	PeriodMax      time.Duration //nolint:golint
	StampAgeMean   time.Duration //nolint:golint
	StampAgeStddev time.Duration //nolint:golint
	StampAgeMax    time.Duration //nolint:golint
}
