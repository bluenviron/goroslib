package rosgraph_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type TopicStatistics struct {
	msg.Package    `ros:"rosgraph_msgs"`
	Topic          string
	NodePub        string
	NodeSub        string
	WindowStart    time.Time
	WindowStop     time.Time
	DeliveredMsgs  int32
	DroppedMsgs    int32
	Traffic        int32
	PeriodMean     time.Duration
	PeriodStddev   time.Duration
	PeriodMax      time.Duration
	StampAgeMean   time.Duration
	StampAgeStddev time.Duration
	StampAgeMax    time.Duration
}
