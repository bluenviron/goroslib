package rosgraph_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	Log_DEBUG int8 = 1  //nolint:golint
	Log_INFO  int8 = 2  //nolint:golint
	Log_WARN  int8 = 4  //nolint:golint
	Log_ERROR int8 = 8  //nolint:golint
	Log_FATAL int8 = 16 //nolint:golint
)

type Log struct { //nolint:golint
	msg.Package     `ros:"rosgraph_msgs"`
	msg.Definitions `ros:"byte DEBUG=1,byte INFO=2,byte WARN=4,byte ERROR=8,byte FATAL=16"`
	Header          std_msgs.Header //nolint:golint
	Level           int8            `rostype:"byte"` //nolint:golint
	Name            string          //nolint:golint
	Msg             string          //nolint:golint
	File            string          //nolint:golint
	Function        string          //nolint:golint
	Line            uint32          //nolint:golint
	Topics          []string        //nolint:golint
}
