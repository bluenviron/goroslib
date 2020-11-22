package rosgraph_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	Log_DEBUG int8 = 1
	Log_INFO  int8 = 2
	Log_WARN  int8 = 4
	Log_ERROR int8 = 8
	Log_FATAL int8 = 16
)

type Log struct {
	msg.Package     `ros:"rosgraph_msgs"`
	msg.Definitions `ros:"byte DEBUG=1,byte INFO=2,byte WARN=4,byte ERROR=8,byte FATAL=16"`
	Header          std_msgs.Header
	Level           int8 `rostype:"byte"`
	Name            string
	Msg             string
	File            string
	Function        string
	Line            uint32
	Topics          []string
}
