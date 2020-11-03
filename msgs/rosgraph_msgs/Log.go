package rosgraph_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
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
	msg.Definitions `ros:"int8 DEBUG=1,int8 INFO=2,int8 WARN=4,int8 ERROR=8,int8 FATAL=16"`
	Header          std_msgs.Header
	Level           int8 `rostype:"byte"`
	Name            string
	Msg             string
	File            string
	Function        string
	Line            uint32
	Topics          []string
}
