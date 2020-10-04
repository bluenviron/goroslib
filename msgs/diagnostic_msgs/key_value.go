package diagnostic_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type KeyValue struct {
	msgs.Package `ros:"diagnostic_msgs"`
	Key          string
	Value        string
}
