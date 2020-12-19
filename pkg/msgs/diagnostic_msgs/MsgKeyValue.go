package diagnostic_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type KeyValue struct { //nolint:golint
	msg.Package `ros:"diagnostic_msgs"`
	Key         string //nolint:golint
	Value       string //nolint:golint
}
