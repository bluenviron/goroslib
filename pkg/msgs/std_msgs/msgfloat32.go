package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Float32 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        float32 //nolint:golint
}
