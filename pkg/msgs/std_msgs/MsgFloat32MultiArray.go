package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Float32MultiArray struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Layout      MultiArrayLayout //nolint:golint
	Data        []float32        //nolint:golint
}
