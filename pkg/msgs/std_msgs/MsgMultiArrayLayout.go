package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MultiArrayLayout struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Dim         []MultiArrayDimension //nolint:golint
	DataOffset  uint32                //nolint:golint
}
