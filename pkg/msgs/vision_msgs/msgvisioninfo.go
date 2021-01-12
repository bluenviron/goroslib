package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type VisionInfo struct { //nolint:golint
	msg.Package      `ros:"vision_msgs"`
	Header           std_msgs.Header //nolint:golint
	Method           string          //nolint:golint
	DatabaseLocation string          //nolint:golint
	DatabaseVersion  int32           //nolint:golint
}
