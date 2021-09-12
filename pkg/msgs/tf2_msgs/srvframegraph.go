//nolint:golint,lll
package tf2_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FrameGraphReq struct {
	msg.Package `ros:"tf2_msgs"`
}

type FrameGraphRes struct {
	msg.Package `ros:"tf2_msgs"`
	FrameYaml   string
}

type FrameGraph struct {
	msg.Package `ros:"tf2_msgs"`
	FrameGraphReq
	FrameGraphRes
}
