//nolint:golint
package tf

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FrameGraphReq struct{}

type FrameGraphRes struct {
	DotGraph string
}

type FrameGraph struct {
	msg.Package `ros:"tf"`
	FrameGraphReq
	FrameGraphRes
}
