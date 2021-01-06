package audio_common_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AudioData struct { //nolint:golint
	msg.Package `ros:"audio_common_msgs"`
	Data        []uint8 //nolint:golint
}
