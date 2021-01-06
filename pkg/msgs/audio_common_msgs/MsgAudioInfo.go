package audio_common_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AudioInfo struct { //nolint:golint
	msg.Package  `ros:"audio_common_msgs"`
	Channels     uint8  //nolint:golint
	SampleRate   uint32 //nolint:golint
	SampleFormat string //nolint:golint
	Bitrate      uint32 //nolint:golint
	CodingFormat string //nolint:golint
}
