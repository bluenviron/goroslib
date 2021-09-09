//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	RTKBaseline_RTK_BASELINE_COORDINATE_SYSTEM_ECEF uint8 = 0
	RTKBaseline_RTK_BASELINE_COORDINATE_SYSTEM_NED  uint8 = 1
)

type RTKBaseline struct {
	msg.Package        `ros:"mavros_msgs"`
	msg.Definitions    `ros:"uint8 RTK_BASELINE_COORDINATE_SYSTEM_ECEF=0,uint8 RTK_BASELINE_COORDINATE_SYSTEM_NED=1"`
	Header             std_msgs.Header
	TimeLastBaselineMs uint32
	RtkReceiverId      uint8
	Wn                 uint16
	Tow                uint32
	RtkHealth          uint8
	RtkRate            uint8
	Nsats              uint8
	BaselineCoordsType uint8
	BaselineAMm        int32
	BaselineBMm        int32
	BaselineCMm        int32
	Accuracy           uint32
	IarNumHypotheses   int32
}
