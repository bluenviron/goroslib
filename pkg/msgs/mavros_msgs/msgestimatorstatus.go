//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type EstimatorStatus struct {
	msg.Package               `ros:"mavros_msgs"`
	Header                    std_msgs.Header
	AttitudeStatusFlag        bool
	VelocityHorizStatusFlag   bool
	VelocityVertStatusFlag    bool
	PosHorizRelStatusFlag     bool
	PosHorizAbsStatusFlag     bool
	PosVertAbsStatusFlag      bool
	PosVertAglStatusFlag      bool
	ConstPosModeStatusFlag    bool
	PredPosHorizRelStatusFlag bool
	PredPosHorizAbsStatusFlag bool
	GpsGlitchStatusFlag       bool
	AccelErrorStatusFlag      bool
}
