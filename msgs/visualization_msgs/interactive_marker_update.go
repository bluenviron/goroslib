package visualization_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type InteractiveMarkerUpdate struct {
	msgs.Package     `ros:"visualization_msgs"`
	msgs.Definitions `ros:"uint8 KEEP_ALIVE=0,uint8 UPDATE=1"`
	ServerId         string
	SeqNum           uint64
	Type             uint8
	Markers          []InteractiveMarker
	Poses            []InteractiveMarkerPose
	Erases           []string
}
