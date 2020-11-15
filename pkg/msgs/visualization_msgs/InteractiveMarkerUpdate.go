package visualization_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	InteractiveMarkerUpdate_KEEP_ALIVE uint8 = 0
	InteractiveMarkerUpdate_UPDATE     uint8 = 1
)

type InteractiveMarkerUpdate struct {
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 KEEP_ALIVE=0,uint8 UPDATE=1"`
	ServerId        string
	SeqNum          uint64
	Type            uint8
	Markers         []InteractiveMarker
	Poses           []InteractiveMarkerPose
	Erases          []string
}
