package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	InteractiveMarkerUpdate_KEEP_ALIVE uint8 = 0 //nolint:golint
	InteractiveMarkerUpdate_UPDATE     uint8 = 1 //nolint:golint
)

type InteractiveMarkerUpdate struct { //nolint:golint
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 KEEP_ALIVE=0,uint8 UPDATE=1"`
	ServerId        string                  //nolint:golint
	SeqNum          uint64                  //nolint:golint
	Type            uint8                   //nolint:golint
	Markers         []InteractiveMarker     //nolint:golint
	Poses           []InteractiveMarkerPose //nolint:golint
	Erases          []string                //nolint:golint
}
