package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

const (
	ImageMarker_CIRCLE     uint8 = 0 //nolint:golint
	ImageMarker_LINE_STRIP uint8 = 1 //nolint:golint
	ImageMarker_LINE_LIST  uint8 = 2 //nolint:golint
	ImageMarker_POLYGON    uint8 = 3 //nolint:golint
	ImageMarker_POINTS     uint8 = 4 //nolint:golint
	ImageMarker_ADD        uint8 = 0 //nolint:golint
	ImageMarker_REMOVE     uint8 = 1 //nolint:golint
)

type ImageMarker struct { //nolint:golint
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 CIRCLE=0,uint8 LINE_STRIP=1,uint8 LINE_LIST=2,uint8 POLYGON=3,uint8 POINTS=4,uint8 ADD=0,uint8 REMOVE=1"`
	Header          std_msgs.Header       //nolint:golint
	Ns              string                //nolint:golint
	Id              int32                 //nolint:golint
	Type            int32                 //nolint:golint
	Action          int32                 //nolint:golint
	Position        geometry_msgs.Point   //nolint:golint
	Scale           float32               //nolint:golint
	OutlineColor    std_msgs.ColorRGBA    //nolint:golint
	Filled          uint8                 //nolint:golint
	FillColor       std_msgs.ColorRGBA    //nolint:golint
	Lifetime        time.Duration         //nolint:golint
	Points          []geometry_msgs.Point //nolint:golint
	OutlineColors   []std_msgs.ColorRGBA  //nolint:golint
}
