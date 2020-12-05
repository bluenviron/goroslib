//nolint:golint
package visualization_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

const (
	ImageMarker_CIRCLE     uint8 = 0
	ImageMarker_LINE_STRIP uint8 = 1
	ImageMarker_LINE_LIST  uint8 = 2
	ImageMarker_POLYGON    uint8 = 3
	ImageMarker_POINTS     uint8 = 4
	ImageMarker_ADD        uint8 = 0
	ImageMarker_REMOVE     uint8 = 1
)

type ImageMarker struct {
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 CIRCLE=0,uint8 LINE_STRIP=1,uint8 LINE_LIST=2,uint8 POLYGON=3,uint8 POINTS=4,uint8 ADD=0,uint8 REMOVE=1"`
	Header          std_msgs.Header
	Ns              string
	Id              int32
	Type            int32
	Action          int32
	Position        geometry_msgs.Point
	Scale           float32
	OutlineColor    std_msgs.ColorRGBA
	Filled          uint8
	FillColor       std_msgs.ColorRGBA
	Lifetime        time.Duration
	Points          []geometry_msgs.Point
	OutlineColors   []std_msgs.ColorRGBA
}
