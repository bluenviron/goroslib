package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type PointCloud2 struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header //nolint:golint
	Height      uint32          //nolint:golint
	Width       uint32          //nolint:golint
	Fields      []PointField    //nolint:golint
	IsBigendian bool            //nolint:golint
	PointStep   uint32          //nolint:golint
	RowStep     uint32          //nolint:golint
	Data        []uint8         //nolint:golint
	IsDense     bool            //nolint:golint
}
