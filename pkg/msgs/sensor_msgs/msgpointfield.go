package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	PointField_INT8    uint8 = 1 //nolint:golint
	PointField_UINT8   uint8 = 2 //nolint:golint
	PointField_INT16   uint8 = 3 //nolint:golint
	PointField_UINT16  uint8 = 4 //nolint:golint
	PointField_INT32   uint8 = 5 //nolint:golint
	PointField_UINT32  uint8 = 6 //nolint:golint
	PointField_FLOAT32 uint8 = 7 //nolint:golint
	PointField_FLOAT64 uint8 = 8 //nolint:golint
)

type PointField struct { //nolint:golint
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"uint8 INT8=1,uint8 UINT8=2,uint8 INT16=3,uint8 UINT16=4,uint8 INT32=5,uint8 UINT32=6,uint8 FLOAT32=7,uint8 FLOAT64=8"`
	Name            string //nolint:golint
	Offset          uint32 //nolint:golint
	Datatype        uint8  //nolint:golint
	Count           uint32 //nolint:golint
}
