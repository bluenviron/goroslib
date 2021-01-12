package shape_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	SolidPrimitive_BOX             uint8 = 1 //nolint:golint
	SolidPrimitive_SPHERE          uint8 = 2 //nolint:golint
	SolidPrimitive_CYLINDER        uint8 = 3 //nolint:golint
	SolidPrimitive_CONE            uint8 = 4 //nolint:golint
	SolidPrimitive_BOX_X           uint8 = 0 //nolint:golint
	SolidPrimitive_BOX_Y           uint8 = 1 //nolint:golint
	SolidPrimitive_BOX_Z           uint8 = 2 //nolint:golint
	SolidPrimitive_SPHERE_RADIUS   uint8 = 0 //nolint:golint
	SolidPrimitive_CYLINDER_HEIGHT uint8 = 0 //nolint:golint
	SolidPrimitive_CYLINDER_RADIUS uint8 = 1 //nolint:golint
	SolidPrimitive_CONE_HEIGHT     uint8 = 0 //nolint:golint
	SolidPrimitive_CONE_RADIUS     uint8 = 1 //nolint:golint
)

type SolidPrimitive struct { //nolint:golint
	msg.Package     `ros:"shape_msgs"`
	msg.Definitions `ros:"uint8 BOX=1,uint8 SPHERE=2,uint8 CYLINDER=3,uint8 CONE=4,uint8 BOX_X=0,uint8 BOX_Y=1,uint8 BOX_Z=2,uint8 SPHERE_RADIUS=0,uint8 CYLINDER_HEIGHT=0,uint8 CYLINDER_RADIUS=1,uint8 CONE_HEIGHT=0,uint8 CONE_RADIUS=1"`
	Type            uint8     //nolint:golint
	Dimensions      []float64 //nolint:golint
}
