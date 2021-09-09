//nolint:golint,lll
package shape_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	SolidPrimitive_BOX             uint8 = 1
	SolidPrimitive_SPHERE          uint8 = 2
	SolidPrimitive_CYLINDER        uint8 = 3
	SolidPrimitive_CONE            uint8 = 4
	SolidPrimitive_BOX_X           uint8 = 0
	SolidPrimitive_BOX_Y           uint8 = 1
	SolidPrimitive_BOX_Z           uint8 = 2
	SolidPrimitive_SPHERE_RADIUS   uint8 = 0
	SolidPrimitive_CYLINDER_HEIGHT uint8 = 0
	SolidPrimitive_CYLINDER_RADIUS uint8 = 1
	SolidPrimitive_CONE_HEIGHT     uint8 = 0
	SolidPrimitive_CONE_RADIUS     uint8 = 1
)

type SolidPrimitive struct {
	msg.Package     `ros:"shape_msgs"`
	msg.Definitions `ros:"uint8 BOX=1,uint8 SPHERE=2,uint8 CYLINDER=3,uint8 CONE=4,uint8 BOX_X=0,uint8 BOX_Y=1,uint8 BOX_Z=2,uint8 SPHERE_RADIUS=0,uint8 CYLINDER_HEIGHT=0,uint8 CYLINDER_RADIUS=1,uint8 CONE_HEIGHT=0,uint8 CONE_RADIUS=1"`
	Type            uint8
	Dimensions      []float64
}
