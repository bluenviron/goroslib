package shape_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type SolidPrimitive struct {
	msg.Package     `ros:"shape_msgs"`
	msg.Definitions `ros:"uint8 BOX=1,uint8 SPHERE=2,uint8 CYLINDER=3,uint8 CONE=4,uint8 BOX_X=0,uint8 BOX_Y=1,uint8 BOX_Z=2,uint8 SPHERE_RADIUS=0,uint8 CYLINDER_HEIGHT=0,uint8 CYLINDER_RADIUS=1,uint8 CONE_HEIGHT=0,uint8 CONE_RADIUS=1"`
	Type            uint8
	Dimensions      []float64
}
