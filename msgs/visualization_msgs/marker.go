package visualization_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
	"time"
)

type Marker struct {
	msg.Package              `ros:"visualization_msgs"`
	msg.Definitions          `ros:"uint8 ARROW=0,uint8 CUBE=1,uint8 SPHERE=2,uint8 CYLINDER=3,uint8 LINE_STRIP=4,uint8 LINE_LIST=5,uint8 CUBE_LIST=6,uint8 SPHERE_LIST=7,uint8 POINTS=8,uint8 TEXT_VIEW_FACING=9,uint8 MESH_RESOURCE=10,uint8 TRIANGLE_LIST=11,uint8 ADD=0,uint8 MODIFY=0,uint8 DELETE=2,uint8 DELETEALL=3"`
	Header                   std_msgs.Header
	Ns                       string
	Id                       int32
	Type                     int32
	Action                   int32
	Pose                     geometry_msgs.Pose
	Scale                    geometry_msgs.Vector3
	Color                    std_msgs.ColorRGBA
	Lifetime                 time.Duration
	FrameLocked              bool
	Points                   []geometry_msgs.Point
	Colors                   []std_msgs.ColorRGBA
	Text                     string
	MeshResource             string
	MeshUseEmbeddedMaterials bool
}
