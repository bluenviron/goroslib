//nolint:golint
package visualization_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

const (
	Marker_ARROW            uint8 = 0
	Marker_CUBE             uint8 = 1
	Marker_SPHERE           uint8 = 2
	Marker_CYLINDER         uint8 = 3
	Marker_LINE_STRIP       uint8 = 4
	Marker_LINE_LIST        uint8 = 5
	Marker_CUBE_LIST        uint8 = 6
	Marker_SPHERE_LIST      uint8 = 7
	Marker_POINTS           uint8 = 8
	Marker_TEXT_VIEW_FACING uint8 = 9
	Marker_MESH_RESOURCE    uint8 = 10
	Marker_TRIANGLE_LIST    uint8 = 11
	Marker_ADD              uint8 = 0
	Marker_MODIFY           uint8 = 0
	Marker_DELETE           uint8 = 2
	Marker_DELETEALL        uint8 = 3
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
