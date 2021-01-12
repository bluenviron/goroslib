package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

const (
	Marker_ARROW            uint8 = 0  //nolint:golint
	Marker_CUBE             uint8 = 1  //nolint:golint
	Marker_SPHERE           uint8 = 2  //nolint:golint
	Marker_CYLINDER         uint8 = 3  //nolint:golint
	Marker_LINE_STRIP       uint8 = 4  //nolint:golint
	Marker_LINE_LIST        uint8 = 5  //nolint:golint
	Marker_CUBE_LIST        uint8 = 6  //nolint:golint
	Marker_SPHERE_LIST      uint8 = 7  //nolint:golint
	Marker_POINTS           uint8 = 8  //nolint:golint
	Marker_TEXT_VIEW_FACING uint8 = 9  //nolint:golint
	Marker_MESH_RESOURCE    uint8 = 10 //nolint:golint
	Marker_TRIANGLE_LIST    uint8 = 11 //nolint:golint
	Marker_ADD              uint8 = 0  //nolint:golint
	Marker_MODIFY           uint8 = 0  //nolint:golint
	Marker_DELETE           uint8 = 2  //nolint:golint
	Marker_DELETEALL        uint8 = 3  //nolint:golint
)

type Marker struct { //nolint:golint
	msg.Package              `ros:"visualization_msgs"`
	msg.Definitions          `ros:"uint8 ARROW=0,uint8 CUBE=1,uint8 SPHERE=2,uint8 CYLINDER=3,uint8 LINE_STRIP=4,uint8 LINE_LIST=5,uint8 CUBE_LIST=6,uint8 SPHERE_LIST=7,uint8 POINTS=8,uint8 TEXT_VIEW_FACING=9,uint8 MESH_RESOURCE=10,uint8 TRIANGLE_LIST=11,uint8 ADD=0,uint8 MODIFY=0,uint8 DELETE=2,uint8 DELETEALL=3"`
	Header                   std_msgs.Header       //nolint:golint
	Ns                       string                //nolint:golint
	Id                       int32                 //nolint:golint
	Type                     int32                 //nolint:golint
	Action                   int32                 //nolint:golint
	Pose                     geometry_msgs.Pose    //nolint:golint
	Scale                    geometry_msgs.Vector3 //nolint:golint
	Color                    std_msgs.ColorRGBA    //nolint:golint
	Lifetime                 time.Duration         //nolint:golint
	FrameLocked              bool                  //nolint:golint
	Points                   []geometry_msgs.Point //nolint:golint
	Colors                   []std_msgs.ColorRGBA  //nolint:golint
	Text                     string                //nolint:golint
	MeshResource             string                //nolint:golint
	MeshUseEmbeddedMaterials bool                  //nolint:golint
}
