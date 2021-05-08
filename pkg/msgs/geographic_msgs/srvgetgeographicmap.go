//nolint:golint
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetGeographicMapReq struct {
	Url    string
	Bounds BoundingBox
}

type GetGeographicMapRes struct {
	Success bool
	Status  string
	Map     GeographicMap
}

type GetGeographicMap struct {
	msg.Package `ros:"geographic_msgs"`
	GetGeographicMapReq
	GetGeographicMapRes
}
