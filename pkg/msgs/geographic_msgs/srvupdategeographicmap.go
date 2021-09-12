//nolint:golint,lll
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type UpdateGeographicMapReq struct {
	msg.Package `ros:"geographic_msgs"`
	Updates     GeographicMapChanges
}

type UpdateGeographicMapRes struct {
	msg.Package `ros:"geographic_msgs"`
	Success     bool
	Status      string
}

type UpdateGeographicMap struct {
	msg.Package `ros:"geographic_msgs"`
	UpdateGeographicMapReq
	UpdateGeographicMapRes
}
