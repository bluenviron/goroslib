package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LoadMapReq struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	MapUrl      string //nolint:golint
}

const (
	LoadMapRes_RESULT_SUCCESS              uint8 = 0   //nolint:golint
	LoadMapRes_RESULT_MAP_DOES_NOT_EXIST   uint8 = 1   //nolint:golint
	LoadMapRes_RESULT_INVALID_MAP_DATA     uint8 = 2   //nolint:golint
	LoadMapRes_RESULT_INVALID_MAP_METADATA uint8 = 3   //nolint:golint
	LoadMapRes_RESULT_UNDEFINED_FAILURE    uint8 = 255 //nolint:golint
)

type LoadMapRes struct { //nolint:golint
	msg.Package     `ros:"nav_msgs"`
	msg.Definitions `ros:"uint8 RESULT_SUCCESS=0,uint8 RESULT_MAP_DOES_NOT_EXIST=1,uint8 RESULT_INVALID_MAP_DATA=2,uint8 RESULT_INVALID_MAP_METADATA=3,uint8 RESULT_UNDEFINED_FAILURE=255"`
	Map             OccupancyGrid //nolint:golint
	Result          uint8         //nolint:golint
}
