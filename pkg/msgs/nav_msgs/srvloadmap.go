//nolint:golint,lll
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LoadMapReq struct {
	MapUrl string
}

const (
	LoadMapRes_RESULT_SUCCESS              uint8 = 0
	LoadMapRes_RESULT_MAP_DOES_NOT_EXIST   uint8 = 1
	LoadMapRes_RESULT_INVALID_MAP_DATA     uint8 = 2
	LoadMapRes_RESULT_INVALID_MAP_METADATA uint8 = 3
	LoadMapRes_RESULT_UNDEFINED_FAILURE    uint8 = 255
)

type LoadMapRes struct {
	msg.Definitions `ros:"uint8 RESULT_SUCCESS=0,uint8 RESULT_MAP_DOES_NOT_EXIST=1,uint8 RESULT_INVALID_MAP_DATA=2,uint8 RESULT_INVALID_MAP_METADATA=3,uint8 RESULT_UNDEFINED_FAILURE=255"`
	Map             OccupancyGrid
	Result          uint8
}

type LoadMap struct {
	msg.Package `ros:"nav_msgs"`
	LoadMapReq
	LoadMapRes
}
