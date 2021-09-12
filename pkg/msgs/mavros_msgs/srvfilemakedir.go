//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FileMakeDirReq struct {
	msg.Package `ros:"mavros_msgs"`
	DirPath     string
}

type FileMakeDirRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
	RErrno      int32
}

type FileMakeDir struct {
	msg.Package `ros:"mavros_msgs"`
	FileMakeDirReq
	FileMakeDirRes
}
