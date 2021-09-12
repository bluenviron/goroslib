//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FileCloseReq struct {
	msg.Package `ros:"mavros_msgs"`
	FilePath    string
}

type FileCloseRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
	RErrno      int32
}

type FileClose struct {
	msg.Package `ros:"mavros_msgs"`
	FileCloseReq
	FileCloseRes
}
