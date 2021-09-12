//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type FileChecksumReq struct {
	msg.Package `ros:"mavros_msgs"`
	FilePath    string
}

type FileChecksumRes struct {
	msg.Package `ros:"mavros_msgs"`
	Crc32       uint32
	Success     bool
	RErrno      int32
}

type FileChecksum struct {
	msg.Package `ros:"mavros_msgs"`
	FileChecksumReq
	FileChecksumRes
}
