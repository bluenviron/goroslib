package main

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

const testMsg1 = `Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
`

const testMsg2 = `uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
`

func TestRun(t *testing.T) {
	dir, err := os.MkdirTemp("", "goroslib")
	require.NoError(t, err)
	defer os.RemoveAll(dir)

	os.MkdirAll(filepath.Join(dir, "my_package", "msg"), 0o755)

	fpath := filepath.Join(dir, "my_package", "msg", "msg1.msg")
	err = os.WriteFile(fpath, []byte(testMsg1), 0o644)
	require.NoError(t, err)

	fpath = filepath.Join(dir, "my_package", "msg", "msg2.msg")
	err = os.WriteFile(fpath, []byte(testMsg2), 0o644)
	require.NoError(t, err)

	os.Mkdir(filepath.Join(dir, "output"), 0o755)
	os.Chdir(filepath.Join(dir, "output"))

	err = run([]string{filepath.Join(dir, "my_package")})
	require.NoError(t, err)

	_, err = os.Stat(filepath.Join(dir, "output", "my_package", "msg_msg1.go"))
	require.NoError(t, err)

	_, err = os.Stat(filepath.Join(dir, "output", "my_package", "msg_msg2.go"))
	require.NoError(t, err)
}
