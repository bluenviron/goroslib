package msg

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

type Parent struct {
	A string
}

type Header struct { //nolint:golint
	Package `ros:"std_msgs"`
	Seq     uint32    //nolint:golint
	Stamp   time.Time //nolint:golint
	FrameId string    //nolint:golint
}

type Log struct {
	Package     `ros:"rosgraph_msgs"`
	Definitions `ros:"byte DEBUG=1,byte INFO=2,byte WARN=4,byte ERROR=8,byte FATAL=16"`
	Header      Header
	Level       int8 `rostype:"byte"`
	Name        string
	Msg         string
	File        string
	Function    string
	Line        uint32
	Topics      []string
}

func TestMD5(t *testing.T) {
	for _, ca := range []struct {
		name string
		msg  interface{}
		sum  string
	}{
		{
			"base types",
			&struct {
				Package `ros:"testing"`
				A       bool
				B       int8
				C       uint8
				D       int16
				E       uint16
				F       int32
				G       uint32
				H       int64
				I       uint64
				J       float32
				K       float64
				L       string
				M       time.Time
				N       time.Duration
				O       int8  `rostype:"byte"`
				P       uint8 `rostype:"char"`
			}{},
			"7fee3a6254fc0562bf1632f0fe8f05c8",
		},
		{
			"variable array",
			&struct {
				A uint8
				B []uint32
			}{},
			"fdee5bb88110a832e32fedabd50c71fc",
		},
		{
			"fixed array",
			&struct {
				A uint8
				B [2]uint32
			}{},
			"fd38051c1051a88ecf1ce1924053076c",
		},
		{
			"parent without pointer",
			&struct {
				A uint8
				B Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"parent with pointer",
			&struct {
				A uint8
				B *Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"array of parent without pointer",
			&struct {
				A uint8
				B []Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"array of parent with pointer",
			&struct {
				A uint8
				B []*Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"fixed array of parent",
			&struct {
				A uint8
				B [2]Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"definitions",
			&Log{},
			"acffd30cd6b6de30f120938c17c593fb",
		},
		{
			"empty struct",
			&struct {
			}{},
			"d41d8cd98f00b204e9800998ecf8427e",
		},
		{
			"custom name",
			&struct {
				A string `rosname:"A"`
			}{},
			"b9fd98954bcc9324b61cf24596e99bae",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			md5, err := MD5(ca.msg)
			require.NoError(t, err)
			require.Equal(t, ca.sum, md5)
		})
	}
}

type MsgExplicitPackage struct {
	Package `ros:"my_package"`
	Value   uint16
}

type MsgImplicitPackage struct {
	Value uint16
}

func TestType(t *testing.T) {
	for _, ca := range []struct {
		name string
		msg  interface{}
		typ  string
	}{
		{
			"explicit package",
			&MsgExplicitPackage{},
			"my_package/MsgExplicitPackage",
		},
		{
			"implicit package",
			&MsgImplicitPackage{},
			"goroslib/MsgImplicitPackage",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			typ, err := Type(ca.msg)
			require.NoError(t, err)
			require.Equal(t, ca.typ, typ)
		})
	}
}
