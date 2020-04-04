package msg_utils

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs"
)

type Parent struct {
	A msgs.String
}

var casesMessageMd5 = []struct {
	name string
	msg  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			msgs.Package `ros:"testing"`
			A            msgs.Bool
			B            msgs.Byte
			C            msgs.Char
			D            msgs.Int8
			E            msgs.Uint8
			F            msgs.Int16
			G            msgs.Uint16
			H            msgs.Int32
			I            msgs.Uint32
			J            msgs.Int64
			K            msgs.Uint64
			L            msgs.Float32
			M            msgs.Float64
			N            msgs.String
			O            msgs.Time
			P            msgs.Duration
		}{},
		"384497568d692fe745850d4b0751295d",
	},
	{
		"variable array",
		&struct {
			A msgs.Uint8
			B []msgs.Uint32
		}{},
		"fdee5bb88110a832e32fedabd50c71fc",
	},
	{
		"fixed array",
		&struct {
			A msgs.Uint8
			B [2]msgs.Uint32
		}{},
		"fd38051c1051a88ecf1ce1924053076c",
	},
	{
		"parent without pointer",
		&struct {
			A msgs.Uint8
			B Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"parent with pointer",
		&struct {
			A msgs.Uint8
			B *Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"array of parent without pointer",
		&struct {
			A msgs.Uint8
			B []Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"array of parent with pointer",
		&struct {
			A msgs.Uint8
			B []*Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"fixed array of parent",
		&struct {
			A msgs.Uint8
			B [2]Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
}

func TestMessageMd5(t *testing.T) {
	for _, c := range casesMessageMd5 {
		t.Run(c.name, func(t *testing.T) {
			md5, err := MessageMd5(c.msg)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}

var casesServiceMd5 = []struct {
	name string
	req  interface{}
	res  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			A msgs.Float64
			B msgs.String
		}{},
		&struct {
			C msgs.Float64
		}{},
		"4fa8f09823d7ad898c6295d42385de20",
	},
}

func TestServiceMd5(t *testing.T) {
	for _, c := range casesServiceMd5 {
		t.Run(c.name, func(t *testing.T) {
			md5, err := ServiceMd5(c.req, c.res)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}
