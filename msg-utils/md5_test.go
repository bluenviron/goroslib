package msg_utils

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs"
)

type Parent struct {
	A string
}

var casesMd5Message = []struct {
	name string
	msg  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			msgs.Package `ros:"testing"`
			A            bool
			B            int8
			C            uint8
			D            int16
			E            uint16
			F            int32
			G            uint32
			H            int64
			I            uint64
			J            float32
			K            float64
			L            string
			M            time.Time
			N            time.Duration
		}{},
		"1fe19d9a049a663e9b521bf584177015",
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
}

func TestMd5Message(t *testing.T) {
	for _, c := range casesMd5Message {
		t.Run(c.name, func(t *testing.T) {
			md5, err := Md5Message(c.msg)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}

var casesMd5Service = []struct {
	name string
	req  interface{}
	res  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			A float64
			B string
		}{},
		&struct {
			C float64
		}{},
		"4fa8f09823d7ad898c6295d42385de20",
	},
}

func TestMd5Service(t *testing.T) {
	for _, c := range casesMd5Service {
		t.Run(c.name, func(t *testing.T) {
			md5, err := Md5Service(c.req, c.res)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}
