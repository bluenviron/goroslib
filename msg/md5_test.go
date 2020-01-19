package msg

import (
	"testing"

	"github.com/stretchr/testify/require"
)

type Parent struct {
	A String
}

var casesMessage = []struct {
	name string
	msg  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			A Bool
			B Byte
			C Char
			D Int8
			E Uint8
			F Int16
			G Uint16
			H Int32
			I Uint32
			J Int64
			K Uint64
			L Float32
			M Float64
			N String
			O Time
			P Duration
		}{},
		"384497568d692fe745850d4b0751295d",
	},
	{
		"variable array",
		&struct {
			A Uint8
			B []Uint32
		}{},
		"fdee5bb88110a832e32fedabd50c71fc",
	},
	{
		"fixed array",
		&struct {
			A Uint8
			B [2]Uint32
		}{},
		"fd38051c1051a88ecf1ce1924053076c",
	},
	{
		"parent without pointer",
		&struct {
			A Uint8
			B Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"parent with pointer",
		&struct {
			A Uint8
			B *Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"array of parent without pointer",
		&struct {
			A Uint8
			B []Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"array of parent with pointer",
		&struct {
			A Uint8
			B []*Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
	{
		"fixed array of parent",
		&struct {
			A Uint8
			B [2]Parent
		}{},
		"e8c99bd7177c56d5ef9104809bae67a1",
	},
}

func TestMessageMd5(t *testing.T) {
	for _, c := range casesMessage {
		t.Run(c.name, func(t *testing.T) {
			md5, err := MessageMd5(c.msg)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}

var casesService = []struct {
	name string
	req  interface{}
	res  interface{}
	sum  string
}{
	{
		"base types",
		&struct {
			A Float64
			B String
		}{},
		&struct {
			C Float64
		}{},
		"4fa8f09823d7ad898c6295d42385de20",
	},
}

func TestServiceMd5(t *testing.T) {
	for _, c := range casesService {
		t.Run(c.name, func(t *testing.T) {
			md5, err := ServiceMd5(c.req, c.res)
			require.NoError(t, err)
			require.Equal(t, c.sum, md5)
		})
	}
}
