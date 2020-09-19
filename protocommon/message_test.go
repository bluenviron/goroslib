package proto_common

import (
	"bytes"
	"reflect"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs"
)

type Parent struct {
	A string
}

type caseMessage struct {
	name string
	msg  interface{}
	byts []byte
}

var casesMessage = []caseMessage{
	{
		"empty",
		&struct{}{},
		[]byte{0x00, 0x00, 0x00, 0x00},
	},
	{
		"base types",
		&struct {
			A bool
			D int8
			E uint8
			F int16
			G uint16
			H int32
			I uint32
			J int64
			K uint64
			L float32
			M float64
			N string
			O time.Time
			P time.Duration
		}{
			true, -1, 2, -3, 4, -5, 6, -7, 8, 9, 10, "abc",
			time.Date(2010, 11, 12, 13, 14, 15, 16, time.UTC),
			time.Duration(5 * time.Second),
		},
		[]byte{
			0x42, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x02, 0xFD,
			0xFF, 0x04, 0x00, 0xFB, 0xFF, 0xFF, 0xFF, 0x06,
			0x00, 0x00, 0x00, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0x08, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x41, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x40, 0x03,
			0x00, 0x00, 0x00, 0x61, 0x62, 0x63, 0xa7, 0x3d,
			0xdd, 0x4c, 0x10, 0x00, 0x00, 0x00, 0x05, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		},
	},
	{
		"empty string",
		&struct {
			A string
		}{
			"",
		},
		[]byte{
			0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		},
	},
	{
		"empty time",
		&struct {
			A time.Time
		}{
			time.Time{},
		},
		[]byte{
			0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
		},
	},
	{
		"empty duration",
		&struct {
			A time.Duration
		}{
			time.Duration(0),
		},
		[]byte{
			0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
		},
	},
	{
		"variable array",
		&struct {
			A uint8
			B []uint32
		}{
			1, []uint32{2, 3},
		},
		[]byte{
			0x0d, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
			0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
			0x00,
		},
	},
	{
		"fixed array",
		&struct {
			A uint8
			B [2]uint32
		}{
			1, [2]uint32{2, 3},
		},
		[]byte{
			0x09, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
			0x00, 0x03, 0x00, 0x00, 0x00,
		},
	},
	{
		"variable array of pointers",
		&struct {
			A []*uint32
		}{
			[]*uint32{&[]uint32{2}[0], &[]uint32{3}[0]},
		},
		[]byte{
			0x0c, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
		},
	},
	{
		"fixed array of pointers",
		&struct {
			A [2]*uint32
		}{
			[2]*uint32{&[]uint32{2}[0], &[]uint32{3}[0]},
		},
		[]byte{
			0x08, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
			0x03, 0x00, 0x00, 0x00,
		},
	},
	{
		"variable array of parent",
		&struct {
			A uint8
			B []Parent
		}{
			1, []Parent{{"abc"}, {"def"}},
		},
		[]byte{
			0x13, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
			0x00, 0x03, 0x00, 0x00, 0x00, 0x61, 0x62, 0x63,
			0x03, 0x00, 0x00, 0x00, 0x64, 0x65, 0x66,
		},
	},
	{
		"fixed array of parent",
		&struct {
			A uint8
			B [2]Parent
		}{
			1, [2]Parent{{"abc"}, {"def"}},
		},
		[]byte{
			0x0f, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00,
			0x00, 0x61, 0x62, 0x63, 0x03, 0x00, 0x00, 0x00,
			0x64, 0x65, 0x66,
		},
	},
	{
		"with custom package",
		&struct {
			msgs.Package `ros:"testing"`
			A            int32
		}{0, 123},
		[]byte{
			0x04, 0x00, 0x00, 0x00, 0x7b, 0x00, 0x00, 0x00,
		},
	},
	{
		"with definition",
		&struct {
			msgs.Definitions `ros:"uint8 A=0,uint8 B=1"`
			A                int32
		}{0, 123},
		[]byte{
			0x04, 0x00, 0x00, 0x00, 0x7b, 0x00, 0x00, 0x00,
		},
	},
}

func TestMessageDecode(t *testing.T) {
	for _, c := range casesMessage {
		t.Run(c.name, func(t *testing.T) {
			msg := reflect.New(reflect.TypeOf(c.msg).Elem()).Interface()
			err := MessageDecode(bytes.NewBuffer(c.byts), msg)
			require.NoError(t, err)
			require.Equal(t, c.msg, msg)
		})
	}
}

func TestMessageEncode(t *testing.T) {
	for _, c := range casesMessage {
		t.Run(c.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := MessageEncode(&buf, c.msg)
			require.NoError(t, err)
			require.Equal(t, c.byts, buf.Bytes())
		})
	}
}

// generate some big messages dynamically
func benchmarkCases(b *testing.B) []caseMessage {
	const size = 2 << 20

	var arrayUint8 [size]uint8
	for i := range arrayUint8 {
		arrayUint8[i] = uint8(i)
	}

	cases := []caseMessage{
		{
			"big variable array",
			&struct {
				A []uint8
			}{arrayUint8[:]},
			nil,
		},
		{
			"big fixed array",
			&struct {
				A [size]uint8
			}{arrayUint8},
			nil,
		},
	}

	for i := range cases {
		var buf bytes.Buffer
		if err := MessageEncode(&buf, cases[i].msg); err != nil {
			b.Fatal(err)
		}
		cases[i].byts = buf.Bytes()
	}

	return cases
}

func BenchmarkMessageDecode(b *testing.B) {
	cases := append(benchmarkCases(b), casesMessage...)
	for _, c := range cases {
		b.Run(c.name, func(b *testing.B) {
			// reuse message in loop to test benefit from preallocated fields
			msg := reflect.New(reflect.TypeOf(c.msg).Elem()).Interface()
			for i := 0; i < b.N; i++ {
				MessageDecode(bytes.NewBuffer(c.byts), msg)
			}
		})
	}
}

func BenchmarkMessageEncode(b *testing.B) {
	cases := append(benchmarkCases(b), casesMessage...)
	for _, c := range cases {
		b.Run(c.name, func(b *testing.B) {
			var buf bytes.Buffer
			for i := 0; i < b.N; i++ {
				MessageEncode(&buf, c.msg)
				buf.Reset()
			}
		})
	}
}
