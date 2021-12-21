package msgproc

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msg"
)

type Parent struct {
	A string
}

type Header struct {
	msg.Package `ros:"std_msgs"`
	Seq         uint32
	Stamp       time.Time
	FrameId     string //nolint:revive
}

type Log struct {
	msg.Package     `ros:"rosgraph_msgs"`
	msg.Definitions `ros:"byte DEBUG=1,byte INFO=2,byte WARN=4,byte ERROR=8,byte FATAL=16"`
	Header          Header
	Level           int8 `rostype:"byte"`
	Name            string
	Msg             string
	File            string
	Function        string
	Line            uint32
	Topics          []string
}

func TestMD5(t *testing.T) {
	for _, ca := range []struct {
		name string
		msg  interface{}
		sum  string
	}{
		{
			"base types",
			struct {
				msg.Package `ros:"testing"`
				A           bool
				B           int8
				C           uint8
				D           int16
				E           uint16
				F           int32
				G           uint32
				H           int64
				I           uint64
				J           float32
				K           float64
				L           string
				M           time.Time
				N           time.Duration
				O           int8  `rostype:"byte"`
				P           uint8 `rostype:"char"`
				Q           []uint32
				R           [2]uint32
			}{},
			"5cd0716936244988af75265581e7d892",
		},
		{
			"parent",
			struct {
				A uint8
				B Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"array of parent",
			struct {
				A uint8
				B []Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"fixed array of parent",
			struct {
				A uint8
				B [2]Parent
			}{},
			"e8c99bd7177c56d5ef9104809bae67a1",
		},
		{
			"custom name",
			struct {
				A string `rosname:"A"`
			}{},
			"b9fd98954bcc9324b61cf24596e99bae",
		},
		{
			"definitions",
			Log{},
			"acffd30cd6b6de30f120938c17c593fb",
		},
		{
			"empty struct",
			struct{}{},
			"d41d8cd98f00b204e9800998ecf8427e",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			md5, err := MD5(ca.msg)
			require.NoError(t, err)
			require.Equal(t, ca.sum, md5)
		})
	}
}

func TestMD5Errors(t *testing.T) {
	for _, ca := range []struct {
		name string
		msg  interface{}
		err  string
	}{
		{
			"not a message",
			123,
			"message must be a struct",
		},
		{
			"unsupported field type 1",
			struct {
				A interface{}
			}{nil},
			"unsupported field type 'interface {}'",
		},
		{
			"unsupported field type 2",
			struct {
				A []interface{}
			}{nil},
			"unsupported field type 'interface {}'",
		},
		{
			"unsupported field type 3",
			struct {
				A [2]interface{}
			}{[2]interface{}{1, 2}},
			"unsupported field type 'interface {}'",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			_, err := MD5(ca.msg)
			require.EqualError(t, err, ca.err)
		})
	}
}
