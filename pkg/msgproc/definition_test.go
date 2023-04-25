package msgproc

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type Parent2 struct {
	msg.Name `ros:"parent_2"`
	X        int32
}

type Parent1 struct {
	Z string
	Y []Parent2
}

func TestDefinition(t *testing.T) {
	for _, ca := range []struct {
		name string
		msg  interface{}
		def  string
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
				R           [4]uint32
			}{},
			"bool a\n" +
				"int8 b\n" +
				"uint8 c\n" +
				"int16 d\n" +
				"uint16 e\n" +
				"int32 f\n" +
				"uint32 g\n" +
				"int64 h\n" +
				"uint64 i\n" +
				"float32 j\n" +
				"float64 k\n" +
				"string l\n" +
				"time m\n" +
				"duration n\n" +
				"byte o\n" +
				"char p\n" +
				"uint32[] q\n" +
				"uint32[4] r\n" +
				"\n",
		},
		{
			"custom name",
			struct {
				A string `rosname:"A"`
			}{},
			"string A\n" +
				"\n",
		},
		{
			"definitions",
			struct {
				msg.Package     `ros:"testing"`
				msg.Definitions `ros:"byte DEBUG=1,byte INFO=2,byte WARN=4,byte ERROR=8,byte FATAL=16"`
				A               uint8
			}{},
			"byte DEBUG=1\n" +
				"byte INFO=2\n" +
				"byte WARN=4\n" +
				"byte ERROR=8\n" +
				"byte FATAL=16\n" +
				"uint8 a\n" +
				"\n",
		},
		{
			"parents",
			struct {
				msg.Package `ros:"testing"`
				A           uint8
				B           Parent1
				C           []Parent1
			}{},
			"uint8 a\n" +
				"goroslib/Parent1 b\n" +
				"goroslib/Parent1[] c\n" +
				"\n" +
				"================================================================================\n" +
				"MSG: goroslib/Parent1\n" +
				"string z\n" +
				"goroslib/parent_2[] y\n" +
				"\n" +
				"================================================================================\n" +
				"MSG: goroslib/parent_2\n" +
				"int32 x\n" +
				"\n",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			def, err := Definition(ca.msg)
			require.NoError(t, err)
			require.Equal(t, ca.def, def)
		})
	}
}

func TestDefinitionErrors(t *testing.T) {
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
			_, err := Definition(ca.msg)
			require.EqualError(t, err, ca.err)
		})
	}
}
