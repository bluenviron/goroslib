package protocommon

import (
	"bytes"
	"fmt"
	"io"
	"reflect"
	"testing"

	"github.com/stretchr/testify/require"
)

type headerSubscriber struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
	TcpNodelay        int //nolint:revive
}

type headerPublisher struct {
	Topic    string
	Type     string
	Md5sum   string
	Callerid string
	Latching int
}

type headerError struct {
	Error string
}

var casesHeader = []struct {
	name   string
	header Header
	byts   []byte
}{
	{
		"subscriber",
		&headerSubscriber{
			Callerid:          "/rostopic_1_1585920221893",
			Md5sum:            "6a62c6daae103f4ff57a132d6f95cec2",
			Topic:             "/test_pub",
			Type:              "sensor_msgs/Imu",
			MessageDefinition: "test definition",
		},
		[]byte("\xb3\x00\x00\x00\x22\x00\x00\x00\x63\x61\x6c\x6c" +
			"\x65\x72\x69\x64\x3d\x2f\x72\x6f\x73\x74\x6f" +
			"\x70\x69\x63\x5f\x31\x5f\x31\x35\x38\x35\x39" +
			"\x32\x30\x32\x32\x31\x38\x39\x33\x0f\x00\x00\x00" +
			"\x74\x6f\x70\x69\x63\x3d\x2f\x74\x65\x73\x74" +
			"\x5f\x70\x75\x62\x14\x00\x00\x00\x74\x79\x70\x65" +
			"\x3d\x73\x65\x6e\x73\x6f\x72\x5f\x6d\x73\x67" +
			"\x73\x2f\x49\x6d\x75\x27\x00\x00\x00\x6d\x64\x35" +
			"\x73\x75\x6d\x3d\x36\x61\x36\x32\x63\x36\x64\x61" +
			"\x61\x65\x31\x30\x33\x66\x34\x66\x66\x35\x37\x61" +
			"\x31\x33\x32\x64\x36\x66\x39\x35\x63\x65\x63\x32" +
			"\x22\x00\x00\x00\x6d\x65\x73\x73\x61\x67\x65\x5f" +
			"\x64\x65\x66\x69\x6e\x69\x74\x69\x6f\x6e\x3d\x74" +
			"\x65\x73\x74\x20\x64\x65\x66\x69\x6e\x69\x74\x69" +
			"\x6f\x6e\x0d\x00\x00\x00\x74\x63\x70\x5f\x6e\x6f\x64" +
			"\x65\x6c\x61\x79\x3d\x30"),
	},
	{
		"publisher",
		&headerPublisher{
			Callerid: "/testing_1_2_3",
			Md5sum:   "6a62c6daae1as3f4ff57a132d6f95cec2",
			Topic:    "/test_topic",
			Type:     "testlib/Msg",
			Latching: 0,
		},
		[]byte("\x7e\x00\x00\x00\x11\x00\x00\x00\x74\x6f\x70\x69\x63\x3d\x2f\x74\x65\x73" +
			"\x74\x5f\x74\x6f\x70\x69\x63\x10\x00\x00\x00\x74\x79\x70\x65\x3d\x74\x65" +
			"\x73\x74\x6c\x69\x62\x2f\x4d\x73\x67\x28\x00\x00\x00\x6d\x64\x35\x73\x75" +
			"\x6d\x3d\x36\x61\x36\x32\x63\x36\x64\x61\x61\x65\x31\x61\x73\x33\x66\x34" +
			"\x66\x66\x35\x37\x61\x31\x33\x32\x64\x36\x66\x39\x35\x63\x65\x63\x32\x17" +
			"\x00\x00\x00\x63\x61\x6c\x6c\x65\x72\x69\x64\x3d\x2f\x74\x65\x73\x74\x69" +
			"\x6e\x67\x5f\x31\x5f\x32\x5f\x33\x0a\x00\x00\x00\x6c\x61\x74\x63\x68\x69" +
			"\x6e\x67\x3d\x30"),
	},
	{
		"publisher with error",
		&headerError{
			Error: ("test error"),
		},
		[]byte("\x14\x00\x00\x00\x10\x00\x00\x00\x65\x72\x72\x6f\x72\x3d\x74\x65\x73" +
			"\x74\x20\x65\x72\x72\x6f\x72"),
	},
}

func TestHeaderDecode(t *testing.T) {
	for _, ca := range casesHeader {
		t.Run(ca.name, func(t *testing.T) {
			raw, err := HeaderRawDecode(bytes.NewBuffer(ca.byts))
			require.NoError(t, err)
			header := reflect.New(reflect.TypeOf(ca.header).Elem()).Interface().(Header)
			err = HeaderDecode(raw, header)
			require.NoError(t, err)
			require.Equal(t, ca.header, header)
		})
	}
}

func TestHeaderDecodeUnhandledField(t *testing.T) {
	raw, err := HeaderRawDecode(bytes.NewBuffer([]byte{
		0x19, 0x00, 0x00, 0x00,
		0x0A, 0x00, 0x00, 0x00,
		'e', 'r', 'r', 'o', 'r', '=', 't', 'e', 's', 't',
		0x07, 0x00, 0x00, 0x00,
		'o', 't', 'h', 'e', 'r', '=', 'k',
	}))
	require.NoError(t, err)

	header := reflect.New(reflect.TypeOf(&headerError{
		Error: ("test"),
	}).Elem()).Interface().(Header)

	err = HeaderDecode(raw, header)
	require.NoError(t, err)
	require.Equal(t, &headerError{
		Error: ("test"),
	}, header)
}

func FuzzHeaderDecode(f *testing.F) {
	f.Add([]byte{
		0x53, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
		0x63, 0x61, 0x6c, 0x6c, 0x65, 0x72, 0x69, 0x64,
		0x3d, 0x06, 0x00, 0x00, 0x00, 0x74, 0x6f, 0x70,
		0x69, 0x63, 0x3d, 0x05, 0x00, 0x00, 0x00, 0x74,
		0x79, 0x70, 0x65, 0x3d, 0x07, 0x00, 0x00, 0x00,
		0x6d, 0x64, 0x35, 0x73, 0x75, 0x6d, 0x3d, 0x13,
		0x00, 0x00, 0x00, 0x6d, 0x65, 0x73, 0x73, 0x61,
		0x67, 0x65, 0x5f, 0x64, 0x65, 0x66, 0x69, 0x6e,
		0x69, 0x74, 0x69, 0x6f, 0x6e, 0x3d, 0x0d, 0x00,
		0x00, 0x00, 0x74, 0x63, 0x70, 0x5f, 0x6e, 0x6f,
		0x64, 0x65, 0x6c, 0x61, 0x79, 0x3d, 0x73,
	})

	f.Fuzz(func(t *testing.T, b []byte) {
		raw, err := HeaderRawDecode(bytes.NewBuffer(b))
		if err == nil {
			var h headerSubscriber
			HeaderDecode(raw, &h) //nolint:errcheck
		}
	})
}

func TestHeaderEncode(t *testing.T) {
	for _, ca := range casesHeader {
		t.Run(ca.name, func(t *testing.T) {
			var buf bytes.Buffer
			err := HeaderEncode(&buf, ca.header)
			require.NoError(t, err)
			require.Equal(t, ca.byts, buf.Bytes())
		})
	}
}

type limitedBuffer struct {
	cap int
	n   int
}

func (b *limitedBuffer) Write(p []byte) (int, error) {
	b.n += len(p)
	if b.n > b.cap {
		return 0, fmt.Errorf("capacity reached")
	}
	return len(p), nil
}

func TestHeaderEncodeErrors(t *testing.T) {
	for _, ca := range []struct {
		name   string
		header Header
		dest   io.Writer
		err    string
	}{
		{
			"src not pointer to struct",
			nil,
			nil,
			"src must be a pointer to a struct",
		},
		{
			"write error",
			&headerPublisher{
				Callerid: "/testing_1_2_3",
				Md5sum:   "6a62c6daae1as3f4ff57a132d6f95cec2",
				Topic:    "/test_topic",
				Type:     "testlib/Msg",
				Latching: 0,
			},
			&limitedBuffer{cap: 0},
			"capacity reached",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			err := HeaderEncode(ca.dest, ca.header)
			require.EqualError(t, err, ca.err)
		})
	}
}
