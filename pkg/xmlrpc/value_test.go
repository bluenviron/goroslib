package xmlrpc

import (
	"bytes"
	"encoding/xml"
	"fmt"
	"io"
	"reflect"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

var casesValue = []struct {
	name string
	bdec []byte
	benc []byte
	v    interface{}
}{
	{
		"bool true",
		[]byte("<value><boolean>1</boolean></value>"),
		[]byte("<value><boolean>1</boolean></value>"),
		true,
	},
	{
		"bool false",
		[]byte("<value><boolean>0</boolean></value>"),
		[]byte("<value><boolean>0</boolean></value>"),
		false,
	},
	{
		"int",
		[]byte("<value><i4>-32</i4></value>"),
		[]byte("<value><i4>-32</i4></value>"),
		-32,
	},
	{
		"int with type int",
		[]byte("<value><int>-32</int></value>"),
		[]byte("<value><i4>-32</i4></value>"),
		-32,
	},
	{
		"double",
		[]byte("<value><double>-1.324543</double></value>"),
		[]byte("<value><double>-1.324543</double></value>"),
		-1.324543,
	},
	{
		"string",
		[]byte("<value><string>testval</string></value>"),
		[]byte("<value>testval</value>"),
		"testval",
	},
	{
		"string without type",
		[]byte("<value>testval</value>"),
		[]byte("<value>testval</value>"),
		"testval",
	},
	{
		"string empty",
		[]byte("<value><string></string></value>"),
		[]byte("<value></value>"),
		"",
	},
	{
		"string empty without type",
		[]byte("<value></value>"),
		[]byte("<value></value>"),
		"",
	},
	{
		"base64",
		[]byte("<value><base64>AQIDBA==</base64></value>"),
		[]byte("<value><base64>AQIDBA==</base64></value>"),
		[]byte("\x01\x02\x03\x04"),
	},
	{
		"array as struct",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		struct {
			Param1 string
			Param2 string
			Param3 int
			Param4 float64
			Param5 []byte
		}{
			"test1",
			"test2",
			123,
			-1.324543,
			[]byte{0x01, 0x02, 0x03, 0x04},
		},
	},
	{
		"array as slice, single type",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`</data></array></value>`),
		[]string{"test1", "test2"},
	},
	{
		"array as slice, multi type",
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		[]byte(`<value><array><data>` +
			`<value>test1</value>` +
			`<value>test2</value>` +
			`<value><i4>123</i4></value>` +
			`<value><double>-1.324543</double></value>` +
			`<value><base64>AQIDBA==</base64></value>` +
			`</data></array></value>`),
		[]interface{}{"test1", "test2", 123, -1.324543, []byte("\x01\x02\x03\x04")},
	},
}

func TestValueDecode(t *testing.T) {
	for _, ca := range casesValue {
		t.Run(ca.name, func(t *testing.T) {
			// decode to typed variable
			func() {
				dec := xml.NewDecoder(bytes.NewReader(ca.bdec))

				err := xmlGetStartElement(dec, "value")
				require.NoError(t, err)

				v := reflect.New(reflect.TypeOf(ca.v))
				err = valueDecode(dec, reflect.ValueOf(v.Interface()))
				require.NoError(t, err)

				require.Equal(t, ca.v, v.Elem().Interface())

				_, err = dec.Token()
				require.Equal(t, io.EOF, err)
			}()

			// decode to interface
			func() {
				if strings.HasPrefix(ca.name, "array") {
					return
				}

				dec := xml.NewDecoder(bytes.NewReader(ca.bdec))

				err := xmlGetStartElement(dec, "value")
				require.NoError(t, err)

				var v interface{}
				err = valueDecode(dec, reflect.ValueOf(&v))
				require.NoError(t, err)

				require.Equal(t, ca.v, v)

				_, err = dec.Token()
				require.Equal(t, io.EOF, err)
			}()
		})
	}
}

func TestValueDecodeErrors(t *testing.T) {
	for _, ca := range []struct {
		name string
		enc  []byte
		dest interface{}
		err  string
	}{
		{
			"not pointer",
			[]byte("<value>"),
			nil,
			"destination is not a pointer",
		},
		{
			"no data",
			[]byte("<value>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"invalid element",
			[]byte("<value><!-- comment -->"),
			new(interface{}),
			"unexpected element type: xml.Comment",
		},
		{
			"invalid type",
			[]byte("<value><invalid>"),
			new(interface{}),
			"unhandled value type: 'invalid'",
		},
		{
			"bool not closed",
			[]byte("<value><boolean>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"bool empty",
			[]byte("<value><boolean></boolean>"),
			new(interface{}),
			"value is not a bool: ''",
		},
		{
			"bool invalid",
			[]byte("<value><boolean>t</boolean>"),
			new(interface{}),
			"value is not a bool: 't'",
		},
		{
			"bool wrong type",
			[]byte("<value><boolean>1</boolean>"),
			func() interface{} {
				v := int(0)
				return &v
			}(),
			"cannot decode a bool into a int",
		},
		{
			"int not closed",
			[]byte("<value><int>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"int invalid",
			[]byte("<value><int>aaa</int>"),
			new(interface{}),
			"strconv.ParseInt: parsing \"aaa\": invalid syntax",
		},
		{
			"int wrong type",
			[]byte("<value><int>123</int>"),
			func() interface{} {
				v := "aaaa"
				return &v
			}(),
			"cannot decode a int into a string",
		},
		{
			"double not closed",
			[]byte("<value><double>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"double invalid",
			[]byte("<value><double>aaa</double>"),
			new(interface{}),
			"strconv.ParseFloat: parsing \"aaa\": invalid syntax",
		},
		{
			"double wrong type",
			[]byte("<value><double>123</double>"),
			func() interface{} {
				v := "aaaa"
				return &v
			}(),
			"cannot decode a double into a string",
		},
		{
			"string not closed",
			[]byte("<value><string>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"string untyped not closed",
			[]byte("<value>asd</otherval>"),
			new(interface{}),
			"XML syntax error on line 1: element <value> closed by </otherval>",
		},
		{
			"string wrong type 1",
			[]byte("<value><string>asd</string>"),
			func() interface{} {
				v := 123
				return &v
			}(),
			"cannot decode a string into a int",
		},
		{
			"string wrong type 2",
			[]byte("<value>asd</value>"),
			func() interface{} {
				v := 123
				return &v
			}(),
			"cannot decode a string into a int",
		},
		{
			"base64 not closed",
			[]byte("<value><base64>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"base64 invalid",
			[]byte("<value><base64>999</base64>"),
			new(interface{}),
			"illegal base64 data at input byte 0",
		},
		{
			"base64 wrong type",
			[]byte("<value><base64>aGVsbG8=</base64>"),
			func() interface{} {
				v := "aaaa"
				return &v
			}(),
			"cannot decode a base64 into a string",
		},
		{
			"array not closed 1",
			[]byte("<value><array>"),
			new(interface{}),
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"array not closed 2",
			[]byte("<value><array><data></data>"),
			&struct{}{},
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"array wrong type",
			[]byte("<value><array><data>"),
			func() interface{} {
				v := "aaaa"
				return &v
			}(),
			"cannot decode an array into a string",
		},
		{
			"array no values to struct",
			[]byte("<value><array><data>"),
			&struct {
				A string
			}{},
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"array no values to array",
			[]byte("<value><array><data>"),
			&[]string{},
			"XML syntax error on line 1: unexpected EOF",
		},
		{
			"array invalid value to struct",
			[]byte("<value><array><data><value><int>123</int>"),
			&struct {
				A string
			}{},
			"cannot decode a int into a string",
		},
		{
			"array invalid value to array",
			[]byte("<value><array><data><value><int>123</int>"),
			&[]string{},
			"cannot decode a int into a string",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			dec := xml.NewDecoder(bytes.NewReader(ca.enc))

			err := xmlGetStartElement(dec, "value")
			require.NoError(t, err)

			err = valueDecode(dec, reflect.ValueOf(ca.dest))
			require.Equal(t, ca.err, err.Error())
		})
	}
}

func TestValueEncode(t *testing.T) {
	for _, ca := range casesValue {
		t.Run(ca.name, func(t *testing.T) {
			var b bytes.Buffer
			err := valueEncode(&b, reflect.ValueOf(ca.v))
			require.NoError(t, err)
			require.Equal(t, ca.benc, b.Bytes())
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

func TestValueEncodeError(t *testing.T) {
	for _, ca := range []struct {
		name string
		v    interface{}
		dest io.Writer
		err  string
	}{
		{
			"open tag write error",
			nil,
			&limitedBuffer{cap: 0},
			"capacity reached",
		},
		{
			"close tag write error",
			true,
			&limitedBuffer{cap: 32},
			"capacity reached",
		},
		{
			"bool write error",
			true,
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"int write error",
			123,
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"double write error",
			float64(123),
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"string write error",
			"testing",
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"base64 write error",
			[]byte("testing"),
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"struct open tag write error",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"struct field write error",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 20},
			"capacity reached",
		},
		{
			"struct close tag write error",
			struct {
				A string
			}{"testing"},
			&limitedBuffer{cap: 50},
			"capacity reached",
		},
		{
			"slice open tag write error",
			[]string{"testing"},
			&limitedBuffer{cap: 10},
			"capacity reached",
		},
		{
			"slice element write error",
			[]string{"testing"},
			&limitedBuffer{cap: 20},
			"capacity reached",
		},
		{
			"slice close tag write error",
			[]string{"testing"},
			&limitedBuffer{cap: 50},
			"capacity reached",
		},
		{
			"unhandled value type",
			int64(123),
			bytes.NewBuffer(nil),
			"unhandled value type: int64",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			err := valueEncode(ca.dest, reflect.ValueOf(ca.v))
			require.Equal(t, ca.err, err.Error())
		})
	}
}
