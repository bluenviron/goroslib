package protocommon

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"reflect"
	"strconv"
	"unicode"
)

func camelToSnake(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToLower(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if unicode.IsUpper(tmp[i]) {
			tmp[i] = unicode.ToLower(tmp[i])
			tmp = append(tmp[:i], append([]rune{'_'}, tmp[i:]...)...)
		}
	}
	return string(tmp)
}

func snakeToCamel(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToUpper(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if tmp[i] == '_' {
			tmp[i+1] = unicode.ToUpper(tmp[i+1])
			tmp = append(tmp[:i], tmp[i+1:]...)
			i--
		}
	}
	return string(tmp)
}

// HeaderRaw is a raw header.
type HeaderRaw map[string]string

// HeaderRawDecode decodes a raw header in binary format.
func HeaderRawDecode(r io.Reader) (HeaderRaw, error) {
	raw := make(HeaderRaw)

	// use a shared buffer for performance reasons
	buf := make([]byte, 8)

	// read header length
	_, err := r.Read(buf[:4])
	if err != nil {
		return nil, err
	}
	hlen := int64(binary.LittleEndian.Uint32(buf))
	if hlen == 0 {
		return nil, fmt.Errorf("invalid header length")
	}

	for hlen > 0 {
		// field length
		err := readLimited(r, buf[:4], &hlen)
		if err != nil {
			return nil, err
		}
		flen := int64(binary.LittleEndian.Uint32(buf))
		if flen == 0 || flen > hlen {
			return nil, fmt.Errorf("invalid field length")
		}

		// field
		field := make([]byte, flen)
		err = readLimitedDoNotCheck(r, field, &hlen)
		if err != nil {
			return nil, err
		}

		i := bytes.IndexByte(field, '=')
		if i < 0 {
			return nil, fmt.Errorf("missing separator")
		}

		key := string(field[:i])
		val := string(field[i+1:])

		raw[key] = val
	}

	return raw, nil
}

// Header is an header.
type Header interface{}

// HeaderDecode decodes an header in binary format.
func HeaderDecode(raw HeaderRaw, dest Header) error {
	rv := reflect.ValueOf(dest)
	if rv.Kind() != reflect.Ptr || rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("dest must be a pointer to a struct")
	}

	for key, val := range raw {
		key = snakeToCamel(key)

		rf := rv.Elem().FieldByName(key)
		zero := reflect.Value{}
		if rf == zero {
			// skip unhandled fields
			continue
		}

		switch rf.Kind() {
		case reflect.String:
			rf.SetString(val)

		case reflect.Int:
			i, err := strconv.ParseInt(val, 10, 64)
			if err != nil {
				return err
			}
			rf.SetInt(i)

		default:
			return fmt.Errorf("unsupported field type: %s", rf.Type())
		}
	}

	return nil
}

// HeaderEncode encodes an header in binary format.
func HeaderEncode(w io.Writer, src Header) error {
	rv := reflect.ValueOf(src)
	if rv.Kind() != reflect.Ptr || rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("src must be a pointer to a struct")
	}

	// use a shared buffer to improve performance
	buf := make([]byte, 8)

	// compute header
	var he bytes.Buffer
	nf := rv.Elem().NumField()
	for i := 0; i < nf; i++ {
		key := camelToSnake(rv.Elem().Type().Field(i).Name)
		val := rv.Elem().Field(i)

		flen := uint32(0)

		bkey := []byte(key)
		flen += uint32(len(bkey))

		flen++

		bval := func() []byte {
			if val.Kind() == reflect.String {
				return []byte(val.Interface().(string))
			}

			// reflect.Int
			return []byte(strconv.FormatInt(int64(val.Interface().(int)), 10))
		}()
		flen += uint32(len(bval))

		// write field length
		binary.LittleEndian.PutUint32(buf, flen)
		he.Write(buf[:4])

		he.Write(bkey)
		he.Write([]byte{'='})
		he.Write(bval)
	}

	// write header length
	binary.LittleEndian.PutUint32(buf, uint32(he.Len()))
	_, err := w.Write(buf[:4])
	if err != nil {
		return err
	}

	// write header
	_, err = w.Write(he.Bytes())
	return err
}
