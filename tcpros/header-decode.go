package tcpros

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"reflect"
	"strconv"
)

type HeaderRaw map[string]string

func (hr HeaderRaw) Decode(header interface{}) error {
	return headerDecode(hr, header)
}

func headerDecodeRaw(r io.Reader) (HeaderRaw, error) {
	raw := make(HeaderRaw)

	// use a shared buffer for performance reasons
	buf := make([]byte, 8)

	// read header length
	_, err := r.Read(buf[:4])
	if err != nil {
		return nil, err
	}
	hlen := binary.LittleEndian.Uint32(buf)
	if hlen == 0 {
		return nil, fmt.Errorf("invalid header length")
	}

	for hlen > 0 {
		// read field length
		_, err := r.Read(buf[:4])
		if err != nil {
			return nil, err
		}
		hlen -= 4
		flen := binary.LittleEndian.Uint32(buf)
		if flen == 0 || flen > hlen {
			return nil, fmt.Errorf("invalid field length")
		}

		// read field
		field := make([]byte, int(flen))
		_, err = io.ReadFull(r, field)
		if err != nil {
			return nil, err
		}
		hlen -= flen

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

func headerDecode(raw HeaderRaw, header interface{}) error {
	// check input
	rv := reflect.ValueOf(header)
	if rv.Kind() != reflect.Ptr {
		return fmt.Errorf("invalid message kind: expected ptr, got %s", rv.Kind())
	}
	if rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("invalid message kind: expected struct, got %s", rv.Kind())
	}

	for key, val := range raw {
		key = snakeToCamel(key)

		rf := rv.Elem().FieldByName(key)
		zero := reflect.Value{}
		if rf == zero {
			// skip unhandled fields
			continue
		}

		if rf.Kind() == reflect.Ptr {
			ptr := reflect.New(rf.Type().Elem())
			rf.Set(ptr)
			rf = ptr.Elem()
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
			return fmt.Errorf("invalid field kind: %s", rf.Kind())
		}
	}

	return nil
}
