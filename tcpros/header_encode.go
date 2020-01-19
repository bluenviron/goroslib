package tcpros

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"reflect"
	"strconv"
)

func headerEncode(w io.Writer, header interface{}) error {
	// check input
	rv := reflect.ValueOf(header)
	if rv.Kind() != reflect.Ptr {
		return fmt.Errorf("invalid message kind: expected ptr, got %s", rv.Kind())
	}
	if rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("invalid message kind: expected struct, got %s", rv.Kind())
	}

	// use a shared buffer for performance reasons
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

		flen += 1

		bval := func() []byte {
			switch val.Kind() {
			case reflect.String:
				return []byte(val.Interface().(string))

			case reflect.Int:
				return []byte(strconv.FormatInt(int64(val.Interface().(int)), 10))
			}
			return nil
		}()
		flen += uint32(len(bval))

		// write field length
		binary.LittleEndian.PutUint32(buf, flen)
		_, err := he.Write(buf[:4])
		if err != nil {
			return err
		}

		_, err = he.Write(bkey)
		if err != nil {
			return err
		}

		_, err = he.Write([]byte{'='})
		if err != nil {
			return err
		}

		_, err = he.Write(bval)
		if err != nil {
			return err
		}
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
