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
			i -= 1
		}
	}
	return string(tmp)
}

type Header interface {
	IsHeader()
}

// HeaderDecode decodes an header in binary format.
func HeaderDecode(raw HeaderRaw, header Header) error {
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

// HeaderEncode encodes an header in binary format.
func HeaderEncode(w io.Writer, header Header) error {
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

		if val.Kind() == reflect.Ptr {
			if val.IsNil() {
				continue
			}
			val = val.Elem()
		}

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
