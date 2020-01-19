package tcpros

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"reflect"
	"time"

	"github.com/aler9/goroslib/msg"
)

func messageEncode(w io.Writer, msg interface{}) error {
	// check target
	rv := reflect.ValueOf(msg)
	if rv.Kind() != reflect.Ptr {
		return fmt.Errorf("invalid message kind: expected ptr, got %s", rv.Kind())
	}
	if rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("invalid message kind: expected struct, got %s", rv.Kind())
	}

	// use a shared buffer for performance reasons
	buf := make([]byte, 8)

	// encode message
	var vw bytes.Buffer
	err := messageEncodeValue(&vw, rv, buf)
	if err != nil {
		return err
	}

	// write message length
	binary.LittleEndian.PutUint32(buf, uint32(vw.Len()))
	_, err = w.Write(buf[:4])
	if err != nil {
		return err
	}

	// write message
	_, err = w.Write(vw.Bytes())
	return err
}

func messageEncodeValue(w io.Writer, val reflect.Value, buf []byte) error {
	switch cv := val.Elem().Interface().(type) {
	case msg.Bool:
		b := uint8(0x00)
		if cv {
			b = 0x01
		}
		_, err := w.Write([]byte{b})
		return err

	case msg.Byte:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msg.Char:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msg.Int8:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msg.Uint8:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msg.Int16:
		binary.LittleEndian.PutUint16(buf, uint16(cv))
		_, err := w.Write(buf[:2])
		return err

	case msg.Uint16:
		binary.LittleEndian.PutUint16(buf, uint16(cv))
		_, err := w.Write(buf[:2])
		return err

	case msg.Int32:
		binary.LittleEndian.PutUint32(buf, uint32(cv))
		_, err := w.Write(buf[:4])
		return err

	case msg.Uint32:
		binary.LittleEndian.PutUint32(buf, uint32(cv))
		_, err := w.Write(buf[:4])
		return err

	case msg.Int64:
		binary.LittleEndian.PutUint64(buf, uint64(cv))
		_, err := w.Write(buf[:8])
		return err

	case msg.Uint64:
		binary.LittleEndian.PutUint64(buf, uint64(cv))
		_, err := w.Write(buf[:8])
		return err

	case msg.Float32:
		binary.LittleEndian.PutUint32(buf, math.Float32bits(float32(cv)))
		_, err := w.Write(buf[:4])
		return err

	case msg.Float64:
		binary.LittleEndian.PutUint64(buf, math.Float64bits(float64(cv)))
		_, err := w.Write(buf[:8])
		return err

	case msg.String:
		bstr := []byte(cv)

		// string length
		binary.LittleEndian.PutUint32(buf, uint32(len(bstr)))
		_, err := w.Write(buf[:4])
		if err != nil {
			return err
		}

		// string
		_, err = w.Write(bstr)
		return err

	case msg.Time:
		// special case: zero means year zero, not 1970
		// so time.Time{} can be encoded / decoded
		var nano int64
		var zero time.Time
		if cv == zero {
			nano = 0
		} else {
			nano = cv.UnixNano()
		}

		binary.LittleEndian.PutUint32(buf, uint32(nano/1000000000))
		_, err := w.Write(buf[:4])
		if err != nil {
			return err
		}

		binary.LittleEndian.PutUint32(buf, uint32(nano%1000000000))
		_, err = w.Write(buf[:4])
		return err

	case msg.Duration:
		nano := cv.Nanoseconds()

		binary.LittleEndian.PutUint32(buf, uint32(nano/1000000000))
		_, err := w.Write(buf[:4])
		if err != nil {
			return err
		}

		binary.LittleEndian.PutUint32(buf, uint32(nano%1000000000))
		_, err = w.Write(buf[:4])
		return err
	}

	switch val.Elem().Kind() {
	case reflect.Slice:
		le := val.Elem().Len()

		// slice length
		binary.LittleEndian.PutUint32(buf, uint32(le))
		_, err := w.Write(buf[:4])
		if err != nil {
			return err
		}

		// slice elements
		for i := 0; i < le; i++ {
			el := val.Elem().Index(i)

			if el.Kind() == reflect.Ptr {
				err := messageEncodeValue(w, el, buf)
				if err != nil {
					return err
				}
			} else {
				err := messageEncodeValue(w, el.Addr(), buf)
				if err != nil {
					return err
				}
			}
		}
		return nil

	case reflect.Array:
		le := val.Elem().Len()

		// array elements
		for i := 0; i < le; i++ {
			el := val.Elem().Index(i)

			if el.Kind() == reflect.Ptr {
				err := messageEncodeValue(w, el, buf)
				if err != nil {
					return err
				}
			} else {
				err := messageEncodeValue(w, el.Addr(), buf)
				if err != nil {
					return err
				}
			}
		}
		return nil

	case reflect.Struct:
		// struct fields
		nf := val.Elem().NumField()
		for i := 0; i < nf; i++ {
			el := val.Elem().Field(i)

			if el.Kind() == reflect.Ptr {
				err := messageEncodeValue(w, el, buf)
				if err != nil {
					return err
				}
			} else {
				err := messageEncodeValue(w, el.Addr(), buf)
				if err != nil {
					return err
				}
			}
		}
		return nil
	}

	return fmt.Errorf("unsupported type '%s'", val.Elem().Type())
}
