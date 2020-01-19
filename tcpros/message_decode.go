package tcpros

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"reflect"
	"time"

	"github.com/aler9/goroslib/msg"
)

func messageDecode(r io.Reader, msg interface{}) error {
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

	// read message length
	_, err := io.ReadFull(r, buf[:4])
	if err != nil {
		return err
	}
	mlen := binary.LittleEndian.Uint32(buf)

	// read message
	err = messageDecodeValue(r, rv, &mlen, buf)
	if err != nil {
		return err
	}

	if mlen != 0 {
		return fmt.Errorf("message was partially parsed, %d bytes are unread", mlen)
	}

	return nil
}

func messageDecodeValue(r io.Reader, val reflect.Value, mlen *uint32, buf []byte) error {
	switch cv := val.Interface().(type) {
	case *msg.Bool:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		b := msg.Bool(false)
		if buf[0] == 0x01 {
			b = true
		}
		*cv = b
		return nil

	case *msg.Byte:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msg.Byte(buf[0])
		return nil

	case *msg.Char:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msg.Char(buf[0])
		return nil

	case *msg.Int8:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msg.Int8(buf[0])
		return nil

	case *msg.Uint8:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msg.Uint8(buf[0])
		return nil

	case *msg.Int16:
		_, err := io.ReadFull(r, buf[:2])
		if err != nil {
			return err
		}
		*mlen -= 2
		*cv = msg.Int16(binary.LittleEndian.Uint16(buf))
		return nil

	case *msg.Uint16:
		_, err := io.ReadFull(r, buf[:2])
		if err != nil {
			return err
		}
		*mlen -= 2
		*cv = msg.Uint16(binary.LittleEndian.Uint16(buf))
		return nil

	case *msg.Int32:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		*cv = msg.Int32(binary.LittleEndian.Uint32(buf))
		return nil

	case *msg.Uint32:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		*cv = msg.Uint32(binary.LittleEndian.Uint32(buf))
		return nil

	case *msg.Int64:
		_, err := io.ReadFull(r, buf[:8])
		if err != nil {
			return err
		}
		*mlen -= 8
		*cv = msg.Int64(binary.LittleEndian.Uint64(buf))
		return nil

	case *msg.Uint64:
		_, err := io.ReadFull(r, buf[:8])
		if err != nil {
			return err
		}
		*mlen -= 8
		*cv = msg.Uint64(binary.LittleEndian.Uint64(buf))
		return nil

	case *msg.Float32:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		*cv = msg.Float32(math.Float32frombits(binary.LittleEndian.Uint32(buf)))
		return nil

	case *msg.Float64:
		_, err := io.ReadFull(r, buf[:8])
		if err != nil {
			return err
		}
		*mlen -= 8
		*cv = msg.Float64(math.Float64frombits(binary.LittleEndian.Uint64(buf)))
		return nil

	case *msg.String:
		// string length
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		le := binary.LittleEndian.Uint32(buf)
		if le > *mlen {
			return fmt.Errorf("invalid message length")
		}

		if le > 0 {
			// string
			bstr := make([]byte, le)
			_, err = io.ReadFull(r, bstr)
			if err != nil {
				return err
			}
			*mlen -= le
			*cv = msg.String(bstr)
		} else {
			*cv = ""
		}
		return nil

	case *msg.Time:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		secs := int32(binary.LittleEndian.Uint32(buf))

		_, err = io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		nano := int32(binary.LittleEndian.Uint32(buf))

		// special case: zero means year zero, not 1970
		// so time.Time{} can be encoded / decoded
		if secs == 0 && nano == 0 {
			*cv = time.Time{}
		} else {
			*cv = time.Unix(int64(secs), int64(nano)).UTC()
		}
		return nil

	case *msg.Duration:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		secs := int32(binary.LittleEndian.Uint32(buf))

		_, err = io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		nano := int32(binary.LittleEndian.Uint32(buf))

		*cv = (time.Second * time.Duration(secs)) + (time.Nanosecond * time.Duration(nano))
		return nil
	}

	switch val.Elem().Kind() {
	case reflect.Slice:
		// slice length
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		le := binary.LittleEndian.Uint32(buf)
		if le > *mlen {
			return fmt.Errorf("invalid slice length")
		}

		// slice elements
		for i := 0; i < int(le); i++ {
			el := reflect.New(val.Elem().Type().Elem())

			if el.Elem().Kind() == reflect.Ptr {
				// allocate if is pointer and null
				if el.Elem().IsNil() {
					el.Elem().Set(reflect.New(el.Elem().Type().Elem()))
				}

				err := messageDecodeValue(r, el.Elem(), mlen, buf)
				if err != nil {
					return err
				}

			} else {
				err := messageDecodeValue(r, el, mlen, buf)
				if err != nil {
					return err
				}
			}

			val.Elem().Set(reflect.Append(val.Elem(), el.Elem()))
		}
		return nil

	case reflect.Array:
		// array elements
		le := val.Elem().Len()
		for i := 0; i < int(le); i++ {
			el := reflect.New(val.Elem().Type().Elem())

			if el.Elem().Kind() == reflect.Ptr {
				// allocate if is pointer and null
				if el.Elem().IsNil() {
					el.Elem().Set(reflect.New(el.Elem().Type().Elem()))
				}

				err := messageDecodeValue(r, el.Elem(), mlen, buf)
				if err != nil {
					return err
				}

			} else {
				err := messageDecodeValue(r, el, mlen, buf)
				if err != nil {
					return err
				}
			}

			val.Elem().Index(i).Set(el.Elem())
		}
		return nil

	case reflect.Struct:
		// struct fields
		nf := val.Elem().NumField()
		for i := 0; i < nf; i++ {
			el := val.Elem().Field(i)

			if el.Kind() == reflect.Ptr {
				// allocate if is pointer and null
				if el.IsNil() {
					el.Set(reflect.New(el.Type().Elem()))
				}

				err := messageDecodeValue(r, el, mlen, buf)
				if err != nil {
					return err
				}

			} else {
				err := messageDecodeValue(r, el.Addr(), mlen, buf)
				if err != nil {
					return err
				}
			}
		}
		return nil
	}

	return fmt.Errorf("unsupported field type '%s'", val.Elem().Type())
}
