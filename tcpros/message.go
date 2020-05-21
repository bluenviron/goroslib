package tcpros

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"reflect"
	"time"

	"github.com/aler9/goroslib/msgs"
)

func messageDecodeValue(r io.Reader, val reflect.Value, mlen *uint32, buf []byte) error {
	switch cv := val.Interface().(type) {
	case *msgs.Bool:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		b := msgs.Bool(false)
		if buf[0] == 0x01 {
			b = true
		}
		*cv = b
		return nil

	case *msgs.Byte:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msgs.Byte(buf[0])
		return nil

	case *msgs.Char:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msgs.Char(buf[0])
		return nil

	case *msgs.Int8:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msgs.Int8(buf[0])
		return nil

	case *msgs.Uint8:
		_, err := io.ReadFull(r, buf[:1])
		if err != nil {
			return err
		}
		*mlen -= 1
		*cv = msgs.Uint8(buf[0])
		return nil

	case *msgs.Int16:
		_, err := io.ReadFull(r, buf[:2])
		if err != nil {
			return err
		}
		*mlen -= 2
		*cv = msgs.Int16(binary.LittleEndian.Uint16(buf))
		return nil

	case *msgs.Uint16:
		_, err := io.ReadFull(r, buf[:2])
		if err != nil {
			return err
		}
		*mlen -= 2
		*cv = msgs.Uint16(binary.LittleEndian.Uint16(buf))
		return nil

	case *msgs.Int32:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		*cv = msgs.Int32(binary.LittleEndian.Uint32(buf))
		return nil

	case *msgs.Uint32:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		*cv = msgs.Uint32(binary.LittleEndian.Uint32(buf))
		return nil

	case *msgs.Int64:
		_, err := io.ReadFull(r, buf[:8])
		if err != nil {
			return err
		}
		*mlen -= 8
		*cv = msgs.Int64(binary.LittleEndian.Uint64(buf))
		return nil

	case *msgs.Uint64:
		_, err := io.ReadFull(r, buf[:8])
		if err != nil {
			return err
		}
		*mlen -= 8
		*cv = msgs.Uint64(binary.LittleEndian.Uint64(buf))
		return nil

	case *msgs.Float32:
		_, err := io.ReadFull(r, buf[:4])
		if err != nil {
			return err
		}
		*mlen -= 4
		*cv = msgs.Float32(math.Float32frombits(binary.LittleEndian.Uint32(buf)))
		return nil

	case *msgs.Float64:
		_, err := io.ReadFull(r, buf[:8])
		if err != nil {
			return err
		}
		*mlen -= 8
		*cv = msgs.Float64(math.Float64frombits(binary.LittleEndian.Uint64(buf)))
		return nil

	case *msgs.String:
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
			*cv = msgs.String(bstr)
		} else {
			*cv = ""
		}
		return nil

	case *msgs.Time:
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

	case *msgs.Duration:
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
			f := val.Elem().Field(i)
			ft := val.Elem().Type().Field(i)

			if ft.Name == "Package" && ft.Anonymous && ft.Type == reflect.TypeOf(msgs.Package(0)) {
				continue
			}

			if f.Kind() == reflect.Ptr {
				// allocate if is pointer and null
				if f.IsNil() {
					f.Set(reflect.New(f.Type().Elem()))
				}

				err := messageDecodeValue(r, f, mlen, buf)
				if err != nil {
					return err
				}

			} else {
				err := messageDecodeValue(r, f.Addr(), mlen, buf)
				if err != nil {
					return err
				}
			}
		}
		return nil
	}

	return fmt.Errorf("unsupported field type '%s'", val.Elem().Type())
}

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

func messageEncodeValue(w io.Writer, val reflect.Value, buf []byte) error {
	switch cv := val.Elem().Interface().(type) {
	case msgs.Bool:
		b := uint8(0x00)
		if cv {
			b = 0x01
		}
		_, err := w.Write([]byte{b})
		return err

	case msgs.Byte:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msgs.Char:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msgs.Int8:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msgs.Uint8:
		_, err := w.Write([]byte{uint8(cv)})
		return err

	case msgs.Int16:
		binary.LittleEndian.PutUint16(buf, uint16(cv))
		_, err := w.Write(buf[:2])
		return err

	case msgs.Uint16:
		binary.LittleEndian.PutUint16(buf, uint16(cv))
		_, err := w.Write(buf[:2])
		return err

	case msgs.Int32:
		binary.LittleEndian.PutUint32(buf, uint32(cv))
		_, err := w.Write(buf[:4])
		return err

	case msgs.Uint32:
		binary.LittleEndian.PutUint32(buf, uint32(cv))
		_, err := w.Write(buf[:4])
		return err

	case msgs.Int64:
		binary.LittleEndian.PutUint64(buf, uint64(cv))
		_, err := w.Write(buf[:8])
		return err

	case msgs.Uint64:
		binary.LittleEndian.PutUint64(buf, uint64(cv))
		_, err := w.Write(buf[:8])
		return err

	case msgs.Float32:
		binary.LittleEndian.PutUint32(buf, math.Float32bits(float32(cv)))
		_, err := w.Write(buf[:4])
		return err

	case msgs.Float64:
		binary.LittleEndian.PutUint64(buf, math.Float64bits(float64(cv)))
		_, err := w.Write(buf[:8])
		return err

	case msgs.String:
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

	case msgs.Time:
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

	case msgs.Duration:
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
			f := val.Elem().Field(i)
			ft := val.Elem().Type().Field(i)

			if ft.Name == "Package" && ft.Anonymous && ft.Type == reflect.TypeOf(msgs.Package(0)) {
				continue
			}

			if f.Kind() == reflect.Ptr {
				err := messageEncodeValue(w, f, buf)
				if err != nil {
					return err
				}
			} else {
				err := messageEncodeValue(w, f.Addr(), buf)
				if err != nil {
					return err
				}
			}
		}
		return nil
	}

	return fmt.Errorf("unsupported type '%s'", val.Elem().Type())
}

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
