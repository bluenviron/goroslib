// Package protocommon contains functions and definitions for both TCPROS and UDPROS.
package protocommon

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"reflect"
	"time"

	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

const (
	maxMessageLength = 16 * 1024 * 1024
)

func readLimited(r io.Reader, buf []byte, mlen *int64) error {
	lb := int64(len(buf))

	if *mlen < lb {
		return fmt.Errorf("message length is too short")
	}

	_, err := io.ReadFull(r, buf)
	if err != nil {
		return err
	}

	*mlen -= lb

	return nil
}

func readLimitedDoNotCheck(r io.Reader, buf []byte, mlen *int64) error {
	lb := int64(len(buf))

	_, err := io.ReadFull(r, buf)
	if err != nil {
		return err
	}

	*mlen -= lb

	return nil
}

func binaryDecodeValue(r io.Reader, dest reflect.Value, mlen *int64, buf []byte) error {
	switch cv := dest.Interface().(type) {
	case *bool:
		err := readLimited(r, buf[:1], mlen)
		if err != nil {
			return err
		}

		b := false
		if buf[0] == 0x01 {
			b = true
		}
		*cv = b

	case *int8:
		err := readLimited(r, buf[:1], mlen)
		if err != nil {
			return err
		}

		*cv = int8(buf[0])

	case *uint8:
		err := readLimited(r, buf[:1], mlen)
		if err != nil {
			return err
		}

		*cv = buf[0]

	case *int16:
		err := readLimited(r, buf[:2], mlen)
		if err != nil {
			return err
		}

		*cv = int16(binary.LittleEndian.Uint16(buf))

	case *uint16:
		err := readLimited(r, buf[:2], mlen)
		if err != nil {
			return err
		}

		*cv = binary.LittleEndian.Uint16(buf)

	case *int32:
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}

		*cv = int32(binary.LittleEndian.Uint32(buf))

	case *uint32:
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}

		*cv = binary.LittleEndian.Uint32(buf)

	case *int64:
		err := readLimited(r, buf[:8], mlen)
		if err != nil {
			return err
		}

		*cv = int64(binary.LittleEndian.Uint64(buf))

	case *uint64:
		err := readLimited(r, buf[:8], mlen)
		if err != nil {
			return err
		}

		*cv = binary.LittleEndian.Uint64(buf)

	case *float32:
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}

		*cv = math.Float32frombits(binary.LittleEndian.Uint32(buf))

	case *float64:
		err := readLimited(r, buf[:8], mlen)
		if err != nil {
			return err
		}

		*cv = math.Float64frombits(binary.LittleEndian.Uint64(buf))

	case *string:
		// string length
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}

		le := int64(binary.LittleEndian.Uint32(buf))
		if le > *mlen {
			return fmt.Errorf("invalid string length")
		}

		// string
		if le > 0 {
			bstr := make([]byte, le)
			err := readLimitedDoNotCheck(r, bstr, mlen)
			if err != nil {
				return err
			}

			*cv = string(bstr)
		} else {
			*cv = ""
		}

	case *time.Time:
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}
		secs := int32(binary.LittleEndian.Uint32(buf))

		err = readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}
		nano := int32(binary.LittleEndian.Uint32(buf))

		// special case: zero means year zero, not 1970
		// so time.Time{} can be encoded / decoded
		if secs == 0 && nano == 0 {
			*cv = time.Time{}
		} else {
			*cv = time.Unix(int64(secs), int64(nano)).UTC()
		}

	case *time.Duration:
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}
		secs := int32(binary.LittleEndian.Uint32(buf))

		err = readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}
		nano := int32(binary.LittleEndian.Uint32(buf))

		*cv = (time.Second * time.Duration(secs)) + (time.Nanosecond * time.Duration(nano))

	case *[]uint8: // special case for performance
		// slice length
		err := readLimited(r, buf[:4], mlen)
		if err != nil {
			return err
		}

		le := int64(binary.LittleEndian.Uint32(buf))
		if le > *mlen {
			return fmt.Errorf("invalid array length")
		}

		// use preallocated slice if possible, allocate if too small
		if cap(*cv) < int(le) {
			*cv = make([]uint8, le)
		}

		err = readLimitedDoNotCheck(r, (*cv)[:le], mlen)
		if err != nil {
			return err
		}

	default:
		switch dest.Elem().Kind() {
		case reflect.Slice:
			// slice length
			err := readLimited(r, buf[:4], mlen)
			if err != nil {
				return err
			}

			le := int64(binary.LittleEndian.Uint32(buf))

			// slice elements
			for i := 0; i < int(le); i++ {
				el := reflect.New(dest.Elem().Type().Elem())
				el2 := el

				err := binaryDecodeValue(r, el2, mlen, buf)
				if err != nil {
					return err
				}

				dest.Elem().Set(reflect.Append(dest.Elem(), el.Elem()))
			}

		case reflect.Array:
			// array elements
			le := dest.Elem().Len()
			for i := 0; i < le; i++ {
				el := reflect.New(dest.Elem().Type().Elem())
				el2 := el

				err := binaryDecodeValue(r, el2, mlen, buf)
				if err != nil {
					return err
				}

				dest.Elem().Index(i).Set(el.Elem())
			}

		case reflect.Struct:
			// struct fields
			nf := dest.Elem().NumField()
			for i := 0; i < nf; i++ {
				f := dest.Elem().Field(i)
				ft := dest.Elem().Type().Field(i)

				if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Package(0)) {
					continue
				}

				if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Name(0)) {
					continue
				}

				if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Definitions(0)) {
					continue
				}

				f = f.Addr()

				err := binaryDecodeValue(r, f, mlen, buf)
				if err != nil {
					return err
				}
			}
		}
	}

	return nil
}

// MessageDecode decodes a message in binary format.
func MessageDecode(r io.Reader, dest interface{}) error {
	rv := reflect.ValueOf(dest)
	if rv.Kind() != reflect.Ptr || rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("destination must be a pointer to a struct")
	}

	// use a shared buffer for performance reasons
	buf := make([]byte, 8)

	// message length
	_, err := io.ReadFull(r, buf[:4])
	if err != nil {
		return err
	}
	mlen := int64(binary.LittleEndian.Uint32(buf))

	if mlen > maxMessageLength {
		return fmt.Errorf("maximum message length exceeded")
	}

	// message
	err = binaryDecodeValue(r, rv, &mlen, buf)
	if err != nil {
		return err
	}

	if mlen != 0 {
		return fmt.Errorf("message was partially parsed, %d bytes are unread", mlen)
	}

	return nil
}

func binaryEncodeValue(w *bytes.Buffer, src reflect.Value, dest []byte) {
	switch cv := src.Elem().Interface().(type) {
	case bool:
		b := uint8(0x00)
		if cv {
			b = 0x01
		}
		w.Write([]byte{b})

	case int8:
		w.Write([]byte{uint8(cv)})

	case uint8:
		w.Write([]byte{cv})

	case int16:
		binary.LittleEndian.PutUint16(dest, uint16(cv))
		w.Write(dest[:2])

	case uint16:
		binary.LittleEndian.PutUint16(dest, cv)
		w.Write(dest[:2])

	case int32:
		binary.LittleEndian.PutUint32(dest, uint32(cv))
		w.Write(dest[:4])

	case uint32:
		binary.LittleEndian.PutUint32(dest, cv)
		w.Write(dest[:4])

	case int64:
		binary.LittleEndian.PutUint64(dest, uint64(cv))
		w.Write(dest[:8])

	case uint64:
		binary.LittleEndian.PutUint64(dest, cv)
		w.Write(dest[:8])

	case float32:
		binary.LittleEndian.PutUint32(dest, math.Float32bits(cv))
		w.Write(dest[:4])

	case float64:
		binary.LittleEndian.PutUint64(dest, math.Float64bits(cv))
		w.Write(dest[:8])

	case string:
		bstr := []byte(cv)

		// string length
		binary.LittleEndian.PutUint32(dest, uint32(len(bstr)))
		w.Write(dest[:4])

		// string
		w.Write(bstr)

	case time.Time:
		// special case: zero means year zero, not 1970
		// so time.Time{} can be encoded / decoded
		var nano int64
		var zero time.Time
		if cv == zero { //nolint:staticcheck
			nano = 0
		} else {
			nano = cv.UnixNano()
		}

		binary.LittleEndian.PutUint32(dest, uint32(nano/1000000000))
		w.Write(dest[:4])

		binary.LittleEndian.PutUint32(dest, uint32(nano%1000000000))
		w.Write(dest[:4])

	case time.Duration:
		nano := cv.Nanoseconds()

		binary.LittleEndian.PutUint32(dest, uint32(nano/1000000000))
		w.Write(dest[:4])

		binary.LittleEndian.PutUint32(dest, uint32(nano%1000000000))
		w.Write(dest[:4])

	case []uint8: // special case for performance
		binary.LittleEndian.PutUint32(dest, uint32(len(cv)))
		w.Write(dest[:4])
		w.Write(cv)

	default:
		switch src.Elem().Kind() {
		case reflect.Slice:
			le := src.Elem().Len()

			// slice length
			binary.LittleEndian.PutUint32(dest, uint32(le))
			w.Write(dest[:4])

			// slice elements
			for i := 0; i < le; i++ {
				el := src.Elem().Index(i).Addr()

				binaryEncodeValue(w, el, dest)
			}

		case reflect.Array:
			le := src.Elem().Len()

			// array elements
			for i := 0; i < le; i++ {
				el := src.Elem().Index(i).Addr()

				binaryEncodeValue(w, el, dest)
			}

		case reflect.Struct:
			// struct fields
			nf := src.Elem().NumField()
			for i := 0; i < nf; i++ {
				ft := src.Elem().Type().Field(i)

				if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Package(0)) {
					continue
				}

				if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Name(0)) {
					continue
				}

				if ft.Anonymous && ft.Type == reflect.TypeOf(msg.Definitions(0)) {
					continue
				}

				f := src.Elem().Field(i).Addr()
				binaryEncodeValue(w, f, dest)
			}
		}
	}
}

// MessageEncode encodes a message in binary format.
func MessageEncode(w io.Writer, src interface{}) error {
	rv := reflect.ValueOf(src)
	if rv.Kind() != reflect.Ptr || rv.Elem().Kind() != reflect.Struct {
		return fmt.Errorf("src must be a pointer to a struct")
	}

	// use a shared buffer to improve performance
	buf := make([]byte, 8)

	// encode message
	var vw bytes.Buffer
	binaryEncodeValue(&vw, rv, buf)

	// write message length
	binary.LittleEndian.PutUint32(buf, uint32(vw.Len()))
	_, err := w.Write(buf[:4])
	if err != nil {
		return err
	}

	// write message
	_, err = w.Write(vw.Bytes())
	return err
}
