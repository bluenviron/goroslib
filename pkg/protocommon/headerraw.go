package protocommon

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
)

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
