package prototcp

import (
	"bufio"
	"fmt"
	"io"
	"net"
	"unicode"

	"github.com/aler9/goroslib/pkg/protocommon"
)

const (
	bufferSize = 2048
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

type Conn struct {
	readBuf  *bufio.Reader
	writeBuf *bufio.Writer
	closer   io.Closer
}

// NewConn allocates a Conn, that takes ownership of the input net.Conn.
func NewConn(in net.Conn) *Conn {
	return &Conn{
		readBuf:  bufio.NewReaderSize(in, bufferSize),
		writeBuf: bufio.NewWriterSize(in, bufferSize),
		closer:   in,
	}
}

func (c *Conn) Close() error {
	return c.closer.Close()
}

func (c *Conn) ReadHeaderRaw() (protocommon.HeaderRaw, error) {
	return protocommon.HeaderRawDecode(c.readBuf)
}

func (c *Conn) ReadHeader(header protocommon.Header) error {
	raw, err := protocommon.HeaderRawDecode(c.readBuf)
	if err != nil {
		return err
	}

	if strErr, ok := raw["error"]; ok {
		return fmt.Errorf(strErr)
	}

	return protocommon.HeaderDecode(raw, header)
}

func (c *Conn) ReadServiceResState() error {
	byt := make([]byte, 1)
	_, err := io.ReadFull(c.readBuf, byt)
	if err != nil {
		return err
	}

	if byt[0] != 1 {
		return fmt.Errorf("service returned an error")
	}

	return nil
}

func (c *Conn) ReadMessage(msg interface{}) error {
	return protocommon.MessageDecode(c.readBuf, msg)
}

func (c *Conn) WriteHeader(header protocommon.Header) error {
	err := protocommon.HeaderEncode(c.writeBuf, header)
	if err != nil {
		return err
	}
	return c.writeBuf.Flush()
}

func (c *Conn) WriteServiceResState() error {
	// do not flush, since WriteServiceResState() is always called before a WriteMessage()
	_, err := c.writeBuf.Write([]byte{1})
	return err
}

func (c *Conn) WriteMessage(msg interface{}) error {
	err := protocommon.MessageEncode(c.writeBuf, msg)
	if err != nil {
		return err
	}
	return c.writeBuf.Flush()
}
