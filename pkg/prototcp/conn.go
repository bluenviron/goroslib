package prototcp

import (
	"bufio"
	"io"
	"net"

	"github.com/aler9/goroslib/pkg/protocommon"
)

const (
	bufferSize = 2048
)

// Conn is a TCPROS connection.
type Conn struct {
	nconn    net.Conn
	readBuf  *bufio.Reader
	writeBuf *bufio.Writer
}

func newConn(nconn net.Conn) *Conn {
	return &Conn{
		nconn:    nconn,
		readBuf:  bufio.NewReaderSize(nconn, bufferSize),
		writeBuf: bufio.NewWriterSize(nconn, bufferSize),
	}
}

// Close closes the connection.
func (c *Conn) Close() error {
	return c.nconn.Close()
}

// NetConn returns the underlying net.Conn.
func (c *Conn) NetConn() net.Conn {
	return c.nconn
}

// ReadHeaderRaw reads an HeaderRaw.
func (c *Conn) ReadHeaderRaw() (protocommon.HeaderRaw, error) {
	return protocommon.HeaderRawDecode(c.readBuf)
}

// WriteHeader writes an header.
func (c *Conn) WriteHeader(header protocommon.Header) error {
	err := protocommon.HeaderEncode(c.writeBuf, header)
	if err != nil {
		return err
	}
	return c.writeBuf.Flush()
}

// ReadServiceResState reads the response of a service state request.
func (c *Conn) ReadServiceResState() (uint8, error) {
	byt := make([]byte, 1)
	_, err := io.ReadFull(c.readBuf, byt)
	if err != nil {
		return 0, err
	}

	return byt[0], nil
}

// WriteServiceResState writes the response of a service state request.
func (c *Conn) WriteServiceResState(v uint8) error {
	_, err := c.writeBuf.Write([]byte{v})
	// do not flush, since WriteServiceResState() is always called before a WriteMessage()
	return err
}

// ReadMessage reads a message.
func (c *Conn) ReadMessage(msg interface{}) error {
	return protocommon.MessageDecode(c.readBuf, msg)
}

// WriteMessage writes a message.
func (c *Conn) WriteMessage(msg interface{}) error {
	err := protocommon.MessageEncode(c.writeBuf, msg)
	if err != nil {
		return err
	}
	return c.writeBuf.Flush()
}
