package prototcp

import (
	"bufio"
	"io"
	"net"
	"time"

	"github.com/aler9/goroslib/pkg/protocommon"
)

const (
	bufferSize   = 2048
	writeTimeout = 10 * time.Second
	readTimeout  = 10 * time.Second
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
	c.nconn.SetReadDeadline(time.Now().Add(readTimeout))
	return protocommon.HeaderRawDecode(c.readBuf)
}

// WriteHeader writes an header.
func (c *Conn) WriteHeader(header protocommon.Header) error {
	c.nconn.SetWriteDeadline(time.Now().Add(writeTimeout))
	err := protocommon.HeaderEncode(c.writeBuf, header)
	if err != nil {
		return err
	}
	return c.writeBuf.Flush()
}

// ReadServiceResState reads the response of a service state request.
func (c *Conn) ReadServiceResState() (bool, error) {
	c.nconn.SetReadDeadline(time.Now().Add(readTimeout))
	byt := make([]byte, 1)
	_, err := io.ReadFull(c.readBuf, byt)
	if err != nil {
		return false, err
	}

	return (byt[0] == 1), nil
}

// WriteServiceResState writes the response of a service state request.
func (c *Conn) WriteServiceResState(v bool) error {
	b := byte(0)
	if v {
		b = 1
	}

	_, err := c.writeBuf.Write([]byte{b})
	// do call Flush() nor SetWriteDeadline()
	// since WriteServiceResState() is always called before a WriteMessage()
	return err
}

// ReadMessage reads a message.
func (c *Conn) ReadMessage(msg interface{}, timeout bool) error {
	if timeout {
		c.nconn.SetReadDeadline(time.Now().Add(readTimeout))
	} else {
		c.nconn.SetReadDeadline(time.Time{})
	}
	return protocommon.MessageDecode(c.readBuf, msg)
}

// WriteMessage writes a message.
func (c *Conn) WriteMessage(msg interface{}) error {
	c.nconn.SetWriteDeadline(time.Now().Add(writeTimeout))
	err := protocommon.MessageEncode(c.writeBuf, msg)
	if err != nil {
		return err
	}
	return c.writeBuf.Flush()
}
