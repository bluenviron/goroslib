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

// ReadServiceResponse reads the response of a service request.
func (c *Conn) ReadServiceResponse(msg interface{}) (bool, error) {
	c.nconn.SetReadDeadline(time.Now().Add(readTimeout))

	// read state
	byt := make([]byte, 1)
	_, err := io.ReadFull(c.readBuf, byt)
	if err != nil {
		return false, err
	}
	state := (byt[0] == 1)

	// stop if state is false
	if !state {
		return state, nil
	}

	// read message
	err = protocommon.MessageDecode(c.readBuf, msg)
	return state, err
}

// WriteServiceResponse writes the response of a service request.
func (c *Conn) WriteServiceResponse(state bool, res interface{}) error {
	c.nconn.SetWriteDeadline(time.Now().Add(writeTimeout))

	// write state
	b := byte(0)
	if state {
		b = 1
	}
	_, err := c.writeBuf.Write([]byte{b})
	if err != nil {
		return err
	}

	// stop if state is false
	if !state {
		return nil
	}

	// write message
	err = protocommon.MessageEncode(c.writeBuf, res)
	if err != nil {
		return err
	}

	return c.writeBuf.Flush()
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
