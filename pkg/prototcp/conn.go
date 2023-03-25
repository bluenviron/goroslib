package prototcp

import (
	"bufio"
	"io"

	"github.com/bluenviron/goroslib/v2/pkg/protocommon"
)

// Conn is a TCPROS connection.
type Conn struct {
	rb *bufio.Reader
	wb *bufio.Writer
}

// NewConn allocates a Conn.
func NewConn(rw io.ReadWriter) *Conn {
	return &Conn{
		rb: bufio.NewReader(rw),
		wb: bufio.NewWriter(rw),
	}
}

// ReadHeaderRaw reads an HeaderRaw.
func (c *Conn) ReadHeaderRaw() (protocommon.HeaderRaw, error) {
	return protocommon.HeaderRawDecode(c.rb)
}

// WriteHeader writes an header.
func (c *Conn) WriteHeader(header protocommon.Header) error {
	err := protocommon.HeaderEncode(c.wb, header)
	if err != nil {
		return err
	}
	return c.wb.Flush()
}

// ReadServiceResponse reads the response of a service request.
func (c *Conn) ReadServiceResponse(msg interface{}) (bool, error) {
	// read state
	byt := make([]byte, 1)
	_, err := io.ReadFull(c.rb, byt)
	if err != nil {
		return false, err
	}
	state := (byt[0] == 1)

	// stop if state is false
	if !state {
		return state, nil
	}

	// read message
	err = protocommon.MessageDecode(c.rb, msg)
	return state, err
}

// WriteServiceResponse writes the response of a service request.
func (c *Conn) WriteServiceResponse(state bool, res interface{}) error {
	// write state
	b := byte(0)
	if state {
		b = 1
	}
	_, err := c.wb.Write([]byte{b})
	if err != nil {
		return err
	}

	// stop if state is false
	if !state {
		return c.wb.Flush()
	}

	// write message
	err = protocommon.MessageEncode(c.wb, res)
	if err != nil {
		return err
	}

	return c.wb.Flush()
}

// ReadMessage reads a message.
func (c *Conn) ReadMessage(msg interface{}) error {
	return protocommon.MessageDecode(c.rb, msg)
}

// WriteMessage writes a message.
func (c *Conn) WriteMessage(msg interface{}) error {
	err := protocommon.MessageEncode(c.wb, msg)
	if err != nil {
		return err
	}
	return c.wb.Flush()
}
