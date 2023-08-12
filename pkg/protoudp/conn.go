package protoudp

import (
	"bytes"
	"net"

	"github.com/bluenviron/goroslib/v2/pkg/protocommon"
)

const (
	bufferSize = 2048
)

// PacketConn is a subset of net.PacketConn.
type PacketConn interface {
	ReadFrom(p []byte) (n int, addr net.Addr, err error)
	WriteTo(p []byte, addr net.Addr) (n int, err error)
}

// Conn is a UDPROS connection.
type Conn struct {
	pc PacketConn
}

// NewConn allocates a Conn.
func NewConn(pc PacketConn) *Conn {
	return &Conn{
		pc: pc,
	}
}

// ReadFrame reads a frame.
func (s *Conn) ReadFrame() (*Frame, *net.UDPAddr, error) {
	buf := make([]byte, bufferSize)

	n, source, err := s.pc.ReadFrom(buf)
	if err != nil {
		return nil, nil, err
	}

	var f Frame
	err = f.decode(buf[:n])
	if err != nil {
		return nil, nil, err
	}

	return &f, source.(*net.UDPAddr), nil
}

// WriteFrame writes a frame.
func (s *Conn) WriteFrame(f *Frame, dest *net.UDPAddr) error {
	_, err := s.pc.WriteTo(f.encode(), dest)
	return err
}

// WriteMessage writes a message.
func (s *Conn) WriteMessage(pubID int, messageID uint8, msg interface{}, dest *net.UDPAddr) error {
	var rawMessage bytes.Buffer
	err := protocommon.MessageEncode(&rawMessage, msg)
	if err != nil {
		return err
	}

	frames := framesForPayload(
		uint32(pubID),
		messageID,
		rawMessage.Bytes())

	for _, f := range frames {
		err := s.WriteFrame(f, dest)
		if err != nil {
			return err
		}
	}

	return nil
}
