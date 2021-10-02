package protoudp

import (
	"bytes"
	"net"

	"github.com/aler9/goroslib/pkg/protocommon"
)

const (
	bufferSize = 2048
)

// Server is a UDPROS server.
type Server struct {
	ln net.PacketConn
}

// NewServer allocates a Server.
func NewServer(address string) (*Server, error) {
	ln, err := net.ListenPacket("udp", address)
	if err != nil {
		return nil, err
	}

	return &Server{
		ln: ln,
	}, nil
}

// Close closes the server.
func (s *Server) Close() error {
	return s.ln.Close()
}

// Port returns the server port.
func (s *Server) Port() int {
	return s.ln.LocalAddr().(*net.UDPAddr).Port
}

// ReadFrame reads a frame.
func (s *Server) ReadFrame() (*Frame, *net.UDPAddr, error) {
	buf := make([]byte, bufferSize)

	n, source, err := s.ln.ReadFrom(buf)
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
func (s *Server) WriteFrame(f *Frame, dest *net.UDPAddr) error {
	_, err := s.ln.WriteTo(f.encode(), dest)
	return err
}

// WriteMessage writes a message.
func (s *Server) WriteMessage(pubID int, messageID uint8, msg interface{}, dest *net.UDPAddr) error {
	var rawMessage bytes.Buffer
	err := protocommon.MessageEncode(&rawMessage, msg)
	if err != nil {
		return err
	}

	frames := FramesForPayload(
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
