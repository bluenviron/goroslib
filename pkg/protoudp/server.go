package protoudp

import (
	"net"
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
	byts, err := f.encode()
	if err != nil {
		return err
	}

	_, err = s.ln.WriteTo(byts, dest)
	return err
}
