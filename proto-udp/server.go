package proto_udp

import (
	"fmt"
	"net"
)

const (
	_BUFFER_SIZE = 2048
)

type Server struct {
	ln net.PacketConn
}

func NewServer(host string, port int) (*Server, error) {
	ln, err := net.ListenPacket("udp", fmt.Sprintf(":%d", port))
	if err != nil {
		return nil, err
	}

	return &Server{
		ln: ln,
	}, nil
}

func (s *Server) Close() error {
	return s.ln.Close()
}

func (s *Server) Port() int {
	return s.ln.LocalAddr().(*net.UDPAddr).Port
}

func (s *Server) ReadFrame() (*Frame, *net.UDPAddr, error) {
	buf := make([]byte, _BUFFER_SIZE)

	n, source, err := s.ln.ReadFrom(buf)
	if err != nil {
		return nil, nil, err
	}

	f, err := frameDecode(buf[:n])
	if err != nil {
		return nil, nil, err
	}

	return f, source.(*net.UDPAddr), nil
}

func (s *Server) WriteFrame(f *Frame, dest *net.UDPAddr) error {
	byts, err := frameEncode(f)
	if err != nil {
		return err
	}

	_, err = s.ln.WriteTo(byts, dest)
	return err
}
