package proto_tcp

import (
	"fmt"
	"net"
)

func ServerUrl(host string, port int) string {
	return fmt.Sprintf("rosrpc://%s:%d", host, port)
}

type Server struct {
	ln net.Listener
}

func NewServer(port int) (*Server, error) {
	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port))
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
	return s.ln.Addr().(*net.TCPAddr).Port
}

func (s *Server) Accept() (*Conn, error) {
	nconn, err := s.ln.Accept()
	if err != nil {
		return nil, err
	}

	return NewConn(nconn), err
}
