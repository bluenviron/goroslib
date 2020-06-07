package proto_tcp

import (
	"fmt"
	"net"
)

type Server struct {
	host string
	ln   net.Listener
}

func NewServer(host string, port int) (*Server, error) {
	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port))
	if err != nil {
		return nil, err
	}

	return &Server{
		host: host,
		ln:   ln,
	}, nil
}

func (s *Server) Close() error {
	return s.ln.Close()
}

func (s *Server) Port() int {
	return s.ln.Addr().(*net.TCPAddr).Port
}

func (s *Server) GetUrl() string {
	return fmt.Sprintf("rosrpc://%s:%d", s.host, s.Port())
}

func (s *Server) Accept() (*Conn, error) {
	nconn, err := s.ln.Accept()
	if err != nil {
		return nil, err
	}

	return NewConn(nconn), err
}
