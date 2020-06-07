package proto_tcp

import (
	"fmt"
	"net"
)

type Server struct {
	host string
	port uint16
	ln   net.Listener
}

func NewServer(host string, port uint16) (*Server, error) {
	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port))
	if err != nil {
		return nil, err
	}

	// if port was chosen automatically, get it
	if port == 0 {
		port = uint16(ln.Addr().(*net.TCPAddr).Port)
	}

	return &Server{
		host: host,
		port: port,
		ln:   ln,
	}, nil
}

func (s *Server) Close() error {
	return s.ln.Close()
}

func (s *Server) Port() uint16 {
	return s.port
}

func (s *Server) GetUrl() string {
	return fmt.Sprintf("rosrpc://%s:%d", s.host, s.port)
}

func (s *Server) Accept() (*Conn, error) {
	nconn, err := s.ln.Accept()
	if err != nil {
		return nil, err
	}

	return NewConn(nconn), err
}
