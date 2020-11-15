package prototcp

import (
	"net"
	"strconv"
)

func ServerUrl(host string, port int) string {
	return "rosrpc://" + host + ":" + strconv.FormatInt(int64(port), 10)
}

type Server struct {
	ln net.Listener
}

func NewServer(port int) (*Server, error) {
	ln, err := net.Listen("tcp4", ":"+strconv.FormatInt(int64(port), 10))
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
