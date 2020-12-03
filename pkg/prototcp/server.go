package prototcp

import (
	"net"
	"net/url"
	"strconv"
)

func ServerUrl(address *net.TCPAddr, port int) string {
	return (&url.URL{
		Scheme: "rosrpc",
		Host: (&net.TCPAddr{
			IP:   address.IP,
			Port: port,
			Zone: address.Zone,
		}).String(),
	}).String()
}

// Server is a TCPROS server.
type Server struct {
	ln net.Listener
}

// NewServer allocates a Server.
func NewServer(port int) (*Server, error) {
	ln, err := net.Listen("tcp", ":"+strconv.FormatInt(int64(port), 10))
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
	return s.ln.Addr().(*net.TCPAddr).Port
}

// Accept accepts clients.
func (s *Server) Accept() (*Conn, error) {
	nconn, err := s.ln.Accept()
	if err != nil {
		return nil, err
	}

	return NewConn(nconn.(*net.TCPConn)), err
}
