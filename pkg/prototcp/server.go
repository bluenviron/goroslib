package prototcp

import (
	"net"
	"net/url"
)

// Server is a TCPROS server.
type Server struct {
	nodeIP   net.IP
	nodeZone string

	ln net.Listener
}

// NewServer allocates a Server.
func NewServer(address string, nodeIP net.IP, nodeZone string) (*Server, error) {
	ln, err := net.Listen("tcp", address)
	if err != nil {
		return nil, err
	}

	return &Server{
		nodeIP:   nodeIP,
		nodeZone: nodeZone,
		ln:       ln,
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

// URL returns the server URL.
func (s *Server) URL() string {
	return (&url.URL{
		Scheme: "rosrpc",
		Host: (&net.TCPAddr{
			IP:   s.nodeIP,
			Port: s.Port(),
			Zone: s.nodeZone,
		}).String(),
	}).String()
}

// Accept accepts clients.
func (s *Server) Accept() (*Conn, error) {
	nconn, err := s.ln.Accept()
	if err != nil {
		return nil, err
	}

	return newConn(nconn), err
}
