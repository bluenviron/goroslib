package apislave

import (
	"net"
	"time"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

// ErrorRes is the error returned by the server in case of wrong or unhandled
// requests.
type ErrorRes xmlrpc.ErrorRes

// Server is a Slave API server.
type Server struct {
	nodeIP   net.IP
	nodeZone string

	xs *xmlrpc.Server
}

// NewServer allocates a Server.
func NewServer(address string, nodeIP net.IP, nodeZone string, writeTimeout time.Duration) (*Server, error) {
	xs, err := xmlrpc.NewServer(address, writeTimeout)
	if err != nil {
		return nil, err
	}

	s := &Server{
		nodeIP:   nodeIP,
		nodeZone: nodeZone,
		xs:       xs,
	}

	return s, nil
}

// Close closes the server.
func (s *Server) Close() error {
	return s.xs.Close()
}

// URL returns the server URL.
func (s *Server) URL() string {
	return s.xs.URL(s.nodeIP, s.nodeZone)
}

// Serve starts serving requests and waits until the server is closed.
func (s *Server) Serve(handler func(req Request) Response) {
	s.xs.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
		req := func() Request {
			switch raw.Method {
			case "getBusInfo":
				return &RequestGetBusInfo{}

			case "getPid":
				return &RequestGetPid{}

			case "getPublications":
				return &RequestGetPublications{}

			case "publisherUpdate":
				return &RequestPublisherUpdate{}

			case "requestTopic":
				return &RequestRequestTopic{}

			case "shutdown":
				return &RequestShutdown{}
			}
			return nil
		}()
		if req == nil {
			return xmlrpc.ErrorRes{}
		}

		err := raw.Decode(req)
		if err != nil {
			return xmlrpc.ErrorRes{}
		}

		return handler(req)
	})
}
