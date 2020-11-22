package apislave

import (
	"github.com/aler9/goroslib/pkg/xmlrpc"
)

type ErrorRes xmlrpc.ErrorRes

func (ErrorRes) isResponse() {}

// Server is a Slave API server.
type Server struct {
	xs *xmlrpc.Server
}

// NewServer allocates a Server.
func NewServer(port int) (*Server, error) {
	xs, err := xmlrpc.NewServer(port)
	if err != nil {
		return nil, err
	}

	s := &Server{
		xs: xs,
	}

	return s, nil
}

// Close closes the server.
func (s *Server) Close() error {
	return s.xs.Close()
}

// Port returns the server port.
func (s *Server) Port() int {
	return s.xs.Port()
}

// Handle sets a callback that is called when a request arrives.
func (s *Server) Handle(cb func(req Request) Response) {
	s.xs.Handle(func(raw *xmlrpc.RequestRaw) interface{} {
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

		return cb(req)
	})
}
