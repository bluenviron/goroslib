package api_slave

import (
	"github.com/aler9/goroslib/xmlrpc"
)

type ErrorRes xmlrpc.ErrorRes

func (ErrorRes) isResponse() {}

type Server struct {
	xs *xmlrpc.Server
}

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

func (s *Server) Port() int {
	return s.xs.Port()
}

func (s *Server) Close() error {
	return s.xs.Close()
}

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
