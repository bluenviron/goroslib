package api_slave

import (
	"github.com/aler9/goroslib/xmlrpc"
)

type Server struct {
	xs *xmlrpc.Server
}

func NewServer(host string, port uint16) (*Server, error) {
	xs, err := xmlrpc.NewServer(host, port)
	if err != nil {
		return nil, err
	}

	s := &Server{
		xs: xs,
	}

	return s, nil
}

func (s *Server) Close() error {
	return s.xs.Close()
}

func (s *Server) GetUrl() string {
	return s.xs.GetUrl()
}

func (s *Server) Read() (Request, error) {
	for {
		raw, err := s.xs.Read()
		if err != nil {
			return nil, err
		}

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
			s.xs.Write(xmlrpc.ErrorRes{})
			continue
		}

		err = raw.Decode(req)
		if err != nil {
			s.xs.Write(xmlrpc.ErrorRes{})
			continue
		}

		return req, nil
	}
}

func (s *Server) Write(res Response) {
	s.xs.Write(res)
}
