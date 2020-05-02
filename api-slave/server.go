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

func (s *Server) Read() (interface{}, error) {
	for {
		raw, err := s.xs.Read()
		if err != nil {
			return nil, err
		}

		var req interface{}
		switch raw.Method {
		case "getPid":
			req = &ReqGetPid{}

		case "shutdown":
			req = &ReqShutdown{}

		case "publisherUpdate":
			req = &ReqPublisherUpdate{}

		case "requestTopic":
			req = &ReqRequestTopic{}

		default:
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

func (s *Server) WriteGetPid(code int, statusMessage string, pid int) {
	s.xs.Write(getPidRes{
		Code:          code,
		StatusMessage: statusMessage,
		Pid:           pid,
	})
}

func (s *Server) WritePublisherUpdate(code int, statusMessage string) {
	s.xs.Write(publisherUpdateRes{
		Code:          code,
		StatusMessage: statusMessage,
	})
}

func (s *Server) WriteRequestTopic(code int, statusMessage string, proto TopicProtocol) {
	s.xs.Write(requestTopicRes{
		Code:          code,
		StatusMessage: statusMessage,
		Proto:         proto,
	})
}
