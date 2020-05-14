package xmlrpc

import (
	"fmt"
	"net"
	"net/http"
)

// special response that sends status code 400
// the C++ implementation in case of errors replies with a
// <methodResponse><fault><value>..., which would require additional parsing
type ErrorRes struct{}

type Server struct {
	host  string
	port  uint16
	ln    net.Listener
	hs    *http.Server
	read  chan *RequestRaw
	write chan interface{}
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

	s := &Server{
		host:  host,
		port:  port,
		ln:    ln,
		read:  make(chan *RequestRaw),
		write: make(chan interface{}),
	}

	s.hs = &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
			if req.URL.Path != "/RPC2" {
				w.WriteHeader(400)
				return
			}

			if req.Method != "POST" {
				w.WriteHeader(400)
				return
			}

			raw, err := requestDecodeRaw(req.Body)
			if err != nil {
				w.WriteHeader(400)
				return
			}

			s.read <- raw

			res, ok := <-s.write
			if !ok {
				return
			}

			if _, ok := res.(ErrorRes); ok {
				w.WriteHeader(400)
			} else {
				responseEncode(w, res)
			}
		}),
	}

	go func() {
		s.hs.Serve(s.ln)
	}()

	return s, nil
}

func (s *Server) Close() error {
	// consume read
	go func() {
		for range s.read {
		}
	}()

	s.ln.Close()
	s.hs.Close()
	close(s.read)
	close(s.write)
	return nil
}

func (s *Server) GetUrl() string {
	return fmt.Sprintf("http://%s:%d", s.host, s.port)
}

func (s *Server) Read() (*RequestRaw, error) {
	raw, ok := <-s.read
	if !ok {
		return nil, fmt.Errorf("closed")
	}
	return raw, nil
}

func (s *Server) Write(res interface{}) {
	s.write <- res
}
