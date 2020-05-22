package xmlrpc

import (
	"context"
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
	read  chan *RequestRaw
	write chan interface{}
	done  chan struct{}
}

func NewServer(host string, port uint16) (*Server, error) {
	// net.Listen and http.Server are splitted since the latter
	// does not allow to use 0 as port
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
		done:  make(chan struct{}),
	}

	go s.run()

	return s, nil
}

func (s *Server) run() {
	hs := &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
			if req.URL.Path != "/RPC2" {
				w.WriteHeader(http.StatusBadRequest)
				return
			}

			if req.Method != "POST" {
				w.WriteHeader(http.StatusBadRequest)
				return
			}

			raw, err := requestDecodeRaw(req.Body)
			if err != nil {
				w.WriteHeader(http.StatusBadRequest)
				return
			}

			s.read <- raw

			res, ok := <-s.write
			if !ok {
				return
			}

			if _, ok := res.(ErrorRes); ok {
				w.WriteHeader(http.StatusBadRequest)
			} else {
				responseEncode(w, res)
			}
		}),
	}

	hs.Serve(s.ln)

	// consume read
	go func() {
		for range s.read {
		}
	}()

	// wait for all handlers to return
	hs.Shutdown(context.Background())

	close(s.read)
	close(s.write)

	close(s.done)
}

func (s *Server) Close() error {
	s.ln.Close()
	<-s.done
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
