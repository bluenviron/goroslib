package xmlrpc

import (
	"context"
	"net"
	"net/http"
	"net/url"
	"strconv"
	"sync"
)

// ErrorRes is a special response that sends status code 400.
// in case of errors, the C++ implementation replies with
// <methodResponse><fault><value>..., a structure which would require additional parsing.
type ErrorRes struct{}

// ServerURL returns a XMLRPC server url.
func ServerURL(address *net.TCPAddr, port int) string {
	return (&url.URL{
		Scheme: "http",
		Host: (&net.TCPAddr{
			IP:   address.IP,
			Port: port,
			Zone: address.Zone,
		}).String(),
	}).String()
}

// Server is a XML-RPC server.
type Server struct {
	ctx       context.Context
	ctxCancel func()
	wg        sync.WaitGroup
	ln        net.Listener

	// in
	setHandler chan func(*RequestRaw) interface{}
}

// NewServer allocates a server.
func NewServer(port int) (*Server, error) {
	// net.Listen and http.Server are splitted since the latter
	// does not allow to use 0 as port
	ln, err := net.Listen("tcp", ":"+strconv.FormatInt(int64(port), 10))
	if err != nil {
		return nil, err
	}

	ctx, ctxCancel := context.WithCancel(context.Background())

	s := &Server{
		ctx:        ctx,
		ctxCancel:  ctxCancel,
		ln:         ln,
		setHandler: make(chan func(*RequestRaw) interface{}),
	}

	s.wg.Add(1)
	go s.run()

	return s, nil
}

// Close closes all the server resources.
func (s *Server) Close() error {
	s.ctxCancel()
	s.wg.Wait()
	return nil
}

// Port returns the server port.
func (s *Server) Port() int {
	return s.ln.Addr().(*net.TCPAddr).Port
}

func (s *Server) run() {
	defer s.wg.Done()

	var handler func(*RequestRaw) interface{}
	select {
	case handler = <-s.setHandler:
	case <-s.ctx.Done():
		s.ln.Close()
		return
	}

	hs := &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
			if req.URL.Path != "/RPC2" && req.URL.Path != "/" {
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

			res := handler(raw)

			if _, ok := res.(ErrorRes); ok {
				w.WriteHeader(http.StatusBadRequest)
			} else {
				responseEncode(w, res)
			}
		}),
	}

	s.wg.Add(1)
	go func() {
		defer s.wg.Done()
		hs.Serve(s.ln)
	}()

	<-s.ctx.Done()

	s.ln.Close()
	hs.Shutdown(context.Background())
}

// Serve starts serving requests and waits until the server is closed.
func (s *Server) Serve(handler func(*RequestRaw) interface{}) {
	s.setHandler <- handler
	s.wg.Wait()
}
