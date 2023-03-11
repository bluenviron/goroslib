package xmlrpc

import (
	"context"
	"net"
	"net/http"
	"net/url"
	"sync"
	"time"
)

// ErrorRes is a special response that sends status code 400.
// in case of errors, the C++ implementation replies with
// <methodResponse><fault><value>..., a structure which would require additional parsing.
type ErrorRes struct{}

// Server is a XML-RPC server.
type Server struct {
	ctx       context.Context
	ctxCancel func()
	wg        sync.WaitGroup
	ln        net.Listener
	hs        *http.Server
	handler   func(*RequestRaw) interface{}

	setHandler chan func(*RequestRaw) interface{}
	done       chan struct{}
}

// NewServer allocates a server.
func NewServer(address string, writeTimeout time.Duration) (*Server, error) {
	// net.Listen and http.Server are splitted since the latter
	// does not allow to use 0 as port
	ln, err := net.Listen("tcp", address)
	if err != nil {
		return nil, err
	}

	ctx, ctxCancel := context.WithCancel(context.Background())

	s := &Server{
		ctx:        ctx,
		ctxCancel:  ctxCancel,
		ln:         ln,
		setHandler: make(chan func(*RequestRaw) interface{}),
		done:       make(chan struct{}),
	}

	s.hs = &http.Server{
		Handler:      http.HandlerFunc(s.handleRequest),
		WriteTimeout: writeTimeout,
	}

	go s.run()

	return s, nil
}

// Close closes all the server resources.
func (s *Server) Close() error {
	s.ctxCancel()
	<-s.done
	return nil
}

// URL returns the server URL.
func (s *Server) URL(ip net.IP, zone string) string {
	return (&url.URL{
		Scheme: "http",
		Host: (&net.TCPAddr{
			IP:   ip,
			Port: s.ln.Addr().(*net.TCPAddr).Port,
			Zone: zone,
		}).String(),
	}).String()
}

func (s *Server) run() {
	defer close(s.done)

	select {
	case handler := <-s.setHandler:
		s.handler = handler

	case <-s.ctx.Done():
		s.ln.Close()
		return
	}

	go s.hs.Serve(s.ln)

	<-s.ctx.Done()

	s.hs.Shutdown(context.Background())
	s.ln.Close() // in case Shutdown() is called before Serve()
}

func (s *Server) handleRequest(w http.ResponseWriter, req *http.Request) {
	if req.URL.Path != "/RPC2" && req.URL.Path != "/" {
		w.WriteHeader(http.StatusNotFound)
		return
	}

	if req.Method != http.MethodPost {
		w.WriteHeader(http.StatusNotFound)
		return
	}

	raw, err := requestDecodeRaw(req.Body)
	if err != nil {
		w.WriteHeader(http.StatusBadRequest)
		return
	}

	res := s.handler(raw)

	if _, ok := res.(ErrorRes); ok {
		w.WriteHeader(http.StatusBadRequest)
		return
	}

	responseEncode(w, res)
}

// Serve starts serving requests and waits until the server is closed.
func (s *Server) Serve(handler func(*RequestRaw) interface{}) {
	select {
	case s.setHandler <- handler:
	case <-s.ctx.Done():
	}
	s.wg.Wait()
}
