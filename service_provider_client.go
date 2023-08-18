package goroslib

import (
	"context"
	"net"
	"reflect"
	"time"

	"github.com/bluenviron/goroslib/v2/pkg/prototcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerID string
	nconn    net.Conn
	tconn    *prototcp.Conn

	ctx       context.Context
	ctxCancel func()
}

func newServiceProviderClient(
	sp *ServiceProvider,
	callerID string,
	nconn net.Conn,
	tconn *prototcp.Conn,
) *serviceProviderClient {
	ctx, ctxCancel := context.WithCancel(sp.ctx)

	spc := &serviceProviderClient{
		sp:        sp,
		callerID:  callerID,
		nconn:     nconn,
		tconn:     tconn,
		ctx:       ctx,
		ctxCancel: ctxCancel,
	}

	if sp.conf.onClient != nil {
		sp.conf.onClient()
	}

	sp.clientsWg.Add(1)
	go spc.run()

	return spc
}

func (spc *serviceProviderClient) run() {
	defer spc.sp.clientsWg.Done()

	readErr := make(chan error)
	go func() {
		readErr <- spc.runReader()
	}()

	select {
	case <-readErr:
		spc.nconn.Close()

	case <-spc.ctx.Done():
		spc.nconn.Close()
		<-readErr
	}

	spc.ctxCancel()

	select {
	case spc.sp.clientClose <- spc:
	case <-spc.sp.ctx.Done():
	}
}

func (spc *serviceProviderClient) runReader() error {
	spc.nconn.SetReadDeadline(time.Time{})

	for {
		req := reflect.New(reflect.TypeOf(spc.sp.srvReq)).Interface()
		err := spc.tconn.ReadMessage(req)
		if err != nil {
			return err
		}

		select {
		case spc.sp.clientRequest <- serviceProviderClientRequestReq{
			spc: spc,
			req: req,
		}:
		case <-spc.sp.ctx.Done():
			return ErrProviderTerminated
		}
	}
}
