package goroslib

import (
	"context"
	"reflect"

	"github.com/aler9/goroslib/pkg/prototcp"
)

type serviceProviderClient struct {
	sp       *ServiceProvider
	callerID string
	conn     *prototcp.Conn

	ctx       context.Context
	ctxCancel func()
}

func newServiceProviderClient(
	sp *ServiceProvider,
	callerID string,
	conn *prototcp.Conn,
) *serviceProviderClient {
	ctx, ctxCancel := context.WithCancel(sp.ctx)

	spc := &serviceProviderClient{
		sp:        sp,
		callerID:  callerID,
		conn:      conn,
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
		readErr <- func() error {
			for {
				req := reflect.New(reflect.TypeOf(spc.sp.srvReq)).Interface()
				err := spc.conn.ReadMessage(req, false)
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
		}()
	}()

	select {
	case <-readErr:
		spc.conn.Close()

	case <-spc.ctx.Done():
		spc.conn.Close()
		<-readErr
	}

	spc.ctxCancel()

	select {
	case spc.sp.clientClose <- spc:
	case <-spc.sp.ctx.Done():
	}
}
