package goroslib

import (
	"context"
	"fmt"
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

func newServiceProviderClient(sp *ServiceProvider, callerID string, conn *prototcp.Conn) {
	ctx, ctxCancel := context.WithCancel(sp.ctx)

	spc := &serviceProviderClient{
		sp:        sp,
		callerID:  callerID,
		conn:      conn,
		ctx:       ctx,
		ctxCancel: ctxCancel,
	}

	sp.clients[callerID] = spc

	sp.clientsWg.Add(1)
	go spc.run()
}

func (spc *serviceProviderClient) close() {
	delete(spc.sp.clients, spc.callerID)
	spc.ctxCancel()
}

func (spc *serviceProviderClient) run() {
	defer spc.sp.clientsWg.Done()

	readErr := make(chan error)
	go func() {
		readErr <- func() error {
			for {
				req := reflect.New(reflect.TypeOf(spc.sp.srvReq)).Interface()
				err := spc.conn.ReadMessage(req)
				if err != nil {
					return err
				}

				select {
				case spc.sp.clientRequest <- serviceProviderClientRequestReq{
					callerID: spc.callerID,
					req:      req,
				}:
				case <-spc.sp.ctx.Done():
					return fmt.Errorf("terminated")
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
