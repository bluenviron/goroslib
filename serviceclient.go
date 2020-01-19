package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/tcpros"
)

type ServiceClientConf struct {
	Node    *Node
	Service string
	Req     interface{}
	Res     interface{}
}

type ServiceClient struct {
	conf ServiceClientConf
	conn *tcpros.Conn
}

func NewServiceClient(conf ServiceClientConf) (*ServiceClient, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if len(conf.Service) < 1 || conf.Service[0] != '/' {
		return nil, fmt.Errorf("Service must begin with /")
	}

	rt := reflect.TypeOf(conf.Req)
	if rt.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Req must be a pointer")
	}
	if rt.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Req must be a pointer to a struct")
	}

	rt = reflect.TypeOf(conf.Res)
	if rt.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Res must be a pointer")
	}
	if rt.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Res must be a pointer to a struct")
	}

	ur, err := conf.Node.masterClient.LookupService(conf.Service)
	if err != nil {
		return nil, fmt.Errorf("lookupService: %v", err)
	}

	srvMd5, err := msg.ServiceMd5(conf.Req, conf.Res)
	if err != nil {
		return nil, err
	}

	host, port, err := parseUrl(ur)
	if err != nil {
		return nil, err
	}

	conn, err := tcpros.NewClient(host, port)
	if err != nil {
		return nil, err
	}

	err = conn.WriteHeader(&tcpros.HeaderServiceClient{
		Callerid:   conf.Node.conf.Name,
		Md5sum:     srvMd5,
		Persistent: 1,
		Service:    conf.Service,
	})
	if err != nil {
		conn.Close()
		return nil, err
	}

	var outHeader tcpros.HeaderServiceProvider
	err = conn.ReadHeader(&outHeader)
	if err != nil {
		conn.Close()
		return nil, err
	}

	if outHeader.Error != "" {
		conn.Close()
		return nil, fmt.Errorf(outHeader.Error)
	}

	if outHeader.Md5sum != srvMd5 {
		conn.Close()
		return nil, fmt.Errorf("missing or wrong md5sum: expected %s, got %s",
			srvMd5, outHeader.Md5sum)
	}

	return &ServiceClient{
		conf: conf,
		conn: conn,
	}, nil
}

func (sc *ServiceClient) Close() error {
	sc.conn.Close()
	return nil
}

func (sc *ServiceClient) Call(req interface{}, res interface{}) error {
	if reflect.TypeOf(req) != reflect.TypeOf(sc.conf.Req) {
		panic("wrong req")
	}
	if reflect.TypeOf(res) != reflect.TypeOf(sc.conf.Res) {
		panic("wrong res")
	}

	err := sc.conn.WriteMessage(req)
	if err != nil {
		return err
	}

	err = sc.conn.ReadServiceResState()
	if err != nil {
		return err
	}

	err = sc.conn.ReadMessage(res)
	if err != nil {
		return err
	}

	return nil
}
