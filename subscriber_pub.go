package goroslib

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"net"
	"net/url"
	"reflect"
	"strconv"
	"time"

	"github.com/bluenviron/goroslib/v2/pkg/apislave"
	"github.com/bluenviron/goroslib/v2/pkg/protocommon"
	"github.com/bluenviron/goroslib/v2/pkg/prototcp"
	"github.com/bluenviron/goroslib/v2/pkg/protoudp"
)

const (
	subscriberPubRestartPause = 5 * time.Second
)

type localIPs map[string]struct{}

func loadLocalIPs() localIPs {
	ret := make(localIPs)

	ifaces, err := net.Interfaces()
	if err == nil {
		for _, i := range ifaces {
			addrs, err := i.Addrs()
			if err != nil {
				continue
			}

			for _, addr := range addrs {
				if v, ok := addr.(*net.IPNet); ok {
					ret[v.IP.String()] = struct{}{}
				}
			}
		}
	}

	return ret
}

func (l localIPs) contains(ip net.IP) bool {
	_, ok := l[ip.String()]
	return ok
}

type subscriberPublisher struct {
	sub     *Subscriber
	address string

	ctx            context.Context
	ctxCancel      func()
	localIPs       localIPs
	udpAddrIsLocal bool
	udpAddr        *net.UDPAddr
	udpID          uint32

	// in
	udpFrame chan *protoudp.Frame
}

func newSubscriberPublisher(
	sub *Subscriber,
	address string,
) *subscriberPublisher {
	ctx, ctxCancel := context.WithCancel(sub.ctx)

	sp := &subscriberPublisher{
		sub:       sub,
		address:   address,
		ctx:       ctx,
		ctxCancel: ctxCancel,
	}

	sub.publishersWg.Add(1)
	go sp.run()

	return sp
}

func (sp *subscriberPublisher) close() {
	sp.ctxCancel()
}

func (sp *subscriberPublisher) run() {
	defer sp.sub.publishersWg.Done()

	sp.sub.conf.Node.Log(LogLevelDebug, "subscriber '%s' got a new publisher '%s'",
		sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
		sp.address)

outer:
	for {
		err := sp.runInner()
		if err == nil { // terminated
			break outer
		}

		if errors.Is(err, io.EOF) {
			sp.sub.conf.Node.Log(LogLevelError,
				"subscriber '%s' got an error: %s",
				sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
				err.Error())
		}

		select {
		case <-time.After(subscriberPubRestartPause):
		case <-sp.ctx.Done():
			break outer
		}
	}

	sp.ctxCancel()

	sp.sub.conf.Node.Log(LogLevelDebug, "subscriber '%s' doesn't have publisher '%s' anymore",
		sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
		sp.address)
}

func (sp *subscriberPublisher) runInner() error {
	sp.sub.conf.Node.Log(LogLevelDebug, "subscriber '%s' is connecting to publisher '%s'",
		sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
		sp.address)

	xcs := apislave.NewClient(sp.address, sp.sub.conf.Node.absoluteName(), sp.sub.conf.Node.httpClient)

	subDone := make(chan struct{})
	var proto []interface{}
	var err error

	go func() {
		defer close(subDone)

		proto, err = func() ([]interface{}, error) {
			var requestedProtos [][]interface{}
			if sp.sub.conf.Protocol == TCP {
				requestedProtos = [][]interface{}{{"TCPROS"}}
			} else {
				var buf bytes.Buffer
				err2 := protocommon.HeaderEncode(&buf, &protoudp.HeaderSubscriber{
					Callerid:          sp.sub.conf.Node.absoluteName(),
					Md5sum:            sp.sub.msgMd5,
					Topic:             sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
					Type:              sp.sub.msgType,
					MessageDefinition: sp.sub.msgDef,
				})
				if err2 != nil {
					return nil, err2
				}

				udpHeader := buf.Bytes()[4:]

				requestedProtos = [][]interface{}{{
					"UDPROS",
					udpHeader,
					sp.sub.conf.Node.nodeAddr.IP.String(),
					sp.sub.conf.Node.udprosListener.LocalAddr().(*net.UDPAddr).Port,
					udpMTU,
				}}
			}

			return xcs.RequestTopic(sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic), requestedProtos)
		}()
	}()

	select {
	case <-subDone:
	case <-sp.ctx.Done():
		<-subDone
		return nil
	}

	if err != nil {
		return err
	}

	if sp.sub.conf.Protocol == TCP {
		return sp.runInnerTCP(proto)
	}
	return sp.runInnerUDP(proto)
}

func (sp *subscriberPublisher) runInnerTCP(proto []interface{}) error {
	if len(proto) != 3 {
		return fmt.Errorf("wrong protocol length")
	}

	protoName, ok := proto[0].(string)
	if !ok {
		return fmt.Errorf("wrong protoName")
	}

	protoHost, ok := proto[1].(string)
	if !ok {
		return fmt.Errorf("wrong protoHost")
	}

	protoPort, ok := proto[2].(int)
	if !ok {
		return fmt.Errorf("wrong protoPort")
	}

	if protoName != "TCPROS" {
		return fmt.Errorf("wrong protoName")
	}

	address := net.JoinHostPort(protoHost, strconv.FormatInt(int64(protoPort), 10))

	ctx2, ctx2Cancel := context.WithTimeout(sp.ctx, sp.sub.conf.Node.conf.ReadTimeout)
	defer ctx2Cancel()

	nconn, err := (&net.Dialer{}).DialContext(ctx2, "tcp", address)
	if err != nil {
		return err
	}
	defer nconn.Close()

	tconn := prototcp.NewConn(nconn)

	if sp.sub.conf.EnableKeepAlive {
		nconn.(*net.TCPConn).SetKeepAlive(true)
		nconn.(*net.TCPConn).SetKeepAlivePeriod(60 * time.Second)
	}

	if sp.sub.conf.DisableNoDelay {
		err = nconn.(*net.TCPConn).SetNoDelay(false)
		if err != nil {
			return err
		}
	}

	subDone := make(chan struct{})
	var outHeader prototcp.HeaderPublisher

	go func() {
		defer close(subDone)

		err = tconn.WriteHeader(&prototcp.HeaderSubscriber{
			Callerid: sp.sub.conf.Node.absoluteName(),
			Md5sum:   sp.sub.msgMd5,
			Topic:    sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
			Type:     sp.sub.msgType,
			TcpNodelay: func() int {
				if sp.sub.conf.DisableNoDelay {
					return 0
				}
				return 1
			}(),
			MessageDefinition: sp.sub.msgDef,
		})
		if err != nil {
			return
		}

		nconn.SetReadDeadline(time.Now().Add(sp.sub.conf.Node.conf.ReadTimeout))
		var raw protocommon.HeaderRaw
		raw, err = tconn.ReadHeaderRaw()
		if err != nil {
			return
		}

		if strErr, ok := raw["error"]; ok {
			err = fmt.Errorf(strErr)
			return
		}

		err = protocommon.HeaderDecode(raw, &outHeader)
	}()

	select {
	case <-subDone:
	case <-sp.ctx.Done():
		nconn.Close()
		<-subDone
		return nil
	}

	if err != nil {
		return err
	}

	if outHeader.Topic != "" &&
		outHeader.Topic != sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic) {
		return fmt.Errorf("wrong topic (expected '%s', got '%s')",
			sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
			outHeader.Topic)
	}

	if outHeader.Md5sum != sp.sub.msgMd5 {
		return fmt.Errorf("wrong message checksum")
	}

	sp.sub.conf.Node.Log(LogLevelDebug, "subscriber '%s' is reading from publisher '%s' with TCP",
		sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
		sp.address)

	if sp.sub.conf.onPublisher != nil {
		sp.sub.conf.onPublisher()
	}

	subDone = make(chan struct{})
	go func() {
		defer close(subDone)

		nconn.SetReadDeadline(time.Time{})

		for {
			msg := reflect.New(sp.sub.msgMsg).Interface()
			err = tconn.ReadMessage(msg)
			if err != nil {
				return
			}

			if sp.sub.conf.QueueSize == 0 {
				select {
				case sp.sub.message <- msg:
				case <-sp.sub.ctx.Done():
				}
			} else {
				select {
				case sp.sub.message <- msg:
				default:
				}
			}
		}
	}()

	select {
	case <-subDone:
		return err

	case <-sp.ctx.Done():
		nconn.Close()
		<-subDone
		return nil
	}
}

func (sp *subscriberPublisher) runInnerUDP(proto []interface{}) error {
	if len(proto) != 6 {
		return fmt.Errorf("wrong protocol length")
	}

	protoName, ok := proto[0].(string)
	if !ok {
		return fmt.Errorf("wrong protoName")
	}

	protoHost, ok := proto[1].(string)
	if !ok {
		return fmt.Errorf("wrong protoHost")
	}

	protoPort, ok := proto[2].(int)
	if !ok {
		return fmt.Errorf("wrong protoPort")
	}

	protoID, ok := proto[3].(int)
	if !ok {
		return fmt.Errorf("wrong protoID")
	}

	if protoName != "UDPROS" {
		return fmt.Errorf("wrong protoName")
	}

	// solve remote host and port
	addr, err := net.ResolveUDPAddr("udp", net.JoinHostPort(protoHost, strconv.FormatInt(int64(protoPort), 10)))
	if err != nil {
		return fmt.Errorf("unable to solve host")
	}

	sp.localIPs = loadLocalIPs()

	if sp.localIPs.contains(addr.IP) {
		sp.udpAddrIsLocal = true
	}

	sp.udpAddr = addr
	sp.udpID = uint32(protoID)
	sp.udpFrame = make(chan *protoudp.Frame)

	select {
	case sp.sub.conf.Node.udpSubPublisherNew <- sp:
	case <-sp.sub.conf.Node.ctx.Done():
	}

	defer func() {
		done := make(chan struct{})
		select {
		case sp.sub.conf.Node.udpSubPublisherClose <- udpSubPublisherCloseReq{sp, done}:
			<-done
		case <-sp.sub.conf.Node.ctx.Done():
		}
		close(sp.udpFrame)
	}()

	sp.sub.conf.Node.Log(LogLevelDebug, "subscriber '%s' is reading from publisher '%s' with UDP",
		sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
		sp.address)

	if sp.sub.conf.onPublisher != nil {
		sp.sub.conf.onPublisher()
	}

	readerClose := make(chan struct{})
	readerDone := make(chan struct{})
	if sp.sub.conf.EnableKeepAlive {
		go sp.runUDPPing(readerClose, readerDone)
	}

	var curMsg []byte
	curFieldID := 0
	curFieldCount := 0

	for {
		select {
		case frame := <-sp.udpFrame:
			switch frame.Opcode {
			case protoudp.Data0:
				curMsg = append([]byte{}, frame.Payload...)
				curFieldID = 0
				curFieldCount = int(frame.BlockID)

			case protoudp.DataN:
				if int(frame.BlockID) != (curFieldID + 1) {
					continue
				}
				curMsg = append(curMsg, frame.Payload...)
				curFieldID++
			}

			if (curFieldID + 1) == curFieldCount {
				msg := reflect.New(sp.sub.msgMsg).Interface()
				err := protocommon.MessageDecode(bytes.NewReader(curMsg), msg)
				if err != nil {
					continue
				}

				if sp.sub.conf.QueueSize == 0 {
					sp.sub.message <- msg
				} else {
					select {
					case sp.sub.message <- msg:
					default:
					}
				}
			}

		case <-sp.ctx.Done():
			if sp.sub.conf.EnableKeepAlive {
				close(readerClose)
				<-readerDone
			}

			return nil
		}
	}
}

func (sp *subscriberPublisher) runUDPPing(readerClose chan struct{}, readerDone chan struct{}) {
	defer close(readerDone)

	curMessageID := uint8(0)

	t := time.NewTicker(60 * time.Second)
	defer t.Stop()

	for {
		select {
		case <-t.C:
			sp.sub.conf.Node.udprosConn.WriteFrame(&protoudp.Frame{ //nolint:errcheck
				ConnectionID: sp.udpID,
				Opcode:       protoudp.Ping,
				MessageID:    curMessageID,
			}, sp.udpAddr)
			curMessageID++

		case <-readerClose:
			return
		}
	}
}

func (sp *subscriberPublisher) busInfo() []interface{} {
	proto := func() string {
		if sp.sub.conf.Protocol == UDP {
			return "UDPROS"
		}
		return "TCPROS"
	}()

	ur := (&url.URL{
		Scheme: "http",
		Host:   sp.address,
		Path:   "/",
	}).String()

	return []interface{}{
		0,
		ur,
		"i",
		proto,
		sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
		true,
	}
}
