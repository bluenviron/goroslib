package goroslib

import (
	"os"
	"sync"

	"github.com/aler9/goroslib/pkg/apislave"
)

func (n *Node) runApiSlaveServer(wg *sync.WaitGroup) {
	defer wg.Done()

	n.apiSlaveServer.Handle(func(req apislave.Request) apislave.Response {
		switch reqt := req.(type) {
		case *apislave.RequestGetBusInfo:
			return apislave.ResponseGetBusInfo{
				Code:          1,
				StatusMessage: "bus info",
				// TODO: provide bus infos in this format:
				// connectionId, destinationId, direction (i, o, b), transport, topic,connected
				// {"1", "/rosout", "o", "tcpros", "/rosout", "1"}
				// [ 1, /rosout, o, tcpros, /rosout, 1, TCPROS connection on port 46477 to [127.0.0.1:51790 on socket 8] ]
				BusInfo: [][]string{},
			}

		case *apislave.RequestGetPid:
			return apislave.ResponseGetPid{
				Code:          1,
				StatusMessage: "",
				Pid:           os.Getpid(),
			}

		case *apislave.RequestGetPublications:
			resChan := make(chan [][]string)
			n.getPublications <- getPublicationsReq{resChan}
			res := <-resChan

			return apislave.ResponseGetPublications{
				Code:          1,
				StatusMessage: "",
				TopicList:     res,
			}

		case *apislave.RequestPublisherUpdate:
			n.publisherUpdate <- publisherUpdateReq{
				topic: reqt.Topic,
				urls:  reqt.PublisherUrls,
			}

			return apislave.ResponsePublisherUpdate{
				Code:          1,
				StatusMessage: "",
			}

		case *apislave.RequestRequestTopic:
			resChan := make(chan apislave.ResponseRequestTopic)
			n.subscriberRequestTopic <- subscriberRequestTopicReq{reqt, resChan}
			res := <-resChan
			return res

		case *apislave.RequestShutdown:
			n.shutdown <- struct{}{}

			return apislave.ResponseShutdown{
				Code:          1,
				StatusMessage: "",
			}
		}

		return apislave.ErrorRes{}
	})
}
