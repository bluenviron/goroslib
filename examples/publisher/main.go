package main

import (
	"log"
	"os"
	"os/signal"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/sensor_msgs"
)

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_pub",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a publisher
	pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "test_topic",
		Msg:   &sensor_msgs.Imu{},
	})
	if err != nil {
		panic(err)
	}
	defer pub.Close()

	r := n.TimeRate(1 * time.Second)

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	for {
		select {
		// publish a message every second
		case <-r.SleepChan():
			msg := &sensor_msgs.Imu{
				AngularVelocity: geometry_msgs.Vector3{
					X: 23.5,
					Y: 22.1,
					Z: -7.5,
				},
			}
			log.Printf("Outgoing: %+v\n", msg)
			pub.Write(msg)

		// handle CTRL-C
		case <-c:
			return
		}
	}
}
