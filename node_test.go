package goroslib

import (
	"net"
	"os/exec"
	"regexp"
	"strconv"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type containerMaster struct {
	ip string
}

func newContainerMaster() (*containerMaster, error) {
	exec.Command("docker", "kill", "goroslib-test-master").Run()
	exec.Command("docker", "wait", "goroslib-test-master").Run()
	exec.Command("docker", "rm", "goroslib-test-master").Run()

	cmd := []string{"docker", "run", "--rm", "-d", "--name=goroslib-test-master"}
	cmd = append(cmd, "goroslib-test-master")
	err := exec.Command(cmd[0], cmd[1:]...).Run()
	if err != nil {
		return nil, err
	}

	// get master ip
	byts, _ := exec.Command("docker", "inspect", "-f",
		"{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}",
		"goroslib-test-master").Output()
	ip := string(byts[:len(byts)-1])

	// wait for master
	address := ip + ":" + strconv.FormatInt(11311, 10)
	for {
		time.Sleep(1 * time.Second)
		conn, err := net.DialTimeout("tcp", address, 5*time.Second)
		if err != nil {
			continue
		}
		conn.Close()
		break
	}

	return &containerMaster{
		ip: ip,
	}, nil
}

func (m *containerMaster) IP() string {
	return m.ip
}

func (m *containerMaster) close() {
	exec.Command("docker", "kill", "goroslib-test-master").Run()
	exec.Command("docker", "wait", "goroslib-test-master").Run()
}

type container struct {
	name string
}

func newContainer(name string, masterIP string) (*container, error) {
	exec.Command("docker", "kill", "goroslib-test-"+name).Run()
	exec.Command("docker", "wait", "goroslib-test-"+name).Run()
	exec.Command("docker", "rm", "goroslib-test-"+name).Run()

	cmd := []string{"docker", "run", "-d", "--name=goroslib-test-" + name}
	cmd = append(cmd, "-e", "MASTER_IP="+masterIP)
	cmd = append(cmd, "goroslib-test-"+name)
	err := exec.Command(cmd[0], cmd[1:]...).Run()
	if err != nil {
		return nil, err
	}

	// wait for node initialization
	time.Sleep(1 * time.Second)

	return &container{
		name: name,
	}, nil
}

func (c *container) close() {
	exec.Command("docker", "kill", "goroslib-test-"+c.name).Run()
	exec.Command("docker", "wait", "goroslib-test-"+c.name).Run()
	exec.Command("docker", "rm", "goroslib-test-"+c.name).Run()
}

func (c *container) waitOutput() string {
	exec.Command("docker", "wait", "goroslib-test-"+c.name).Run()
	out, _ := exec.Command("docker", "logs", "goroslib-test-"+c.name).Output()
	exec.Command("docker", "rm", "goroslib-test-"+c.name).Run()
	return string(out)
}

func TestNodeOpen(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()
}

func TestNodeNoMaster(t *testing.T) {
	_, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: "127.0.0.1:11311",
	})
	require.Error(t, err)
}

func TestNodeRosnodeInfo(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	// a publisher/subscriber is needed for the node to be listed
	// standard nodes creates some default publishers, subscribers and
	// services during initialization, while goroslib does not do this yet
	pub, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "test_topic",
		Msg:   &std_msgs.Float64{},
	})
	require.NoError(t, err)
	defer pub.Close()

	sub, err := NewSubscriber(SubscriberConf{
		Node:     n,
		Topic:    "test_topic",
		Callback: func(msg *sensor_msgs.Imu) {},
	})
	require.NoError(t, err)
	defer sub.Close()

	sp, err := NewServiceProvider(ServiceProviderConf{
		Node:    n,
		Service: "test_srv",
		Callback: func(req *TestServiceReq) *TestServiceRes {
			return &TestServiceRes{}
		},
	})
	require.NoError(t, err)
	defer sp.Close()

	rt, err := newContainer("rosnode-info", m.IP())
	require.NoError(t, err)

	require.Regexp(t, regexp.MustCompile("^--------------------------------------------------------------------------------\n"+
		"Node \\[/myns/goroslib\\]\n"+
		"Publications: \n"+
		" \\* /myns/test_topic \\[std_msgs/Float64\\]\n"+
		" \\* /rosout \\[rosgraph_msgs/Log\\]\n"+
		"\n"+
		"Subscriptions: \n"+
		" \\* /myns/test_topic \\[std_msgs/Float64\\]\n"+
		"\n"+
		"Services: \n"+
		" \\* /myns/test_srv\n"+
		"\n"+
		"\n"+
		"contacting node http://.+? ...\n"+
		"Pid: [0-9]+\n"+
		"\n$"), rt.waitOutput())
}
