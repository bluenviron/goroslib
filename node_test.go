package goroslib

import (
	"net"
	"net/http"
	"os"
	"os/exec"
	"regexp"
	"sort"
	"strconv"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/apislave"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

type containerMaster struct {
	ip string
}

func newContainerMaster(t *testing.T) *containerMaster {
	exec.Command("docker", "kill", "goroslib-test-master").Run()
	exec.Command("docker", "wait", "goroslib-test-master").Run()
	exec.Command("docker", "rm", "goroslib-test-master").Run()

	cmd := []string{"docker", "run", "--rm", "-d", "--name=goroslib-test-master"}
	cmd = append(cmd, "goroslib-test-master")
	err := exec.Command(cmd[0], cmd[1:]...).Run()
	require.NoError(t, err)

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
	}
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

func newContainer(t *testing.T, name string, masterIP string) *container {
	exec.Command("docker", "kill", "goroslib-test-"+name).Run()
	exec.Command("docker", "wait", "goroslib-test-"+name).Run()
	exec.Command("docker", "rm", "goroslib-test-"+name).Run()

	cmd := []string{"docker", "run", "-d", "--name=goroslib-test-" + name}
	cmd = append(cmd, "-e", "MASTER_IP="+masterIP)
	cmd = append(cmd, "goroslib-test-"+name)
	err := exec.Command(cmd[0], cmd[1:]...).Run()
	require.NoError(t, err)

	// wait for node initialization
	time.Sleep(1 * time.Second)

	return &container{
		name: name,
	}
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
	t.Run("normal", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib1",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()
	})

	t.Run("http", func(t *testing.T) {
		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Namespace:     "/myns",
			Name:          "goroslib2",
			MasterAddress: "http://" + m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()
	})
}

func TestNodeOpenErrors(t *testing.T) {
	_, err := NewNode(NodeConf{
		Namespace: "/myns",
		Name:      "goroslib",
	})
	require.Error(t, err)

	_, err = NewNode(NodeConf{
		Namespace: "myns",
		Name:      "goroslib",
	})
	require.Error(t, err)

	_, err = NewNode(NodeConf{
		Namespace: "myns/",
		Name:      "goroslib",
	})
	require.Error(t, err)

	_, err = NewNode(NodeConf{
		Namespace: "/myns",
		Name:      "",
	})
	require.Error(t, err)

	_, err = NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: "unresolved",
	})
	require.EqualError(t, err, "unable to solve master address: address unresolved: missing port in address")

	_, err = NewNode(NodeConf{
		Namespace: "/myns",
		Name:      "goroslib",
		MasterAddress: (&net.TCPAddr{
			IP:   net.ParseIP("::1"),
			Zone: "lo",
			Port: 123,
		}).String(),
	})
	require.EqualError(t, err, "stateless IPv6 master addresses are not supported")

	_, err = NewNode(NodeConf{
		Namespace: "/myns",
		Name:      "goroslib",
		Host:      "nonexistent",
	})
	require.Error(t, err)
}

func TestNodeNamespaceFromEnv(t *testing.T) {
	t.Run("from environment", func(t *testing.T) {
		os.Setenv("ROS_NAMESPACE", "/myns")
		defer os.Unsetenv("ROS_NAMESPACE")

		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:          "goroslib1",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		require.Equal(t, "/myns", n.conf.Namespace)
	})

	t.Run("from environment and conf", func(t *testing.T) {
		os.Setenv("ROS_NAMESPACE", "/myns1")
		defer os.Unsetenv("ROS_NAMESPACE")

		m := newContainerMaster(t)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:          "goroslib1",
			Namespace:     "/myns2",
			MasterAddress: m.IP() + ":11311",
		})
		require.NoError(t, err)
		defer n.Close()

		require.Equal(t, "/myns1", n.conf.Namespace)
	})
}

func TestNodeCliRemapping(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
		args:          []string{"goroslib:=replaced"},
	})
	require.NoError(t, err)
	defer n.Close()

	require.Equal(t, "replaced", n.conf.Name)
}

func TestNodeRosnodeInfo(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "test_pub",
		Msg:   &std_msgs.Float64{},
	})
	require.NoError(t, err)
	defer pub.Close()

	sub, err := NewSubscriber(SubscriberConf{
		Node:     n,
		Topic:    "test_sub",
		Callback: func(msg *std_msgs.Int32) {},
	})
	require.NoError(t, err)
	defer sub.Close()

	sp, err := NewServiceProvider(ServiceProviderConf{
		Node: n,
		Name: "test_srv",
		Srv:  &TestService{},
		Callback: func(req *TestServiceReq) (*TestServiceRes, bool) {
			return &TestServiceRes{}, true
		},
	})
	require.NoError(t, err)
	defer sp.Close()

	rt := newContainer(t, "rosnode-info", m.IP())

	require.Regexp(t, regexp.MustCompile(
		"^--------------------------------------------------------------------------------\n"+
			"Node \\[/myns/goroslib\\]\n"+
			"Publications: \n"+
			" \\* /myns/test_pub \\[std_msgs/Float64\\]\n"+
			" \\* /rosout \\[rosgraph_msgs/Log\\]\n"+
			"\n"+
			"Subscriptions: \n"+
			" \\* /myns/test_sub \\[unknown type\\]\n"+
			"\n"+
			"Services: \n"+
			" \\* /myns/test_srv\n"+
			"\n"+
			"\n"+
			"contacting node http://.+? ...\n"+
			"Pid: [0-9]+\n"+
			"Connections:\n"+
			" \\* topic: /rosout\n"+
			"    \\* to: /rosout\n"+
			"    \\* direction: outbound\n"+
			"    \\* transport: TCPROS\n"+
			"\n$"), rt.waitOutput())
}

func TestNodeGetPublications(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
		ApislavePort:  9906,
	})
	require.NoError(t, err)
	defer n.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n,
		Topic: "test_pub",
		Msg:   &std_msgs.Float64{},
	})
	require.NoError(t, err)
	defer pub.Close()

	c := apislave.NewClient(n.nodeAddr.IP.String()+":9906", "mycallerid", &http.Client{})

	res, err := c.GetPublications()
	require.NoError(t, err)
	sort.Slice(res, func(a, b int) bool {
		return res[a][0] < res[b][0]
	})
	require.Equal(t, [][]string{
		{"/myns/test_pub", "std_msgs/Float64"},
		{"/rosout", "rosgraph_msgs/Log"},
	}, res)
}
