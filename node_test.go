package goroslib

import (
	"net"
	"os/exec"
	"regexp"
	"strconv"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msgs/sensor_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
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
		conn, err := net.DialTimeout("tcp4", address, 5*time.Second)
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

func (m *containerMaster) Ip() string {
	return m.ip
}

func (m *containerMaster) close() {
	exec.Command("docker", "kill", "goroslib-test-master").Run()
	exec.Command("docker", "wait", "goroslib-test-master").Run()
}

type container struct {
	name string
}

func newContainer(name string, masterIp string) (*container, error) {
	exec.Command("docker", "kill", "goroslib-test-"+name).Run()
	exec.Command("docker", "wait", "goroslib-test-"+name).Run()
	exec.Command("docker", "rm", "goroslib-test-"+name).Run()

	cmd := []string{"docker", "run", "-d", "--name=goroslib-test-" + name}
	cmd = append(cmd, "-e", "MASTER_IP="+masterIp)
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
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()
}

func TestNodeGetNodes(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n1, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n1.Close()

	n2, err := NewNode(NodeConf{
		Name:       "/goroslib2",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)

	res, err := n1.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 4, len(res))

	node, ok := res["/nodegen"]
	require.True(t, ok)

	_, ok = node.PublishedTopics["/rosout"]
	require.True(t, ok)

	_, ok = node.ProvidedServices["/nodegen/set_logger_level"]
	require.True(t, ok)

	_, ok = node.ProvidedServices["/nodegen/get_loggers"]
	require.True(t, ok)

	n2.Close()

	res, err = n1.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 3, len(res))
}

func TestNodeGetMachines(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetMachines()
	require.NoError(t, err)

	require.Equal(t, 3, len(res))
}

func TestNodeGetTopics(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-pub", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetTopics()
	require.NoError(t, err)

	topic, ok := res["/test_pub"]
	require.True(t, ok)

	require.Equal(t, "nodepub/Mymsg", topic.Type)

	require.Equal(t, map[string]struct{}{"/nodepub": {}}, topic.Publishers)
}

func TestNodeGetServices(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-serviceprovider", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetServices()
	require.NoError(t, err)

	service, ok := res["/test_srv"]
	require.True(t, ok)

	require.Equal(t, map[string]struct{}{"/nodeserviceprovider": {}}, service.Providers)
}

func TestNodePingCppNode(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	_, err = n.PingNode("/nodegen")
	require.NoError(t, err)
}

func TestNodePingGoNode(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n1, err := NewNode(NodeConf{
		Name:       "/goroslib1",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n1.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n1,
		Topic: "/test_pub",
		Msg:   &TestMessage{},
	})
	require.NoError(t, err)
	defer pub.Close()

	n2, err := NewNode(NodeConf{
		Name:       "/goroslib2",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n2.Close()

	_, err = n2.PingNode("/goroslib1")
	require.NoError(t, err)
}

func TestNodeKillCppNode(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	err = n.KillNode("/nodegen")
	require.NoError(t, err)

	time.Sleep(1 * time.Second)

	res, err := n.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 2, len(res))
}

func TestNodeKillGoNode(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n1, err := NewNode(NodeConf{
		Name:       "/goroslib1",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n1.Close()

	pub, err := NewPublisher(PublisherConf{
		Node:  n1,
		Topic: "/test_pub",
		Msg:   &TestMessage{},
	})
	require.NoError(t, err)
	defer pub.Close()

	n2, err := NewNode(NodeConf{
		Name:       "/goroslib2",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n2.Close()

	err = n2.KillNode("/goroslib1")
	require.NoError(t, err)

	time.Sleep(1 * time.Second)

	res, err := n2.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 2, len(res))
}

func TestNodeGetParam(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-setparam", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	t.Run("bool", func(t *testing.T) {
		res, err := n.GetParamBool("/test_bool")
		require.NoError(t, err)
		require.Equal(t, true, res)
	})

	t.Run("int", func(t *testing.T) {
		res, err := n.GetParamInt("/test_int")
		require.NoError(t, err)
		require.Equal(t, 123, res)
	})

	t.Run("string", func(t *testing.T) {
		res, err := n.GetParamString("/test_string")
		require.NoError(t, err)
		require.Equal(t, "ABC", res)
	})
}

func TestNodeSetParam(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	t.Run("bool", func(t *testing.T) {
		err = n.SetParamBool("/test_bool", true)
		require.NoError(t, err)

		res, err := n.GetParamBool("/test_bool")
		require.NoError(t, err)
		require.Equal(t, true, res)
	})

	t.Run("int", func(t *testing.T) {
		err = n.SetParamInt("/test_int", 123)
		require.NoError(t, err)

		res, err := n.GetParamInt("/test_int")
		require.NoError(t, err)
		require.Equal(t, 123, res)
	})

	t.Run("string", func(t *testing.T) {
		err = n.SetParamString("/test_string", "ABC")
		require.NoError(t, err)

		res, err := n.GetParamString("/test_string")
		require.NoError(t, err)
		require.Equal(t, "ABC", res)
	})
}

func TestNodeRosnodeInfo(t *testing.T) {
	recv := func() string {
		m, err := newContainerMaster()
		require.NoError(t, err)
		defer m.close()

		n, err := NewNode(NodeConf{
			Name:       "/goroslib",
			MasterHost: m.Ip(),
		})
		require.NoError(t, err)
		defer n.Close()

		// a publisher/subscriber is needed for the node to be listed
		// standard nodes creates some default publishers, subscribers and
		// services during initialization, while goroslib does not do this yet
		pub, err := NewPublisher(PublisherConf{
			Node:  n,
			Topic: "/test_pub",
			Msg:   &std_msgs.Float64{},
		})
		require.NoError(t, err)
		defer pub.Close()

		sub, err := NewSubscriber(SubscriberConf{
			Node:     n,
			Topic:    "/test_pub",
			Callback: func(msg *sensor_msgs.Imu) {},
		})
		require.NoError(t, err)
		defer sub.Close()

		rt, err := newContainer("rosnode-info", m.Ip())
		require.NoError(t, err)

		return rt.waitOutput()
	}()

	require.Regexp(t, regexp.MustCompile("^--------------------------------------------------------------------------------\n"+
		"Node \\[/goroslib\\]\n"+
		"Publications: \n"+
		"* \\* /rosout \\[rosgraph_msgs/Log\\]\n"+
		" \\* /test_pub \\[std_msgs/Float64\\]\n"+
		"\n"+
		"Subscriptions: \n"+
		" \\* /test_pub \\[std_msgs/Float64\\]\n"+
		"\n"+
		"Services: None\n"+
		"\n"+
		"\n"+
		"contacting node http://.+? ...\n"+
		"Pid: .+?\n"+
		"\n$"), recv)
}
