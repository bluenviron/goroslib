package goroslib

import (
	"net"
	"os/exec"
	"strconv"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

type cntMaster struct {
	ip string
}

func newCntMaster() (*cntMaster, error) {
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

	return &cntMaster{
		ip: ip,
	}, nil
}

func (m *cntMaster) Ip() string {
	return m.ip
}

func (m *cntMaster) close() {
	exec.Command("docker", "kill", "goroslib-test-master").Run()
	exec.Command("docker", "wait", "goroslib-test-master").Run()
}

type cntNode struct {
	name string
}

func newCntNode(name string, masterIp string) (*cntNode, error) {
	exec.Command("docker", "kill", "goroslib-test-"+name).Run()
	exec.Command("docker", "wait", "goroslib-test-"+name).Run()
	exec.Command("docker", "rm", "goroslib-test-"+name).Run()

	cmd := []string{"docker", "run", "--rm", "-d", "--name=goroslib-test-" + name}
	cmd = append(cmd, "-e", "MASTER_IP="+masterIp)
	cmd = append(cmd, "goroslib-test-"+name)
	err := exec.Command(cmd[0], cmd[1:]...).Run()
	if err != nil {
		return nil, err
	}

	time.Sleep(1 * time.Second)

	return &cntNode{
		name: name,
	}, nil
}

func (n *cntNode) close() {
	exec.Command("docker", "kill", "goroslib-test-"+n.name).Run()
	exec.Command("docker", "wait", "goroslib-test-"+n.name).Run()
}

func newCntNodegen(masterIp string) (*cntNode, error) {
	return newCntNode("nodegen", masterIp)
}

func newCntNodepub(masterIp string) (*cntNode, error) {
	return newCntNode("nodepub", masterIp)
}

func newCntNodesub(masterIp string) (*cntNode, error) {
	return newCntNode("nodesub", masterIp)
}

func newCntNodesetparam(masterIp string) (*cntNode, error) {
	return newCntNode("nodesetparam", masterIp)
}

func newCntNodeserviceprovider(masterIp string) (*cntNode, error) {
	return newCntNode("nodeserviceprovider", masterIp)
}

func newCntNodeserviceclient(masterIp string) (*cntNode, error) {
	return newCntNode("nodeserviceclient", masterIp)
}

func TestNodeOpen(t *testing.T) {
	m, err := newCntMaster()
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
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodegen(m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 2, len(res))

	node, ok := res["/nodegen"]
	require.True(t, ok)

	_, ok = node.PublishedTopics["/rosout"]
	require.True(t, ok)

	_, ok = node.ProvidedServices["/nodegen/set_logger_level"]
	require.True(t, ok)

	_, ok = node.ProvidedServices["/nodegen/get_loggers"]
	require.True(t, ok)
}

func TestNodeGetMachines(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodegen(m.Ip())
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

	require.Equal(t, 2, len(res))
}

func TestNodeGetTopics(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodepub(m.Ip())
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
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodeserviceprovider(m.Ip())
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

func TestNodePingNode(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodegen(m.Ip())
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

func TestNodeKillNode(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodegen(m.Ip())
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

	require.Equal(t, 1, len(res))
}

func TestNodeGetParam(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodesetparam(m.Ip())
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
	m, err := newCntMaster()
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
