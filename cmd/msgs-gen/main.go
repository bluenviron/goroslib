// main package.
package main

import (
	"fmt"
	"net/url"
	"os"
	"os/exec"
	"path/filepath"

	"github.com/go-git/go-git/v5"
	"github.com/go-git/go-git/v5/plumbing"

	"github.com/bluenviron/goroslib/v2/pkg/conversion"
)

func shellCommand(cmdstr string) error {
	fmt.Fprintf(os.Stderr, "%s\n", cmdstr)
	cmd := exec.Command("sh", "-c", cmdstr)
	cmd.Stderr = os.Stderr
	cmd.Stdout = os.Stdout
	return cmd.Run()
}

func processRepo(ur string, branch string) error {
	dir, err := os.MkdirTemp("", "goroslib")
	if err != nil {
		return err
	}
	defer os.RemoveAll(dir)

	u, _ := url.Parse(ur)
	dir = filepath.Join(dir, u.Path)

	os.Mkdir(dir, 0o755)

	_, err = git.PlainClone(dir, false, &git.CloneOptions{
		URL:           ur,
		Depth:         1,
		ReferenceName: plumbing.NewBranchReferenceName(branch),
	})
	if err != nil {
		return err
	}

	fmt.Fprintf(os.Stderr, "importing packages from %s, branch %s\n", ur, branch)
	return conversion.ImportPackageRecursive(dir)
}

func run() error {
	err := shellCommand("rm -rf pkg/msgs/*/")
	if err != nil {
		return err
	}

	os.Chdir(filepath.Join("pkg", "msgs"))

	done := make(chan error)
	count := 0

	for _, entry := range []struct {
		ur     string
		branch string
	}{
		{
			ur:     "https://github.com/ros/std_msgs",
			branch: "kinetic-devel",
		},
		{
			ur:     "https://github.com/ros/ros_comm_msgs",
			branch: "kinetic-devel",
		},
		{
			ur:     "https://github.com/ros/common_msgs",
			branch: "noetic-devel",
		},
		{
			ur:     "https://github.com/ros-drivers/ackermann_msgs",
			branch: "master",
		},
		{
			ur:     "https://github.com/ros-drivers/audio_common",
			branch: "master",
		},
		{
			ur:     "https://github.com/ros-drivers/velodyne",
			branch: "master",
		},
		{
			ur:     "https://github.com/ros-controls/control_msgs",
			branch: "kinetic-devel",
		},
		{
			ur:     "https://github.com/ros-perception/vision_msgs",
			branch: "noetic-devel",
		},
		{
			ur:     "https://github.com/ros/actionlib",
			branch: "noetic-devel",
		},
		{
			ur:     "https://github.com/mavlink/mavros",
			branch: "master",
		},
		{
			ur:     "https://github.com/ros-geographic-info/geographic_info",
			branch: "master",
		},
		{
			ur:     "https://github.com/ros-geographic-info/unique_identifier",
			branch: "master",
		},
		{
			ur:     "https://github.com/ros/geometry",
			branch: "noetic-devel",
		},
		{
			ur:     "https://github.com/ros/geometry2",
			branch: "noetic-devel",
		},
	} {
		count++
		go func(ur string, branch string) {
			done <- processRepo(ur, branch)
		}(entry.ur, entry.branch)
	}

	for i := 0; i < count; i++ {
		err := <-done
		if err != nil {
			return err
		}
	}

	return nil
}

func main() {
	err := run()
	if err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
