// main package.
package main

import (
	"fmt"
	"os"

	"github.com/alecthomas/kong"

	"github.com/bluenviron/goroslib/v2/cmd"
)

var cli struct {
	GoPackage  string `name:"gopackage" help:"Go package name" default:"main"`
	RosPackage string `name:"rospackage" help:"ROS package name" default:"my_package"`
	URL        string `arg:"" help:"path or url pointing to a ROS message"`
}

func run() error {
	kong.Parse(&cli,
		kong.Description("Convert ROS messages into Go structs."),
		kong.UsageOnError())

	return cmd.ImportMsg(cli.URL, cli.GoPackage, cli.RosPackage, os.Stdout)
}

func main() {
	err := run()
	if err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
