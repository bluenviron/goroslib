// main package.
package main

import (
	"fmt"
	"os"

	"github.com/alecthomas/kong"

	"github.com/bluenviron/goroslib/v2/pkg/conversion"
)

var cli struct {
	Path string `arg:"" help:"path pointing to a ROS package"`
}

func run() error {
	kong.Parse(&cli,
		kong.Description("Convert all messages, services and actions definions in a ROS package into Go structs."),
		kong.UsageOnError())

	return conversion.ImportPackageRecursive(cli.Path)
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintf(os.Stderr, "ERR: %s\n", err)
		os.Exit(1)
	}
}
