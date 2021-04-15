package msgconv

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestMessageDefinition(t *testing.T) {
	for _, ca := range []struct {
		name   string
		ros    string
		golang string
	}{
		{
			"native types",
			"bool a\n" +
				"int8 b\n" +
				"uint8 c\n" +
				"int16 d\n" +
				"uint16 e\n" +
				"int32 f\n" +
				"uint32 g\n" +
				"int64 h\n" +
				"uint64 i\n" +
				"float32 j\n" +
				"float64 k\n" +
				"string l\n" +
				"time m\n" +
				"duration n\n" +
				"byte o\n" +
				"char p\n",
			"\n\ntype msgname struct {\n" +
				"    msg.Package `ros:\"rospkg\"`\n" +
				"    A bool\n" +
				"    B int8\n" +
				"    C uint8\n" +
				"    D int16\n" +
				"    E uint16\n" +
				"    F int32\n" +
				"    G uint32\n" +
				"    H int64\n" +
				"    I uint64\n" +
				"    J float32\n" +
				"    K float64\n" +
				"    L string\n" +
				"    M time.Time\n" +
				"    N time.Duration\n" +
				"    O int8 `rostype:\"byte\"`\n" +
				"    P uint8 `rostype:\"char\"`\n" +
				"}\n",
		},
		{
			"type with spaces",
			"bool     a  \n",
			"\n\ntype msgname struct {\n" +
				"    msg.Package `ros:\"rospkg\"`\n" +
				"    A bool\n" +
				"}\n",
		},
		{
			"definitions",
			"int32 VAL1=3\n" +
				"int32 VAL2=4\n" +
				"\n" +
				"int32 var\n",
			"\n\nconst (\n" +
				"    msgname_VAL1 int32 = 3\n" +
				"    msgname_VAL2 int32 = 4\n" +
				")\n" +
				"\n" +
				"type msgname struct {\n" +
				"    msg.Package `ros:\"rospkg\"`\n" +
				"    msg.Definitions `ros:\"int32 VAL1=3,int32 VAL2=4\"`\n" +
				"    Var int32\n" +
				"}\n",
		},
		{
			"definition with spaces",
			"int32 VAL1  =    3\n" +
				"\n" +
				"int32 var\n",
			"\n\nconst (\n" +
				"    msgname_VAL1 int32 = 3\n" +
				")\n" +
				"\n" +
				"type msgname struct {\n" +
				"    msg.Package `ros:\"rospkg\"`\n" +
				"    msg.Definitions `ros:\"int32 VAL1=3\"`\n" +
				"    Var int32\n" +
				"}\n",
		},
		{
			"string definition without quotes",
			"string CONSTANT1=CONSTANT_VALUE_1\n" +
				"\n" +
				"string source\n",
			"\n\nconst (\n" +
				"    msgname_CONSTANT1 string = \"CONSTANT_VALUE_1\"\n" +
				")\n" +
				"\n" +
				"type msgname struct {\n" +
				"    msg.Package `ros:\"rospkg\"`\n" +
				"    msg.Definitions `ros:\"string CONSTANT1=\\\"CONSTANT_VALUE_1\\\"\"`\n" +
				"    Source string\n" +
				"}\n" +
				"",
		},
		{
			"string definition with quotes",
			"string CONSTANT1=\"CONSTANT_VALUE_1\"\n" +
				"\n" +
				"string source\n",
			"\n\nconst (\n" +
				"    msgname_CONSTANT1 string = \"CONSTANT_VALUE_1\"\n" +
				")\n" +
				"\n" +
				"type msgname struct {\n" +
				"    msg.Package `ros:\"rospkg\"`\n" +
				"    msg.Definitions `ros:\"string CONSTANT1=\\\"CONSTANT_VALUE_1\\\"\"`\n" +
				"    Source string\n" +
				"}\n" +
				"",
		},
	} {
		t.Run(ca.name, func(t *testing.T) {
			def, err := ParseMessageDefinition("gopkg", "rospkg", "msgname", ca.ros)
			require.NoError(t, err)
			golang, err := def.Write()
			require.NoError(t, err)
			require.Equal(t, ca.golang, golang)
		})
	}
}
