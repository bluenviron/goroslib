// msgs contains base types and standard messages
package msgs

import (
	"time"
)

// extra field that is added to set package name
type Package int

// base types, http://wiki.ros.org/msg#Field_Types
type Bool bool
type Int8 int8
type Uint8 uint8
type Int16 int16
type Uint16 uint16
type Int32 int32
type Uint32 uint32
type Int64 int64
type Uint64 uint64
type Float32 float32
type Float64 float64
type String string
type Time = time.Time
type Duration = time.Duration

// deprecated types
type Byte int8
type Char uint8
