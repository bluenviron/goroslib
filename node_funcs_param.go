package goroslib

// ParamIsSet returns whether a parameter is set into the master.
func (n *Node) ParamIsSet(key string) (bool, error) {
	return n.apiParamClient.HasParam(key)
}

// ParamGetBool returns a bool parameter from the master.
func (n *Node) ParamGetBool(key string) (bool, error) {
	return n.apiParamClient.GetParamBool(key)
}

// ParamGetInt returns an int parameter from the master.
func (n *Node) ParamGetInt(key string) (int, error) {
	return n.apiParamClient.GetParamInt(key)
}

// ParamGetString returns a string parameter from the master.
func (n *Node) ParamGetString(key string) (string, error) {
	return n.apiParamClient.GetParamString(key)
}

// ParamGetFloat64 returns a float64 parameter from the master.
func (n *Node) ParamGetFloat64(key string) (float64, error) {
	return n.apiParamClient.GetParamFloat64(key)
}

// ParamSetBool sets a bool parameter into the master.
func (n *Node) ParamSetBool(key string, val bool) error {
	return n.apiParamClient.SetParamBool(key, val)
}

// ParamSetInt sets an int parameter into the master.
func (n *Node) ParamSetInt(key string, val int) error {
	return n.apiParamClient.SetParamInt(key, val)
}

// ParamSetString sets a string parameter into the master.
func (n *Node) ParamSetString(key string, val string) error {
	return n.apiParamClient.SetParamString(key, val)
}

// ParamSetFloat64 sets a float64 parameter into the master.
func (n *Node) ParamSetFloat64(key string, val float64) error {
	return n.apiParamClient.SetParamFloat64(key, val)
}
