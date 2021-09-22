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

// ParamSetBool sets a bool parameter in the master.
func (n *Node) ParamSetBool(key string, val bool) error {
	return n.apiParamClient.SetParamBool(key, val)
}

// ParamSetInt sets an int parameter in the master.
func (n *Node) ParamSetInt(key string, val int) error {
	return n.apiParamClient.SetParamInt(key, val)
}

// ParamSetString sets a string parameter in the master.
func (n *Node) ParamSetString(key string, val string) error {
	return n.apiParamClient.SetParamString(key, val)
}
