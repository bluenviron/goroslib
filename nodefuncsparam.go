package goroslib

// ParamIsSet returns whether a parameter is set into the master.
func (n *Node) ParamIsSet(key string) (bool, error) {
	res, err := n.apiParamClient.HasParam(key)
	if err != nil {
		return false, err
	}
	return res.Res, nil
}

// ParamGetBool returns a bool parameter from the master.
func (n *Node) ParamGetBool(key string) (bool, error) {
	res, err := n.apiParamClient.GetParamBool(key)
	if err != nil {
		return false, err
	}
	return res.Res, nil
}

// ParamGetInt returns an int parameter from the master.
func (n *Node) ParamGetInt(key string) (int, error) {
	res, err := n.apiParamClient.GetParamInt(key)
	if err != nil {
		return 0, err
	}
	return res.Res, nil
}

// ParamGetString returns a string parameter from the master.
func (n *Node) ParamGetString(key string) (string, error) {
	res, err := n.apiParamClient.GetParamString(key)
	if err != nil {
		return "", err
	}
	return res.Res, nil
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
