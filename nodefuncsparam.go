package goroslib

// ParamGetBool returns a bool parameter from the master.
func (n *Node) ParamGetBool(key string) (bool, error) {
	res, err := n.apiParamClient.ParamGetBool(key)
	if err != nil {
		return false, err
	}
	return res.Res, nil
}

// ParamGetInt returns an int parameter from the master.
func (n *Node) ParamGetInt(key string) (int, error) {
	res, err := n.apiParamClient.ParamGetInt(key)
	if err != nil {
		return 0, err
	}
	return res.Res, nil
}

// ParamGetString returns a string parameter from the master.
func (n *Node) ParamGetString(key string) (string, error) {
	res, err := n.apiParamClient.ParamGetString(key)
	if err != nil {
		return "", err
	}
	return res.Res, nil
}

// ParamSetBool sets a bool parameter in the master.
func (n *Node) ParamSetBool(key string, val bool) error {
	return n.apiParamClient.ParamSetBool(key, val)
}

// ParamSetInt sets an int parameter in the master.
func (n *Node) ParamSetInt(key string, val int) error {
	return n.apiParamClient.ParamSetInt(key, val)
}

// ParamSetString sets a string parameter in the master.
func (n *Node) ParamSetString(key string, val string) error {
	return n.apiParamClient.ParamSetString(key, val)
}
