package goroslib

// GetParamBool returns a bool parameter from the master.
func (n *Node) GetParamBool(key string) (bool, error) {
	res, err := n.apiParamClient.GetParamBool(key)
	if err != nil {
		return false, err
	}
	return res.Res, nil
}

// GetParamInt returns an int parameter from the master.
func (n *Node) GetParamInt(key string) (int, error) {
	res, err := n.apiParamClient.GetParamInt(key)
	if err != nil {
		return 0, err
	}
	return res.Res, nil
}

// GetParamString returns a string parameter from the master.
func (n *Node) GetParamString(key string) (string, error) {
	res, err := n.apiParamClient.GetParamString(key)
	if err != nil {
		return "", err
	}
	return res.Res, nil
}

// SetParamBool sets a bool parameter in the master.
func (n *Node) SetParamBool(key string, val bool) error {
	return n.apiParamClient.SetParamBool(key, val)
}

// SetParamInt sets an int parameter in the master.
func (n *Node) SetParamInt(key string, val int) error {
	return n.apiParamClient.SetParamInt(key, val)
}

// SetParamString sets a string parameter in the master.
func (n *Node) SetParamString(key string, val string) error {
	return n.apiParamClient.SetParamString(key, val)
}
