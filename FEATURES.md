
# goroslib Client Features

## High level

|name|status|
|----|------|
|publish to topics|ok|
|subscribe to topics|ok|
|provide services|ok|
|call services|ok|
|provide actions|ok|
|call actions|ok|
|provide simple actions|ok|
|call simple actions|ok|
|support namespaces|ok|
|support IPv6|ok|
|provide a time API|ok|

## Client library requirements

https://wiki.ros.org/Implementing%20Client%20Libraries

|name|status|
|----|------|
|implement the slave side of the master/slave API|ok|
|handle node-to-node transport negotiation and connection setup|ok|
|handle transport-specific serialization and deserialization of messages|ok|
|parse command-line Remapping Arguments| |
|Subscribe to a simulated Clock|ok|
|publish debugging messages to rosout| |
|object representation of message types|ok|
|event loop for connection servicing|ok|
|user callback invocation on message receipt|ok|

## Protocols

|name|status|
|----|------|
|xml-rpc|ok|
|TCPROS|ok|
|UDPROS|ok|

## Master API

https://wiki.ros.org/ROS/Master_API

|method|client|
|------|------|
|registerService|ok|
|unregisterService|ok|
|registerSubscriber|ok|
|unregisterSubscriber|ok|
|registerPublisher|ok|
|unregisterPublisher|ok|
|lookupNode|ok|
|getPublishedTopics|ok|
|getTopicTypes|ok|
|getSystemState|ok|
|getUri|ok|
|lookupService|ok|

## Parameter Server API

https://wiki.ros.org/ROS/Parameter%20Server%20API

|method|client|
|------|------|
|deleteParam|ok|
|setParam|ok|
|getParam|ok|
|searchParam|ok|
|subscribeParam||
|unsubscribeParam||
|hasParam|ok|
|getParamNames|ok|

## Slave API

https://wiki.ros.org/ROS/Slave_API

|method|client|server|
|------|------|------|
|getBusStats|||
|getBusInfo|ok|ok|
|getMasterUri|||
|shutdown|ok|ok|
|getPid|ok|ok|
|getSubscriptions|||
|getPublications||ok|
|paramUpdate|||
|publisherUpdate||ok|
|requestTopic|ok|ok|
