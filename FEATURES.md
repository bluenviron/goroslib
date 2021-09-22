
# Library features

## High level client features

|name|status|
|----|------|
|publish to topics|:heavy_check_mark:|
|subscribe to topics|:heavy_check_mark:|
|provide services|:heavy_check_mark:|
|call services|:heavy_check_mark:|
|provide actions|:heavy_check_mark:|
|call actions|:heavy_check_mark:|
|provide simple actions|:heavy_check_mark:|
|call simple actions|:heavy_check_mark:|
|support namespaces|:heavy_check_mark:|
|support IPv6|:heavy_check_mark:|
|provide a time API|:heavy_check_mark:|

## Client library requirements

https://wiki.ros.org/Implementing%20Client%20Libraries

|name|status|
|----|------|
|implement the slave side of the master/slave API|:heavy_check_mark:|
|handle node-to-node transport negotiation and connection setup|:heavy_check_mark:|
|handle transport-specific serialization and deserialization of messages|:heavy_check_mark:|
|parse command-line Remapping Arguments|:heavy_check_mark:|
|Subscribe to a simulated Clock|:heavy_check_mark:|
|publish debugging messages to rosout|:heavy_check_mark:|
|object representation of message types|:heavy_check_mark:|
|event loop for connection servicing|:heavy_check_mark:|
|user callback invocation on message receipt|:heavy_check_mark:|

## Protocols

|name|status|
|----|------|
|xml-rpc|:heavy_check_mark:|
|TCPROS|:heavy_check_mark:|
|UDPROS|:heavy_check_mark:|

## Master API

https://wiki.ros.org/ROS/Master_API

|method|client|
|------|------|
|registerService|:heavy_check_mark:|
|unregisterService|:heavy_check_mark:|
|registerSubscriber|:heavy_check_mark:|
|unregisterSubscriber|:heavy_check_mark:|
|registerPublisher|:heavy_check_mark:|
|unregisterPublisher|:heavy_check_mark:|
|lookupNode|:heavy_check_mark:|
|getPublishedTopics|:heavy_check_mark:|
|getTopicTypes|:heavy_check_mark:|
|getSystemState|:heavy_check_mark:|
|getUri|:heavy_check_mark:|
|lookupService|:heavy_check_mark:|

## Parameter Server API

https://wiki.ros.org/ROS/Parameter%20Server%20API

|method|client|
|------|------|
|deleteParam|:heavy_check_mark:|
|setParam|:heavy_check_mark:|
|getParam|:heavy_check_mark:|
|searchParam|:heavy_check_mark:|
|subscribeParam||
|unsubscribeParam||
|hasParam|:heavy_check_mark:|
|getParamNames|:heavy_check_mark:|

## Slave API

https://wiki.ros.org/ROS/Slave_API

|method|client|server|
|------|------|------|
|getBusStats|||
|getBusInfo|:heavy_check_mark:|:heavy_check_mark:|
|getMasterUri|||
|shutdown|:heavy_check_mark:|:heavy_check_mark:|
|getPid|:heavy_check_mark:|:heavy_check_mark:|
|getSubscriptions|||
|getPublications|:heavy_check_mark:|:heavy_check_mark:|
|paramUpdate|||
|publisherUpdate|:heavy_check_mark:|:heavy_check_mark:|
|requestTopic|:heavy_check_mark:|:heavy_check_mark:|
