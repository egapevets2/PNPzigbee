This is for ESP32C6, implementing an end-device or a router.
it receives packets from the coordinator, which can command things like set motor speed.
it also outputs all packet info out the serial port, so that an arduino co-processor has
connection to the coordinator.  
the concept is to get the ESP32C6 network solid, working in the espressdif system in vscode,
then lock that down and do all subsequent code in the arduino, in a much less critical environment.
