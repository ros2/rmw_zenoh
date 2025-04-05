# zenoh_security_tools

The `zenoh_security_tools` package contains the `generate_configs` executable which generates Zenoh session config files with access control, authentication and encryption parameters based on policies and keystores generated using [sros2](https://github.com/ros2/sros2).


# Generate zenoh config file using policy.xml

 1 ) Launch zenohd
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

 2 ) Launch the listener
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp listener
```

 3 ) Launch the talker
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp talker
```

Now run the policy generator

```bash
ros2 security generate_policy policy_listener_talker.xml
```

Finally run the script:

```bash
ros2 run zenoh_security_configuration_tools zenoh_security_configuration_tools --policy policy_service.xml --config <path to default session config>/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5
```

# Try access control

 1) Run the talker with the new config file
```bash
export ZENOH_SESSION_CONFIG_URI=talker.json5
ros2 run demo_nodes_cpp talker
[INFO] [1740601932.350808475] [talker]: Publishing: 'Hello World: 1'
[INFO] [1740601933.350487483] [talker]: Publishing: 'Hello World: 2'
```

 2) Run the listener with the new config file
```bash
export ZENOH_SESSION_CONFIG_URI=listener.json5
ros2 run demo_nodes_cpp listener
...
[INFO] [1740602312.492840958] [listener]: I heard: [Hello World: 1]
[INFO] [1740602313.492200366] [listener]: I heard: [Hello World: 2]
```

You can check that everything is fine remapping the topic name (this should not work):

```bash
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=new_topic
```

```bash
ros2 run demo_nodes_cpp listener --ros-args -r chatter:=new_topic
...
# listener should not receive anything
```

# policy files

Just in case you want to try this tools here you can find some examples



<details>
<summary><b>policy_talker_listerner.xml</b></summary>
```xml
<policy version="0.2.0">
  <enclaves>
    <enclave path="/">
      <profiles>
        <profile node="listener" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/get_type_description</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>chatter</topic>
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
        <profile node="talker" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/get_type_description</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>chatter</topic>
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```
</details>

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

<details>
<summary><b>Policy_service.xml</b></summary>
```xml
<policy version="0.2.0">
  <enclaves>
    <enclave path="/">
      <profiles>
        <profile node="add_two_ints_client" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/get_type_description</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <services request="ALLOW">
            <service>add_two_ints</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
        <profile node="add_two_ints_server" ns="/">
          <services reply="ALLOW">
            <service>add_two_ints</service>
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/get_type_description</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```
</details>

```bash
ros2 run demo_nodes_cpp add_two_ints_client
ros2 run demo_nodes_cpp add_two_ints_server
```

<details>
<summary><b>policy_action.xml</b></summary>
```xml
<policy version="0.2.0">
  <enclaves>
    <enclave path="/">
      <profiles>
        <profile node="fibonacci_action_client" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/get_type_description</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <services request="ALLOW">
            <service>/fibonacci/_action/cancel_goal</service>
            <service>/fibonacci/_action/get_result</service>
            <service>/fibonacci/_action/send_goal</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>/fibonacci/_action/feedback</topic>
            <topic>/fibonacci/_action/status</topic>
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
        <profile node="fibonacci_action_server" ns="/">
          <services reply="ALLOW">
            <service>/fibonacci/_action/cancel_goal</service>
            <service>/fibonacci/_action/get_result</service>
            <service>/fibonacci/_action/send_goal</service>
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/get_type_description</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>/fibonacci/_action/feedback</topic>
            <topic>/fibonacci/_action/status</topic>
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```
</details>

```bash
ros2 run action_tutorials_cpp fibonacci_action_client
ros2 run action_tutorials_cpp fibonacci_action_server
```
