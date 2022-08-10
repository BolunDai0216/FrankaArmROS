# FrankaArmROS

## Run the PD controller

To run the PD controller, use the command

```console
roslaunch franka_arm_ros franka_arm_ros.launch controller:=panda/pd_controller
```

## How to start simulation

If we take a look at this tag:

```xml
<node pkg="controller_manager" 
      type="spawner" 
      name="$(arg arm_id)_controller_spawner" 
      respawn="false" 
      output="screen" 
      args="--wait-for initialized franka_state_controller $(arg controller)" />
```

we can see that the controllers are not activated unless the topic `/initialized` is `true`. Therefore, to start the simulation we can simply publish to the `/initialized` topic

```console
rostopic pub /initialized std_msgs/Bool "data: true"
```

More regarding `spawner` can be found in its help message

```console
>>> rosrun controller_manager spawner -h
usage: spawner [-h] [--stopped] [--wait-for topic] [--namespace ns] [--timeout T] [--no-timeout]
               [--shutdown-timeout SHUTDOWN_TIMEOUT]
               controller [controller ...]

Controller spawner

positional arguments:
  controller            controllers to load

optional arguments:
  -h, --help            show this help message and exit
  --stopped             loads controllers, but does not start them
  --wait-for topic      does not load or start controllers until it hears "True" on a topic (Bool)
  --namespace ns        namespace of the controller_manager services
  --timeout T           how long to wait for controller_manager services when starting up [s] (default: 30). <=0 waits
                        indefinitely.
  --no-timeout          wait for controller_manager services indefinitely (same as `--timeout 0`)
  --shutdown-timeout SHUTDOWN_TIMEOUT
                        DEPRECATED: this argument has no effect.
```
