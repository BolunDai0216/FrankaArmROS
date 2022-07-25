# FrankaArmROS

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
