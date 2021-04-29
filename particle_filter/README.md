### Particle Filter Localization

The majority of parameters you might want to tweak are in the launch/localize.launch file. You may have to modify the `odom_topic` or `scan_topic` parameters to match your environment.

```
user@ros-robot: roslaunch particle_filter localization.launch
```
