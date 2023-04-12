# Usage Guide
To use this node on the physical car, run

```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```

This will use the parameters from the config file `config/config.yaml` 

If you want to test it out in simulation, which uses different topic names, run 

```bash
ros2 launch pure_pursuit sim_pure_pursuit_launch.py
```

### Trying different parameters without rebuilding

```bash
ros2 param set pure_pursuit_node K_p 0.1
```
