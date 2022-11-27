**Execute ```source install/setup.bash``` before every command**

Simulation mode - launch simple simulator:
```ruby
ros2 launch kia_test sim_load.launch.py auto_sync_mode:=True
```

With the KIA - launch GNSS node (Applanix) and drive-by-wire (IDAN)
```ruby
ros2 launch kia_test kia_load.launch.py
```

Record a trajectory:
```ruby
ros2 launch kia_test record_trajectory.launch.py
```

Drive the vehicle along the trajectory
```ruby
ros2 launch kia_test drive_vehicle.launch.py
```

Launch CognataStation simulator:
```ruby
ros2 run cognata_sdk_ros2 ROS2SDK <IP_Address> <Port>
```

CognataStation - Run sim converter node:
```ruby
ros2 run cognata_pkg convtocognata
```

CognataStation - Launch simulation:
```ruby
ros2 launch cognata.sim.launch.py
```
