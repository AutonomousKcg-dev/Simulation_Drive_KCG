this readme is about how to run the code with cognata Simulation

Simulation mode - launch simple simulator - "connect to the real car:
	
	ros2 launch kia_test sim_load.launch.py auto_sync_mode:=True

Open Cognata Simulation:

    cd /cognata_station_2_0/cognataStation
    
    ./CognataStation-2022.2.5.AppImage
    
    run Simulation
    
Connect Via cliant:

    ros2 run cognata_sdk_ros2 ROS2SDK 192.0.0.1 3056
    
Record a trajectory:
	
	ros2 launch cognata_sim.launch.py


Drive the vehicle along the trajectory:
	
	ros2 launch kia_test drive_vehicle.launch.py

Drive the vehicle along the trajectory:
    
    ros2 run cognata_pkg conv
    

