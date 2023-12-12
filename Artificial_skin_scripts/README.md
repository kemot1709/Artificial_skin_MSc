## Lunching Dialogflow
```
source ./stasiek_ws/devel/setup.bash
roslaunch dialogflow websocket.launch
```

## My nodes
```
cd ./Desktop/Tomek_Indeka_ArtSkin/Artificial_skin_MSc/Artificial_skin_scripts/
source ../../venv/Art_Skin_MSc/bin/activate
```

### Inteligent table node
```
sudo chmod 777 /dev/ttyUSB0
python3 ./intelligent_table.py
```

Turn on node
```
rostopic pub /table/sgn_on std_msgs/Bool "data: true"
```

### Scenario node
```
python3 test_usage_node.py
```

## Undock robot
```
rostopic pub /undocker_server/goal laser_servoing_msgs/UndockActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}"
```

## Dock robot
```
rostopic pub /go_and_dock/goal dock_charge_sm_msgs/GoAndDockActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  retry_delay:
    secs: 0
    nsecs: 0
  use_current_pose: true"
```
