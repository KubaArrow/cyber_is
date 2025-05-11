```
sudo apt install libyaml-cpp-dev

```

```
rostopic pub /prohibition_input geometry_msgs/PolygonStamped "
header:
  frame_id: 'map'
polygon:  
    points:
    - {x: -2.0, y: -1.0, z: 0.0}
    - {x: -2.0, y: 1.0, z: 0.0}
    - {x: 0.0, y: 1.0, z: 0.0}
    - {x: 0.0, y: -1.0, z: 0.0}"


```