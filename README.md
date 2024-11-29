# assignment1_rt

This project is part of the **Research Track 1** course and aims to create a ROS package, named assignment1_rt, that includes 2 nodes: a ui node and a distance node.

## How to Run:
1. ** Clone the repository to your workspace**:

```bash
cd ~/workspace/src
git clone https://github.com/luisa38/assignment1_rt.git
```

2. ** Build the package**:

```bash
cd ~/workspace
catkin_make
source devel/setup.bash
```

3. ** Launch turtlesim**:

```bash
rosrun turtlesim turtlesim_node
```

4. ** Run the 2 nodes**:

```bash
rosrun assignment1_rt ui_node.py
rosrun assignment1_rt distance_node
```

## The UI Node:

This node spawns a second turtel, `turtle2`, at `(1.0, 1.0)` when initialized - this position in in one of the boundaries, so some messages regarding that may appear in the beginning. In the terminal the user can control the turtles.

### Features - 
1. Spawning a new turtle - `turtle2`:
```python
rospy.wait_for_service('/spawn')
spawn_service = rospy.ServiceProxy('/spawn', Spawn)
spawn_service(1.0, 1.0, 0.0, "turtle2")
```
2. User can choose the turtle to control ('1' or '2'):

 ``choice = input("Choose (1/2): ")``

3. User can specify the linear and angular velocities:
```python
'linear_vel = float(input("Insert linear velocity: "))
angular_vel = float(input("Insert angular velocity: "))'
```

4. The commands are sent to the respective turtle for 1 second:
```python
vel = Twist()
vel.linear.x = linear_vel
vel.angular.z = angular_vel

if choice == '1':
    pub_turtle1.publish(vel)
else:
    pub_turtle2.publish(vel)
rospy.sleep(1)
```

5. Stop the turtle after 1 second:
```python
vel.linear.x = 0
vel.angular.z = 0
if choice == '1':
    pub_turtle1.publish(vel)
else:
    pub_turtle2.publish(vel)
```

## The Distance Node:

This node continuously monitures and enforces the turtles' positions and distances.

### Features - 
1. Monitoring the turtle positions:
```cpp
ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 1, turtle1PoseCallback);
ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 1, turtle2PoseCallback);
```

2. Calculates the distance between `turtle1` and `turtle2`:
```cpp
double calculateDistance() {
    double dx = x1 - x2;
    double dy = y1_turtle - y2_turtle;
    return sqrt(dx * dx + dy * dy);
}
```

3. Stops the turtle when they're too close (less than `1.5`):
```cpp
if (distance < DISTANCE_THRESHOLD) {
    ROS_WARN("Turtles too close! Stopping.");
    stopTurtle(turtle2_pub);
    stopTurtle(turtle1_pub);
}
```

4. Stops the turtle if they move outside the bounderies:
```cpp
if (isOutOfBounds(x1, y1_turtle)) {
    ROS_WARN("Turtle1 is out of bounds. Stopping.");
    stopTurtle(turtle1_pub);
}
if (isOutOfBounds(x2, y2_turtle)) {
    ROS_WARN("Turtle2 is out of bounds. Stopping.");
    stopTurtle(turtle2_pub);
}
```

### Limits defined

**Distance Threshold**: `1.5`
**Boundary Limits**: `x` and `y` must stay within `[1.0, 10.0]`.
 
