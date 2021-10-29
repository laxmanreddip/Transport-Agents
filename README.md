### Compile code :
Download and extract the files
cd (Location of the downloaded directory)
catkin_make

source devel/setup.bash
```

### To start simulation :

```bash
cd agents
source devel/setup.bash
roslaunch simulation multi_robot.launch
```
*Note:* 
- By default the robots will be in formation. 
- Use the second last pose goal to drive the formation.

### Disable formation :

```bash
cd agents
source devel/setup.bash
rosrun simulation formation_disable.py
```
*Note:* 
- Use the first six pose goal buttons on rviz to drive the individual robots. 

### Enable formation:

*Note:* 
- DO THIS ONLY WHEN YOU HAVE DISABLED THE FORMATION.
- Use the last pose goal button to individually drive the formation to a goal and enable the formation controller.

### Spawn payload:

```bash
cd agents
source devel/setup.bash
rosrun simulation payload.py
```
*Note:*
- Use the second last pose goal to drive the formation with payload.

---

## Dynamic Obstacle avoidance 

```bash
cd agents
source devel/setup.bash
roslaunch stage_sim stage.launch
```
*Note:*
- Use the 3 navigation buttons in the rviz to send individual goals to the robot.



## Sequencial goals for single robot


```bash
cd agents
source devel/setup.bash
roslaunch simulation single_robot.launch
```

*Note: *
- robot will automatically traverse a 3 goals back-to-back

## Sequencial goals for 3 robots


```bash
cd agents
source devel/setup.bash
roslaunch simulation 3_single_robot.launch
```

*Note: *
- all 3 robots will automatically traverse a 3 goals back-to-back resp.
