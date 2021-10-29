# robotics_project_007

### Compile code :

```bash
git clone https://github.com/scifiswapnil/robotics_project_007
cd robotics_project_007
catkin_make
source devel/setup.bash
```

### To start simulation :

```bash
cd robotics_project_007
source devel/setup.bash
roslaunch simulation multi_robot.launch
```
*Note:* 
- By default the robots will be in formation. 
- Use the second last pose goal to drive the formation.

### Disable formation :

```bash
cd robotics_project_007
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
cd robotics_project_007
source devel/setup.bash
rosrun simulation payload.py
```
*Note:*
- Use the second last pose goal to drive the formation with payload.

---

## Obstacle avoidance 

```bash
cd robotics_project_007
source devel/setup.bash
roslaunch stage_sim stage.launch
```
*Note:*
- Use the 3 navigation buttons in the rviz to send individual goals to the robot.

#### Stage simulator

Stage is a robot simulator. It provides a virtual world populated by mobile robots and sensors, along with various objects for the robots to sense and manipulate. Stage provides several sensor and actuator models, including sonar or infrared rangers, scanning laser rangefinder, color-blob tracking, fiducial tracking, bumpers, grippers and mobile robot bases with odometric or global localization.

There are three ways to use Stage:
- The "stage" program: a standalone robot simulation program that loads your robot control program from a library that you provide.
- The Stage plugin for Player (libstageplugin) - provides a population of virtual robots for the popular Player networked robot interface system.
- Write your own simulator: the "libstage" C++ library makes it easy to create, run and customize a Stage simulation from inside your own programs.

Stage was designed with multi-agent systems in mind, so it provides fairly simple, computationally cheap models of lots of devices rather than attempting to emulate any device with great fidelity. This design is intended to be useful compromise between conventional high-fidelity robot simulations, the minimal simulations and the grid-world simulations common in artificial life research.

## Sequencial goals for single robot


```bash
cd robotics_project_007
source devel/setup.bash
roslaunch simulation single_robot.launch
```

*Note: *
- robot will automatically traverse a 3 goals back-to-back

## Sequencial goals for 3 robots


```bash
cd robotics_project_007
source devel/setup.bash
roslaunch simulation 3_single_robot.launch
```

*Note: *
- all 3 robots will automatically traverse a 3 goals back-to-back resp.