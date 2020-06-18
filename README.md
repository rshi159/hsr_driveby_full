# hsr_driveby_full
## REQUIREMENTS:

- Ubuntu 16.04 & ROS Kinetic
- Follow install instructions at:
	https://wiki.oxfordrobots.com/pages/viewpage.action?spaceKey=ROBOCUP&title=HSR+simulator+installation
- Get hsr_description from https://github.com/ToyotaResearchInstitute/hsr_description
- Install EXOTica


## INSTRUCTIONS:

### Driveby table in Gazebo
#### Descrition
- Uses AICO solver with a given base trajectory to pick up a soda_can off of a table.
-Assumed location of soda_can is known, obstacle locations known, and the base trajectory is valid and approaches the soda can
#### Run
- Start the gazebo simulation:
`roslaunch hsr_driveby_demo hsrb_empty_world.launch`
- Wait for gazebo to finish loading
- Press play to start the simulation
- In an new terminal window, go to scripts directory: `roscd hsr_driveby_full/scripts/rob_stuff/`
- 3 choices:
  - 1. Run a saved trajectory output of AICO solver `./hsrb_pickup.py`
  - 2. Rerun AICO solver `./hsr_meeting_table_aico`
  - 3. comment and uncomment line in main function.
#### Video
- see driveby_pickup.mp4
  
### Driveby table with Apriltags
#### Description
- Tried to incorporate Apriltags to identify position of sodacan
- Warning: this code may give you an aneurysm.
- Detects locations of Apriltags. Applies transform to get location of table center and bottle center.
- Finds the starting base pose of the HSR from the edge of the table and the bottle position. Filters out all noisy frames by checking of the and y of the frame are parallel to the world frame, so lots of lost data.
- Run the controller a bunch of times until it coincides with a moment where the starting base pose was just published.
- There is a chance the HSR won't drop the soda can or collide with the table.

#### Run
- `roslaunch hsr_driveby_full hsrb_empty_world_april_tag_metrics.launch `
- in a separate terminal window: `roscd hsr_driveby_full/scripts/rob_stuff` `./object_positions.py`
- in a separate terminal window: `roscd hsr_driveby_full/scripts/rob_stuff` `./trajectory_start_end.py`
- in a separate terminal window: `roscd hsr_driveby_full/scripts/rob_stuff` `./hsrb_exotica_python_script.py`
- by default, it should run a saved trajectory. comment and uncomment a line in the main function of hsrb_exotica_python_script.py.
### Video
-see go_to_location.movie.mp4 and go_to_location.movie_2.mp4
 
### Pick and Place Planner
#### Description
- Driveby pick up of soda can and driveby placing of soda can. In planning scene. Working on running in Gazebo. Kinda screwed up on converting SDF => Scene files. world frame offest by 90 degrees in xy plane. Need to fix /resources/sdf_to_scene_15_in_world_v2
- uses IK solver to find base positions for pickup and placing pose.
- RRTConnect to go from base poses: start_pose => grasp_pose => intermediate_pose (some pose between the start and placing location that acts as the end pose of the picking segment and the start pose of the placing segment) => place_pose => start_pose
- AICO solver to solve for grasping and placing motions.
 
- need to specify start pose, intermediate pose, soda_can location, target_placing location, duration_of_gripper_opening/closing, and timestep for AICO solver. These are all currently hardcoded.
#### Run
- In a terminal, start roscore: `roscore`
- In a separate terminal, start rviz for the hsr: `roscd hsr_driveby_full/resources` `rviz -d hsr_driveby.rviz`
- In a separate terminal, run the planner: `roscd hsr_driveby_full/scripts/rob_stuff` `./hsr_wrs_world_pick_and_place_full`
#### Video
- see hsr_pick_and_place.mp4




