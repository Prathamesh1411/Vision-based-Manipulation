### Topic- Grasp and Pick objects (mustard bottle, pudding box, and drill) using Franka robot

### Tasks-

1. Create a urdf/xacro/sdf file for the mustard bottle, pudding box, and drill. Spawn the table and three YCB dataset (Yale-CMU-Berkeley) objects- mustard bottle, pudding box, and drill in front of the robot.
2. Spawn only the table and the pudding box in front of the robot making sure that the pudding box is standing up.
3. Manually make the robot end effector go to the object such that its gripper can grasp it.
4. Start the robot far away from the object, where its camera is observing the table scene from a distance with pudding box spawned on it. Perform following tasks:
    * Capture a point cloud image of the scene
    * Run RANSAC algorithm of PCL
    * Detect the major plane
    * Remove it from the scene. In this way the object should be segmented from the scene.
5. Using the RANSAC algorithm, fit a plane to the object point cloud. Calculate thecenter of the points belonging to the plane. Calculate the normal of the    plane. The center points and the normal defines where you want one of your fingers at. Move the robot to the corresponding location, close the gripper      and lift the object.

### Deliverables-

1. Take snapshots of the gazebo environment for each spawned object and include it to the report.
2. Take a snapshot of your grasp in the gazebo simulator.
3. Visualize the following in the PCL visualizer and submit:
    * The scene without the segmentation
    * The segmented table plane
    * The segmented object plane
4. For 3 different pudding box position (all standing up), Take a snapshot of the robot in the pre-grasp position AND when the object is grasped AND when      the object is lifted. Include these to the report TOGETHER WITH the code.

### Results and Report-

Report consisting of the deliverables can be found in the ` Report ` folder.
