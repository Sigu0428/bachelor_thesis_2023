.rviz files contain a definition of the world/fixed frame name.
Either urdf files will need to contain the transformation/joint connecting the robot to the fixed frame, or a static_transform_publisher will have to be launched, to provide the transformation.
<node name="world2robot" pkg="tf" type="static_transform_publisher" 
      args="0 0 0 0 0 0 /world /robot1_tf/panda_link0 100" />
This is also used to allow for multiple robots, which can then be linked using this static_transform_publisher to the world frame.