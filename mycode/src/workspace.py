#!/usr/bin/env python3

import torch
import math
import pytorch_kinematics as pk
import numpy
import yaml

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

pub = rospy.Publisher('workspacePointCloud', PointCloud, queue_size=1, latch=True)
rospy.init_node('urdfToWorkspace', anonymous=True)

END_EFFECTOR = rospy.get_param("end_effector_name") # "panda_link8"
BASE = rospy.get_param("base_name") # "panda_link0_sc"
URDF_PATH = rospy.get_param("urdf_path") # "/home/sigurd/catkin_ws/src/mycode/src/panda_generated.urdf"

# if gpu available, set devices
d = "cuda" if torch.cuda.is_available() else "cpu"
dtype = torch.float32

# load robot description from URDF and specify end effector link
chain = pk.build_serial_chain_from_urdf(open(URDF_PATH).read(), END_EFFECTOR)

# Print kinematic chain
print(chain)

print(chain.get_joint_parameter_names()[0])

N = 10000
# N Uniform random configuration [0, 1]
th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=d)

# Load yaml file with joint limits
with open("/home/sigurd/catkin_ws/src/mycode/src/joint_limits.yaml", "r") as stream:
    try:
        joint_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Make uniform random configurations [0, 1] correspond to limits [lower, upper]
for x in th_batch:
    for i, value in enumerate(joint_data.values()):
        x[i] *= value["limit"]["upper"] - value["limit"]["lower"]
        x[i] += value["limit"]["lower"]

# Generate forward kinematics 3D transform N x 2D transforms
ret = chain.forward_kinematics(th_batch, end_only=True)

# 3D jacobian N x 2D jacobians and svd
J = chain.jacobian(th_batch)
svd = torch.svd(J)

singular_metric = ChannelFloat32()
singular_metric.name = "singular metric"
for x in svd.S:
    singular_metric.values.append(x[-1])

points = torch.zeros(N, 1, 3)
tf_points = ret.transform_points(points)

h = Header()
h.stamp = rospy.Time.now()
h.frame_id = BASE

pointCloud = PointCloud()
pointCloud.header = h

for i, [x] in enumerate(tf_points.numpy()):
    p = Point(x[0], x[1], x[2])
    pointCloud.points.append(p)

pointCloud.channels = [singular_metric]


pub.publish(pointCloud)