#!/usr/bin/env python3

import torch
import math
import pytorch_kinematics as pk
import numpy
import yaml
from math import prod
import time

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

rospy.init_node('urdfToWorkspace', anonymous=True)
rospy.loginfo("genPointCloud: START")

END_EFFECTOR = rospy.get_param("end_effector_name") # "panda_link8"

# eg. "robot1_tf/panda_link0_sc" COULD BE A LIST OF BASE NAMES (in which case the same pointcloud will be published with each base frames on different topics)
BASE = rospy.get_param("base_name") 

MODEL = rospy.get_param("robot_description") # "/home/sigurd/catkin_ws/src/mycode/src/panda_generated.urdf"
JOINT_LIMITS = rospy.get_param("joint_limit_yaml")

N = rospy.get_param("N_points", 100000)
BATCHES = rospy.get_param("batches", 10)

# if gpu available, set devices
d = "cuda" if torch.cuda.is_available() else "cpu"
dtype = torch.float32

from contextlib import contextmanager,redirect_stderr,redirect_stdout
from os import devnull

@contextmanager
def suppress_stdout_stderr():
    """A context manager that redirects stdout and stderr to devnull"""
    with open(devnull, 'w') as fnull:
        with redirect_stderr(fnull) as err, redirect_stdout(fnull) as out:
            yield (err, out)

t0 = time.process_time()
# load robot description from URDF and specify end effector link
with suppress_stdout_stderr():
    chain = pk.build_serial_chain_from_urdf(MODEL, END_EFFECTOR)
#chain = pk.build_serial_chain_from_urdf(open("/home/sigurd/catkin_ws/src/mycode/src/panda_generated.urdf").read(), END_EFFECTOR)
rospy.loginfo("genPointCloud: Load URDF -> DONE, time elapsed:{}".format(time.process_time()-t0))

# Print kinematic chain
#print(chain)

#print(chain.get_frame_names())

singular_metric = ChannelFloat32()
singular_metric.name = "singular metric"
points = []
for i in range(BATCHES):

    t0 = time.process_time()
    # N Uniform random configuration [0, 1]
    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=d)
    rospy.loginfo("genPointCloud: generate configurations -> DONE, time elapsed:{}".format(time.process_time()-t0))

    t0 = time.process_time()
    # Load yaml file with joint limits
    try:
        joint_data = yaml.safe_load(JOINT_LIMITS)
    except yaml.YAMLError as exc:
        print(exc)

    # Make uniform random configurations [0, 1] correspond to limits [lower, upper]
    for x in th_batch:
        for i, value in enumerate(joint_data.values()):
            x[i] *= (value["limit"]["upper"] - value["limit"]["lower"])
            x[i] += value["limit"]["lower"]

    #limit_width = torch.zeros(N, 1, 3)
    #for i, value in enumerate(joint_data.values()):
    #        limit_width[i] = value
    #        x[i] += value["limit"]["lower"]
    #torch.diag()

    rospy.loginfo("genPointCloud: load joint limits -> DONE, time elapsed:{}".format(time.process_time()-t0))

    t0 = time.process_time()
    # Generate forward kinematics 3D transform N x 2D transforms
    ret = chain.forward_kinematics(th_batch, end_only=True)
    rospy.loginfo("genPointCloud: gen. fk -> DONE, time elapsed:{}".format(time.process_time()-t0))

    # 3D jacobian N x 2D jacobians and svd
    t0 = time.process_time()
    J = chain.jacobian(th_batch)
    rospy.loginfo("genPointCloud: gen. jacobian -> DONE, time elapsed:{}".format(time.process_time()-t0))

    t0 = time.process_time()
    svd = torch.svd(J)
    rospy.loginfo("genPointCloud: gen. svd -> DONE, time elapsed:{}".format(time.process_time()-t0))

    t0 = time.process_time()
    for x in svd.S:
        #singular_metric.values.append(prod(x)**(1.0/6.0))
        #singular_metric.values.append(x[-1] / x[0])
        singular_metric.values.append(x[-1])
        #singular_metric.values.append(x[0] * x[1] * x[2] * x[3] * x[4] * x[5])
    rospy.loginfo("genPointCloud: calc. manipulability index -> DONE, time elapsed:{}".format(time.process_time()-t0))

    t0 = time.process_time()
    zero_points = torch.zeros(N, 1, 3)
    tf_points = ret.transform_points(zero_points)

    for i, [x] in enumerate(tf_points.numpy()):
        p = Point(x[0], x[1], x[2])
        points.append(p)
    rospy.loginfo("genPointCloud: tf points using fk -> DONE, time elapsed:{}".format(time.process_time()-t0))
    

# if base is not a list, make it one (a list of bases will publish the same pointcloud on multiple topics, relative to multiple frames)
if isinstance(BASE, str):
    BASE = [BASE]

for base in BASE:
    pointCloud = PointCloud()
    pointCloud.header.stamp = rospy.Time.now()
    pointCloud.points = points
    pointCloud.channels = [singular_metric]
    pointCloud.header.frame_id = base

    pub = rospy.Publisher('{}/workspacePointCloud'.format(base), PointCloud, queue_size=1, latch=True)
    pub.publish(pointCloud)

rospy.spin()