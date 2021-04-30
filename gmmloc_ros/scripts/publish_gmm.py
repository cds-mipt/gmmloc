#!/usr/bin/env python

import argparse
import rospy
import numpy as np
import tf

from tqdm import tqdm
from gmmloc import GMM_pb2
from google.protobuf.internal.decoder import _DecodeVarint32

from visualization_msgs.msg import MarkerArray, Marker


np.warnings.filterwarnings('ignore')


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gmm', type=str, required=True)
    return parser


def create_marker_msg(mean, cov):
    vals, vecs = np.linalg.eig(cov)
    vals = np.round(vals, 4)

    assert (vals >= 0).all() and np.isfinite(vals).all(), vals

    rotm = np.eye(4)
    rotm[:3, 0] = vecs[:, 0]
    rotm[:3, 1] = vecs[:, 1]
    rotm[:3, 2] = np.cross(rotm[:3, 0], rotm[:3, 1])

    quat = tf.transformations.quaternion_from_matrix(rotm)

    msg_marker = Marker()
    msg_marker.header.frame_id = 'map'
    msg_marker.header.stamp = rospy.Time.now()
    msg_marker.type = Marker.SPHERE
    msg_marker.action = Marker.ADD

    msg_marker.scale.x = 3 * np.sqrt(vals[0]) + 1e-4
    msg_marker.scale.y = 3 * np.sqrt(vals[1]) + 1e-4
    msg_marker.scale.z = 3 * np.sqrt(vals[2]) + 1e-4
    msg_marker.color.a = 0.5
    msg_marker.color.r = 0.4
    msg_marker.color.g = 1.0
    msg_marker.color.b = 0.4
    msg_marker.pose.position.x = mean[0]
    msg_marker.pose.position.y = mean[1]
    msg_marker.pose.position.z = mean[2]
    msg_marker.pose.orientation.x = quat[0]
    msg_marker.pose.orientation.y = quat[1]
    msg_marker.pose.orientation.z = quat[2]
    msg_marker.pose.orientation.w = quat[3]

    return msg_marker


def main(args):
    rospy.init_node('publish_gmm')

    with open(args.gmm, 'rb') as f:
        data = f.read()

    pos = 0
    num_comps, pos = _DecodeVarint32(data, pos)

    means, covs = [], []
    for _ in range(num_comps):
        msg_size, pos = _DecodeVarint32(data, pos)
        gmm = GMM_pb2.ComponentProto()
        gmm.ParseFromString(data[pos:pos + msg_size])
        pos += msg_size
        means.append(np.array(gmm.mean))
        covs.append(np.array(gmm.covariance).reshape(3, 3))

    msg_arr = MarkerArray()
    for i, (mean, cov) in enumerate(tqdm(zip(means, covs))):
        msg_marker = create_marker_msg(mean, cov)
        msg_marker.id = i
        msg_arr.markers.append(msg_marker)

    pub_gmm = rospy.Publisher('gmm', MarkerArray, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub_gmm.publish(msg_arr)
        rate.sleep()


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    main(args)
