import rospy
import tf
import numpy as np
import pandas as pd
import json
import time
import torch
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PointStamped
from environment import Environment, Scene, Node
from model.model_registrar import ModelRegistrar
from model.online.online_trajectron import OnlineTrajectron
from pedsim_msgs.msg import TrackedPersons
from sidewalk_msgs.msg import Trajectories, Trajectory

standardization = {
    'PEDESTRIAN': {
        'position': {
            'x': {'mean': 0, 'std': 1},
            'y': {'mean': 0, 'std': 1}
        },
        'velocity': {
            'x': {'mean': 0, 'std': 2},
            'y': {'mean': 0, 'std': 2}
        },
        'acceleration': {
            'x': {'mean': 0, 'std': 1},
            'y': {'mean': 0, 'std': 1}
        }
    }
}

data_columns = pd.MultiIndex.from_product([['position', 'velocity', 'acceleration'], ['x', 'y']])

device = torch.device("cpu")

class PredictionException(Exception):
    pass


class Prediction:
    def __init__(self):
        if not rospy.has_param('prediction_config'):
            raise PredictionException('No prediction configuration found')
        self.config = rospy.get_param('prediction_config')

        self.cnt = 0

        model_config_dir = rospy.get_param('model_config_dir')
        model_dir = rospy.get_param('model_dir')
        self.hyperparams = dict()
        self.load_hyperparams(model_config_dir)

        self.online_env = self.create_online_env()
        self.online_trajectron = self.create_online_model(model_dir, self.online_env, 0)

        self.listener = tf.TransformListener()

        # initiate subscribers
        rospy.Subscriber('/tracked_persons', TrackedPersons,
                         self.tracked_persons_callback)

        self.pred_pub = rospy.Publisher('/prediction_trajectory', Trajectories, queue_size=1)
        self.local_pred_pub = rospy.Publisher('/local_prediction_trajectory', Trajectories, queue_size=1)
        self.visual_pred_pub = rospy.Publisher('/visualization_prediction_trajectory', MarkerArray, queue_size=1)

    def load_hyperparams(self, config_file):
        with open(config_file, 'r') as conf_json:
            self.hyperparams = json.load(conf_json)

        # Add hyperparams from arguments
        self.hyperparams['dynamic_edges'] = 'yes'
        self.hyperparams['edge_state_combine_method'] = 'sum'
        self.hyperparams['edge_influence_combine_method'] = 'attention'
        self.hyperparams['edge_addition_filter'] = [0.25, 0.5, 0.75, 1.0]
        self.hyperparams['edge_removal_filter'] = [1.0, 0]
        self.hyperparams['batch_size'] = 256
        self.hyperparams['k_eval'] = 25
        self.hyperparams['offline_scene_graph'] = "yes"
        self.hyperparams['incl_robot_node'] = False
        self.hyperparams['edge_encoding'] = True
        self.hyperparams['use_map_encoding'] = False

    def create_online_model(self, model_dir, online_env, init_timestep):
        model_registrar = ModelRegistrar(model_dir, device)
        model_registrar.load_models(iter_num=100)
        trajectron = OnlineTrajectron(model_registrar,
                                      self.hyperparams,
                                      device)
        trajectron.set_environment(online_env, init_timestep)
        return trajectron

    def create_online_env(self):
        env = Environment(node_type_list=['PEDESTRIAN'], standardization=standardization)
        attention_radius = dict()
        attention_radius[(env.NodeType.PEDESTRIAN, env.NodeType.PEDESTRIAN)] = 3.0
        env.attention_radius = attention_radius
        scenes = []
        scene = Scene(timesteps=7 + 1, dt=0.4, name="ntu", aug_func=None)
        scenes.append(scene)
        env.scenes = scenes
        return env

    def make_up_input_dict(self, msg):
        input_dict = dict()
        for agent in msg.tracks:
            x = agent.pose.pose.position.x
            y = agent.pose.pose.position.y
            vx = agent.twist.twist.linear.x
            vy = agent.twist.twist.linear.y
            ax = 0
            ay = 0
            data_dict = {('position', 'x'): [x],
                         ('position', 'y'): [y],
                         ('velocity', 'x'): [vx],
                         ('velocity', 'y'): [vy],
                         ('acceleration', 'x'): [ax],
                         ('acceleration', 'y'): [ay]}

            node_data = pd.DataFrame(data_dict, columns=data_columns)
            agent_name = "PEDESTRIAN/" + str(agent.track_id)
            node = Node(node_type=self.online_env.NodeType.PEDESTRIAN, node_id=agent_name, data=node_data)

            input_dict[node] = [agent.pose.pose.position.x, agent.pose.pose.position.y,
                                agent.twist.twist.linear.x, agent.twist.twist.linear.y,
                                0, 0]
        return input_dict

    def predict(self, input_dict):
        dists, preds = self.online_trajectron.incremental_forward(input_dict,
                                                                  None,
                                                                  prediction_horizon=self.hyperparams[
                                                                      "prediction_horizon"],
                                                                  num_samples=1,
                                                                  robot_present_and_future=None,
                                                                  full_dist=True)
        return dists, preds

    def tracked_persons_callback(self, msg):
        """
        Callback function for the vision_cone_detector topic.
        """

        # due to the computation cost via CPU, calculate one time in 0.4s
        if self.cnt % 2 != 0:
            self.cnt = self.cnt + 1
            return
        self.cnt = 1

        start = time.time()
        input_dict = self.make_up_input_dict(msg)
        dists, preds = self.predict(input_dict)
        end = time.time()

        print(f"time cost: {end - start}")

        markers = MarkerArray()
        trajectories = Trajectories()
        trajectories.dt = 0.4
        trajectories.header.frame_id = "map"
        trajectories.header.stamp = msg.header.stamp

        local_trajectories = Trajectories()
        local_trajectories.dt = 0.4
        local_trajectories.header.frame_id = "base_link"
        local_trajectories.header.stamp = msg.header.stamp

        msg_header = msg.header

        for node, traj in preds.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = node.id
            marker.action = 0  # ADD
            marker.type = 4  # LINE_STRIP
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0
            marker.color.a = 1.0

            trajectory = Trajectory()
            local_trajectory = Trajectory()

            for state in traj[0][0]:
                p = Point()
                p.x = state[0]
                p.y = state[1]
                marker.points.append(p)
                trajectory.position.append(p)

                local_p = self.transformToLocal(p, msg_header)
                if local_p != None:
                    local_trajectory.position.append(local_p)

            trajectories.trajectories.append(trajectory)
            local_trajectories.trajectories.append(local_trajectory)
            markers.markers.append(marker)
        self.visual_pred_pub.publish(markers)
        self.pred_pub.publish(trajectories)
        self.local_pred_pub.publish(local_trajectories)

    def transformToLocal(self, point, header):
        local = PointStamped()
        local.header = header
        local.header.frame_id = "map"
        local.point.x, local.point.y = point.x, point.y
        temp = PointStamped()
        try:
            temp = self.listener.transformPoint("base_link", local)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        local_point = Point()
        local_point.x, local_point.y = temp.point.x, temp.point.y
        return local_point