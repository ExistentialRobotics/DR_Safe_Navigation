#!/usr/bin/env python3
""" Low level velocity conttoller for unicycle-like robot.

Interfaces:
    Input:
            desired robot states (z*) from ref_gvn node z* = zg
            current robot states (z) from odometry
    Output:
            velocity command for mobile platform or simulated dynamics

"""

from __future__ import print_function
import rospy
import numpy as np
import numpy.linalg as npla
import time
from erl_clf_cbf_controller.pre_process import ClfCbfPreprocess
from erl_clf_cbf_controller.post_process import ClfCbfPostprocess

import json
# different controller formulation
from erl_clf_cbf_controller.clf_cbf_controller import ClfCbfController
from erl_clf_cbf_controller.clf_only_controller import ClfQPController


from erl_clf_cbf_controller.clf_cbf_socp_controller_gp_map import ClfCbfSOCP_GP_MAP

from erl_clf_cbf_controller.clf_cbf_drccp_dynamic_controller import ClfCbfDrccp_dynamic_Controller

import csv
import os



def off_wheel_map(rbt_pose, off_wheel_length):
    """
    Remap robot pose from center of mass to off-wheel point

    """
    off_wheel_x = rbt_pose[0] + off_wheel_length * np.cos(rbt_pose[2])
    off_wheel_y = rbt_pose[1] + off_wheel_length * np.sin(rbt_pose[2])
    off_wheel_theta = rbt_pose[2]

    return np.array([off_wheel_x, off_wheel_y, off_wheel_theta])



class ClfCbfControllerWrapper:
    """
    CLF-CBF-QP controller wrapper, check system status
    """

    # Running status table, higher number better status
    NORMAL = 0
    GOAL_REACHED = 1

    def __init__(self, config_table=None):
        """ Init ClfCbfControllerWrapper class.
            This controller subscribes:
                odom (from localization)
                desired robot states (from high level controller, i.e., ref_)
            Publish:
                desired velocity / body twist
        """


        # function class handle
        self.preprocessor = None
        self.core = None
        self.postprocessor = None

        # loading external config parameters
        self._config_dict = config_table

        self.controller_type = rospy.get_param('~controller_type', 'drccp')  # Default to 'drccp' if not set
        self.load_controller_config()

        # system status
        self.status = ClfCbfControllerWrapper.NORMAL

        # -------------------- Constants -------------------------
        self.ctrl_limits = self._config_dict["ctrl_limits"]

        rospy.logwarn("self.ctrl_limits %s" % self.ctrl_limits)

        # ------------------- Init Modules --------------------
        self.init_preprocessor()
        self.init_postprocessor()


        # ------------------- Cache --------------------
        self.response_cache = []


        # ----------------- for plotting performance -------------

        self.v_list = []
        self.w_list = []
        self.distance_list = []

        self.error_to_gamma =  []

        self.robot_pos = []

        # ------------------ for plotting moving obstacles in dynamic env ----------------

        self.robot_started_moving = False
        self.mov_obs_flag = False
        self.mov_obs_data = []
        self.waypoints = []
        self.last_waypoint = None

        # set numpy array console print precision = 2
        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})
        rospy.loginfo("CLF-CBF CONTROL NODE INIT SUCCESSFUL!")

    def load_controller_config(self):
        # Load the configuration file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, 'controller_config.json')
        with open(config_path, 'r') as file:
            configs = json.load(file)

        # Determine which controller configuration to use
        controller_type = self.controller_type

        if controller_type not in configs:
            raise ValueError(f"Unknown controller configuration: {controller_type}")

        self.config = configs[controller_type]

        # Assigning new variables from the selected config
        params = self.config['parameters']
        self._wheel_offset = params.get('wheel_offset', 0.08)  # Default to 0.08 if not set
        self._cbf_rate = params.get('cbf_rate', 0.4)  # Default to 0.4 if not set
        self.noise_level = params.get('noise_level', 0.01)  # Default to 0.01 if not set

        # Initialize the controller based on the controller type
        self.init_core()

    def init_preprocessor(self):
        """
        Init preprocessor of node.
        """
        self.preprocessor = ClfCbfPreprocess()

    def save_rob_data_to_csv(self, filename):
        """Save collected data to a CSV file."""
        # Ensure directory exists
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, filename)

        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Linear Velocity', 'Angular Velocity', 'Distance to Obstacle', 'Tracking Error', 'X Position', 'Y Position'])

            for v, w, dist, tracking_error, pos in zip(self.v_list, self.w_list, self.distance_list, self.error_to_gamma, self.robot_pos):
                writer.writerow([v, w, dist, tracking_error, pos[0][0], pos[1][0]])

    def save_mov_obs_data_to_csv(self, filename="mov_obs_data.csv"):
        """Save collected moving obstacles data to a CSV file."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, filename)

        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['PosX', 'PosY', 'VelX', 'VelY', 'Radius'])
            for data in self.mov_obs_data:
                writer.writerow(data)


    def save_waypoints_to_csv(self, filename="waypoints.csv"):
        """Save collected waypoints to a CSV file."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, filename)

        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y', 'Theta'])
            for waypoint in self.waypoints:
                writer.writerow(waypoint)



    def init_core(self):
        """
        Init unicycle controller. For each controller the interface might be different.
        More details in controller itself.
        """
        # Initialize the controller based on the configuration
        controller_type = self.controller_type
        params = self.config['parameters']

        if controller_type == "baseline_clf_cbf_qp":
            self.core = ClfCbfController(**params)
        elif controller_type == "clf_qp_only":
            self.core = ClfQPController(**params)
        elif controller_type == "robust_cbf_socp":
            self.core = ClfCbf_Robust_SOCP_Controller(**params)
        elif controller_type == "gp_cbf_socp":
            self.core = ClfCbfSOCP_GP_MAP(**params)
        elif controller_type == "drccp":
            self.core = ClfCbfDrccp_dynamic_Controller(**params)
        else:
            raise ValueError("Unknown controller type specified.")


    def init_postprocessor(self):
        """
        Init preprocessor of node.
        """
        self.postprocessor = ClfCbfPostprocess(ctrl_limits=self.ctrl_limits)


    def update_waypoints(self):
        current_waypoint = self.preprocessor._np_z_dsr.tolist()
        if self.last_waypoint != current_waypoint:  # Check if the waypoint has changed
            self.waypoints.append(current_waypoint)
            self.last_waypoint = current_waypoint  # Update the last waypoint


    def update(self, eps_dist=0.25, eps_dist_reset=0.35, num_samples=5):
        """
        Update loop as follows:
            1. collect latest data from preprocessor (callback automatically)
            2. execuate update loop using core
            3. sending command to downstream via post-processor
        """

        # update the waypoint (currently, it is just local goal)
        self.update_waypoints()


        # receive robot pose from process
        z = off_wheel_map(rbt_pose=self.preprocessor._np_z, off_wheel_length=self._wheel_offset)
        rbt_xy = np.array([[self.preprocessor._np_z[0]], [self.preprocessor._np_z[1]]])

        # obtain local goal on path using simplified reference governorClfCbfControllerWrapper
        z_dsr = off_wheel_map(rbt_pose=self.preprocessor._np_z_dsr, off_wheel_length=self._wheel_offset)

        # obtain distance error
        dist_err = npla.norm(self.preprocessor._np_z[0:2] - z_dsr[0:2])

        # start with default zero control
        v, w = 0.0, 0.0

        # obtain cbf value and gradient from sdf server
        if self.preprocessor.use_sdf:
            start = time.time()
            while True:
                try:
                    response_temp = self.preprocessor.sdf_client([z[0]], [z[1]])
                    response_cache = self.response_cache
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn_throttle(0.5, f"Service call failed: {e}. Retrying...")
            end = time.time()
            print(f"ros sdf_client: {end - start} sec")

            if np.isnan(np.array([response_temp.sdf])):
                response = self.response_cache
            else:
                response = response_temp
                self.response_cache = response_temp

            cbf_value = np.array([response.sdf])
            cbf_grad = np.array([response.gradient_x, response.gradient_y])


            # print('gp_cbf:', cbf_value)
            # print('gp_cbf_grad:', cbf_grad)

        else:
            cbf_value = np.zeros((num_samples, ))
            cbf_grad = np.zeros((2, num_samples))
            partial_h_partial_t_static = np.zeros((num_samples, ))

            for index, item in enumerate(self.preprocessor.scan_buffer):
                dist_rbt2surface = np.linalg.norm(rbt_xy - item, axis=0)


                '''
                Adding lidar noise (naive way), it is highly recommended to modClfCbfControllerWrapperify the source urdf file instead
                '''
                # Adding Gaussian noise to each distance measurement

                # sigma_values = self.noise_level * dist_rbt2surface


                # noise = np.random.normal(0, sigma_values)
                # dist_rbt2surface = dist_rbt2surface + noise

                index_closest = np.argmin(dist_rbt2surface)
                if np.size(index_closest) > 1:
                    index_closest = index_closest[0]

                cbf_value[index] = dist_rbt2surface[index_closest]

                # Calculate the gradient
                gradient = rbt_xy[:, 0] - item[:, index_closest]

                # Normalize the gradient to have a 2-norm of 1
                norm_gradient = gradient / np.linalg.norm(gradient)
                cbf_grad[:, index] = norm_gradient

            # static obstacle cbf has no time change
            partial_h_partial_t = partial_h_partial_t_static

            # calculate cbf related parameters for moving obstacles
            if self.preprocessor.mov_obs:
                if self.preprocessor.mov_obs_ready:
                    self.mov_obs_flag = True

                    mov_obs_xy = np.vstack((self.preprocessor.mo_list[:, 0].T, self.preprocessor.mo_list[:, 1].T))
                    mov_obs_rad = self.preprocessor.mo_list[:, 4]
                    mov_obs_vel = np.vstack((self.preprocessor.mo_list[:, 2].T, self.preprocessor.mo_list[:, 3].T))

                    # if self.robot_started_moving:
                    # # saving data for visualization
                    #     for i in range(len(mov_obs_xy[0])):
                    #         self.mov_obs_data.append([mov_obs_xy[0][i], mov_obs_xy[1][i], mov_obs_vel[0][i], mov_obs_vel[1][i], mov_obs_rad[i]])



                    dist_rbt2mov = np.linalg.norm(rbt_xy - mov_obs_xy, axis=0) - mov_obs_rad
                    mov_obs_grad = rbt_xy - mov_obs_xy
                    norm_mov_gradient = mov_obs_grad / np.linalg.norm(mov_obs_grad, axis=0)

                    cbf_grad = np.concatenate((norm_mov_gradient, cbf_grad), axis=1)
                    cbf_value = np.concatenate((dist_rbt2mov, cbf_value))

                    # calculate the time change of cbf caused by moving obstacles
                    partial_h_partial_t_mov = (-1.0 * norm_mov_gradient.T @ mov_obs_vel).diagonal()
                    partial_h_partial_t = np.concatenate((partial_h_partial_t_mov, partial_h_partial_t))

            # obtain smallest five samples
            cbf_index = np.argsort((cbf_value - 0.31) * self._cbf_rate + partial_h_partial_t)[0:5]
            sorted_cbf_value = cbf_value[cbf_index]
            sorted_cbf_grad = cbf_grad[:, cbf_index]
            sorted_partial_h_partial_t = partial_h_partial_t[cbf_index]

            #print('sorted_dh_dt:', partial_h_partial_t_mov)

        min_index = np.argmin(cbf_value)  # Find the index of the minimum cbf_value

        # Set sdf_val as the min of cbf_value
        sdf_val = cbf_value[min_index]

        #   Set sdf_grad as the corresponding cbf_grad
        sdf_grad = cbf_grad[:, min_index]

        # send as ros msg to measure time
        self.postprocessor.send_sdf_val(sdf_val)

        # start with pose_reached status
        if self.status == ClfCbfControllerWrapper.GOAL_REACHED:
            if dist_err > eps_dist_reset:
                self.status = ClfCbfControllerWrapper.NORMAL
                msg = "[clf-cbf controller] status [down] [goal --> normal] triggered by [dist] err"
                msg += ": dist err > eps_dist_reset (%.3f > %.3f)" % (dist_err, eps_dist_reset)
                rospy.logwarn_throttle(0.5, msg)

        if self.status == ClfCbfControllerWrapper.NORMAL:
            if dist_err <= eps_dist:
                self.status = ClfCbfControllerWrapper.GOAL_REACHED
                msg = "[clf-cbf controller] status [ up ] [normal --> goal]"
                msg += ": dist err <= eps_dist (%.3f <= %.3f)" % (dist_err, eps_dist)
                rospy.logwarn_throttle(0.5, msg)

        # run status table
        if self.status == ClfCbfControllerWrapper.GOAL_REACHED:
            v, w = 0.0, 0.0
        elif self.status == ClfCbfControllerWrapper.NORMAL:
            # generate control value

            if self.controller_type == "baseline_clf_cbf_qp":
                # Example of how you might use the controller
                v, w = self.core.generate_controller(rbt_pose=z, gamma_s=z_dsr[0:2],
                                                     cbf_h_val=sdf_val, cbf_h_grad=sdf_grad)

            elif self.controller_type == "clf_qp_only":
                v, w = self.core.generate_controller(rbt_pose=z, gamma_s=z_dsr[0:2],
                                                  cbf_h_val=sdf_val, cbf_h_grad=sdf_grad)


            elif self.controller_type == "robust_cbf_socp":
                v, w = self.core.generate_controller(rbt_pose=z, gamma_s=z_dsr[0:2],
                                                  cbf_h_val=sdf_val, cbf_h_grad=sdf_grad,
                                                  h_error_bound = np.array([response.var_sdf]),
                                                  grad_h_error_bound = np.array([response.var_gradient_x, response.var_gradient_y]))

            elif self.controller_type == "gp_cbf_socp":
                v, w = self.core.generate_controller(rbt_pose=self.preprocessor._np_z, gamma_s=z_dsr[0:2],
                                                  cbf_h_val=sdf_val,
                                                  cbf_h_grad=sdf_grad,
                                                  cbf_h_variance = np.array([response.var_sdf]),
                                                  cbf_h_grad_var = np.array([response.var_gradient_x, response.var_gradient_y]))

            elif self.controller_type == "drccp":
                # Example for DRCCP controller
                v, w = self.core.generate_controller(rbt_pose=z, gamma_s=z_dsr[0:2],
                                                 h_samples=sorted_cbf_value, h_grad_samples=sorted_cbf_grad,
                                                 dh_dt_samples=sorted_partial_h_partial_t)
            else:
                raise ValueError(f"Unknown controller type: {self.controller_type}")

            # for plotting purposes

            # Check if the robot has started moving
            if abs(v) > 0.001 or abs(w) > 0.001:
                self.robot_started_moving = True

            if self.mov_obs_flag:
                for i in range(len(mov_obs_xy[0])):
                    self.mov_obs_data.append([mov_obs_xy[0][i], mov_obs_xy[1][i], mov_obs_vel[0][i], mov_obs_vel[1][i], mov_obs_rad[i]])

            self.v_list.append(v)
            self.w_list.append(w)
            if self.preprocessor.use_sdf:
                self.distance_list.append(sdf_val[0] - 0.31)
            else:
                self.distance_list.append(np.min(sorted_cbf_value) - 0.31)
            self.error_to_gamma.append(dist_err)
            self.robot_pos.append(rbt_xy)


        else:
            # reserve for more status
            pass

        self.postprocessor.send_cmd(v_dsr=v, w_dsr=w, clip_ctrl=True)
        self.postprocessor.send_debug(rbt_pose=z, gradient=sdf_grad)


if __name__ == '__main__':
    try:
        rospy.init_node('clf_cbf_controller')
        rospy.loginfo("[clf_cbf_controller] Started!\n")

        # loading parameters
        ctrl_freq = rospy.get_param("~ctrl_freq", 50.0)

        # control limit
        v_min = rospy.get_param("~v_min", -2.0)
        v_max = rospy.get_param("~v_max", 2.0)

        w_min = rospy.get_param("~w_min", 2.0)
        w_max = rospy.get_param("~w_max", -2.0)

        config_dict = {
            'ctrl_limits': {'v_min': v_min, 'v_max': v_max, 'w_min': w_min, 'w_max': w_max}
        }

        clf_cbf_controller = ClfCbfControllerWrapper(config_dict)
        rate = rospy.Rate(ctrl_freq)

        while not rospy.is_shutdown():
            if clf_cbf_controller.preprocessor._upstream_init_finish:
                clf_cbf_controller.update()
            else:
                pass
            rate.sleep()

    except rospy.ROSInterruptException:
        #clf_cbf_controller.save_rob_data_to_csv('saved_data/path2/test.csv')
        #clf_cbf_controller.save_mov_obs_data_to_csv('saved_data/dynamic/mov_obs_data.csv')
        #clf_cbf_controller.save_waypoints_to_csv('saved_data/dynamic/waypoints.csv')
        pass
