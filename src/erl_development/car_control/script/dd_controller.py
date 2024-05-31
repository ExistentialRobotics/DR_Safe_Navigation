
import numpy as np
import matplotlib.pyplot as plt
import pdb

def wrap_angle(angle):
	angle += np.pi
	return angle % (2 * np.pi) - np.pi

def compute_control(state, desired_state, k, vmax=1, wmax=2):
	x = state[0]
	y = state[1]
	th = state[2]
	des_x = desired_state[0]
	des_y = desired_state[1]
	des_th = desired_state[2]

	# State Differences
	dx = des_x - x
	dy = des_y - y
	dth = th - des_th

	# Gains
	kp = k[0]
	ka = k[1]
	kb = k[2]

	# Compute Alternate States
	rho = np.sqrt(dx ** 2 + dy ** 2)
	alpha = wrap_angle(np.arctan2(dy, dx) - th)

	beta = wrap_angle(-dth - alpha)

	# Control Inputs
	v = kp * rho
	w = ka * alpha + kb * beta

	# Clip Control Inputs
	v = np.clip(v, 0.0, vmax)
	w = np.clip(w, -wmax, wmax)
	return v, w, np.array([rho, alpha, beta])


def apply_control(state, control, dt):
	x = state[0]
	y = state[1]
	th = state[2]
	v = control[0]
	w = control[1]

	# Apply DD Motion Model
	x += v * np.cos(th) * dt
	y += v * np.sin(th) * dt
	th = wrap_angle(th + w * dt)

	return np.array([x, y, th])

if __name__ == "__main__":
	# Initialize Start and Goal waypoints
	start = [-.5, -.5, np.pi/2]
	goal = [0, 0, 0]
	k = np.array([3, 8, -1.5])
	T = 10 # Time to reach waypoint
	discretization = 1000
	# Assign Vectors to collect states
	state_vec = np.empty((3, discretization))
	state_transform_vec = np.empty((3, discretization))

	# Set Start state
	state = start
	t_vec = np.linspace(0, T, discretization)

	for i in range(0, discretization):
		t = t_vec[i]
		v, w, newstate = compute_control(state, goal, k)
		print(f'State: {state} \nTransformState {newstate} \
			\nControl: {v, w} \nTime {t}')
		state = apply_control(state, [v, w], T / discretization)

		# Append Outputs
		state_vec[:, i] = state
		state_transform_vec[:, i] = newstate

	# Generate Error Plots
	fig, axes = plt.subplots(3, 2, num=1, figsize=(8,8))
	# X, Y, Yaw Error
	axes[0, 0].scatter(t_vec, state_vec[0,:], marker='.')
	axes[1, 0].scatter(t_vec, state_vec[1,:], marker='.')
	axes[2, 0].scatter(t_vec, state_vec[2,:], marker='.')
	# Rho, Alpha, Beta Error
	axes[0, 1].scatter(t_vec, state_transform_vec[0,:], marker='.')
	axes[1, 1].scatter(t_vec, state_transform_vec[1,:], marker='.')
	axes[2, 1].scatter(t_vec, state_transform_vec[2,:], marker='.')
	# Plot Titles
	axes[0, 0].set_title('X error (m)')
	axes[1, 0].set_title('Y error (m)')
	axes[2, 0].set_title('Yaw error (m)')

	axes[0, 1].set_title('Rho error (m)')
	axes[1, 1].set_title('Alpha error (m)')
	axes[2, 1].set_title('Beta error (m)')

	# Generate Main Plot
	fig= plt.figure(2, figsize=(4, 4))
	ax = fig.gca()
	ax.scatter(state_vec[0,:], state_vec[1,:], marker='.')
	ax.scatter(start[0], start[1], color='Green') # Plot Start
	ax.scatter(goal[0], goal[1], color='Red') # Plot Goal
	ax.set_title('Main Plot')
	ax.set_xlabel('X (m)')
	ax.set_ylabel('Y (m)')
	ax.set_xlim([-1, 1])
	ax.set_ylim([-1, 1])

	# Print Error
	X_err = state[0]
	Y_err = state[1]
	Th_err = state[2]

	print(f'X Error: {X_err} \nY error {Y_err} \nTh error {Th_err}')
	plt.show()


