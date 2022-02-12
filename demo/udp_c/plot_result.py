#!/usr/bin/python
import matplotlib.pyplot as plt
import os

def create_figures():
	input_txt = os.getcwd() + '/actuator_data.txt'
	f = open(input_txt)

	num = []
	kp = []; q_des = []; q = []; kd = []; qd_des = []; qd = []; 
	tau_ff = []; tau_des = []; tau = []

	# read from 2nd line
	for line in f.readlines()[2:]:
		line = line.strip('\n')
		line = line.split(' ')

		num.append(int(line[0]))

		q_des.append(float(line[1]))
		q.append(float(line[2]))
		qd_des.append(float(line[3]))
		qd.append(float(line[4]))
		# tau_ff.append(float(line[5]))
		tau_des.append(float(line[5]))
		tau.append(float(line[6]))


	fig = plt.figure(figsize = (12,4))

	# plot q_cmd & q_data of abads
	ax1 = plt.subplot(1,3,1)
	ax1.plot(num, q_des, color='blue', label='q_des')
	ax1.plot(num, q, color='green', label='q')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint Position (rad)')
	ax1.grid(True)
	ax1.legend(loc='upper right')

	# plot q_cmd & q_data of hip
	ax2 = plt.subplot(1,3,2)
	ax2.plot(num, qd_des, color='blue', label='qd_des')
	ax2.plot(num, qd, color='green', label='qd')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint Velocity (rad/s)')
	ax2.grid(True)
	ax2.legend(loc='upper right')

	# plot torque of abad
	ax3 = plt.subplot(1,3,3)
	ax3.plot(num, tau_des, color='red', label='tau_des')
	# ax3.plot(num, tau_ff, color='orange', label='tau_ff')
	ax3.plot(num, tau, color='magenta', label='tau')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint Torque (N.m)')
	ax3.grid(True)
	ax3.legend(loc='upper right')

if __name__ == "__main__":
    create_figures()
    plt.show()
    # fig.savefig('data.png')
    # plt.close(fig)
