#!/usr/bin/python
# plot results from "client_arm.c"

import matplotlib.pyplot as plt
import os

def create_figures():
	input_txt = os.getcwd() + '/actuator_data.txt'
	f = open(input_txt)

	num = []
	q_des_A = []; q_data_A = []; qd_des_A = []; qd_data_A = []; tau_des_A = []; tau_data_A = []
	q_des_B = []; q_data_B = []; qd_des_B = []; qd_data_B = []; tau_des_B = []; tau_data_B = []
	q_des_C = []; q_data_C = []; qd_des_C = []; qd_data_C = []; tau_des_C = []; tau_data_C = []

	# read from 2nd line
	for line in f.readlines()[2:]:
		line = line.strip('\n')
		line = line.split(' ')

		num.append(int(line[0]))

		q_des_A.append(float(line[1]))
		q_data_A.append(float(line[2]))
		qd_des_A.append(float(line[3]))
		qd_data_A.append(float(line[4]))
		tau_des_A.append(float(line[5]))
		tau_data_A.append(float(line[6]))

		q_des_B.append(float(line[7]))
		q_data_B.append(float(line[8]))
		qd_des_B.append(float(line[9]))
		qd_data_B.append(float(line[10]))
		tau_des_B.append(float(line[11]))
		tau_data_B.append(float(line[12]))

		q_des_C.append(float(line[13]))
		q_data_C.append(float(line[14]))
		qd_des_C.append(float(line[15]))
		qd_data_C.append(float(line[16]))
		tau_des_C.append(float(line[17]))
		tau_data_C.append(float(line[18]))

	fig = plt.figure(figsize = (12,8))

	# plot q_cmd & q_data of joint A
	ax1 = plt.subplot(2,3,1)
	ax1.plot(num, q_des_A, color='blue', label='q_des_A')
	ax1.plot(num, q_data_A, color='green', label='q_data_A')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint A Position (rad)')
	ax1.grid(True)
	ax1.legend(loc='upper right')

	# plot q_cmd & q_data of joint B
	ax2 = plt.subplot(2,3,2)
	ax2.plot(num, q_des_B, color='blue', label='q_des_B')
	ax2.plot(num, q_data_B, color='green', label='q_data_B')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint B Position (rad)')
	ax2.grid(True)
	ax2.legend(loc='upper right')

	# plot q_cmd & q_data of joint C
	ax3 = plt.subplot(2,3,3)
	ax3.plot(num, q_des_C, color='blue', label='q_des_C')
	ax3.plot(num, q_data_C, color='green', label='q_data_C')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint C Position (rad)')
	ax3.grid(True)
	ax3.legend(loc='upper right')

	# plot torque of joint A
	ax4 = plt.subplot(2,3,4)
	ax4.plot(num, tau_des_A, color='red', label='tau_des_A')
	ax4.plot(num, tau_data_A, color='magenta', label='tau_data_A')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint A Torque (N.m)')
	ax4.grid(True)
	ax4.legend(loc='upper right')

	# plot torque of joint B
	ax5 = plt.subplot(2,3,5)
	ax5.plot(num, tau_des_B, color='red', label='tau_des_B')
	ax5.plot(num, tau_data_B, color='magenta', label='tau_data_B')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint B Torque (N.m)')
	ax5.grid(True)
	ax5.legend(loc='upper right')

	# plot torque of joint B
	ax6 = plt.subplot(2,3,6)
	ax6.plot(num, tau_des_C, color='red', label='tau_des_C')
	ax6.plot(num, tau_data_C, color='magenta', label='tau_data_C')
	plt.xlabel('time (ms)')
	plt.ylabel('Joint C Torque (N.m)')
	ax6.grid(True)
	ax6.legend(loc='upper right')

if __name__ == "__main__":
    create_figures()
    plt.show()
    # fig.savefig('data.png')
    # plt.close(fig)
