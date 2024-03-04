# -*- coding: utf-8 -*-
"""
Created on Sat Nov 11 18:52:52 2023

@author: user
"""

from MPC import MPC

import numpy as np
from python_client import TCPclient
import matplotlib.pyplot as plt
import time
# Constants
TIME_HEADWAY = 2.0  # (s)
STOPPING_DISTANCE = 5.0  # (m)

FOLLOWING_MODE = 1
CRUISE_MODE = 0
ACCELERATION_MODE = 15
DECELERATION_MODE = 2
DRIVER_MODE = -1

TARGET_VELOCITY = 70.0/3.6  # (m/s), ~=22
# The range of target velocity : 60.0-120.0 kmh. Aware unit
MIN_ACC_OPEN_SPEED = 60.0/3.6  # (m/s), ~= 13.888
# The range of target velocity : 60.0-120.0 kmh. Aware unit
MAX_ACC_OPEN_SPEED = 120.0/3.6  # (m/s) ~=33
MAX_DISTANCE_ERROR = 10.0  # (m)

# should change: cruise and accele wrong


def ACC_mode_decision(status, distance, distance_error, velocity):
    if status:
        if distance_error <= 1 and distance != -1:
            # print(distance_error, velocity)
            return FOLLOWING_MODE
        elif TARGET_VELOCITY-velocity <= 1:
            return CRUISE_MODE
        else:
            return DRIVER_MODE
    else:
        return DRIVER_MODE


if __name__ == '__main__':
    
    time.sleep(2)
    
    client = TCPclient('127.0.0.1', 8001)

    mpc = MPC()
    mpc_problem = mpc.problem()

    total_f, total_c, success_f, success_c = 0, 0, 0, 0
    last_du, du = 0.5, 0.5  # initial acceleration
    dis_arr, rel_spd_arr, vf_arr, af_arr, ap_arr, mode_arr, du_arr = np.array(
        []), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
    u_ = np.array([])

    while True:
        try:
            # units: (m), (m/s), (m/s^2), (m/s), (m/s^2), 1/0
            dis, rel_spd, af, vf, ap, ACC_Status = client.receivemsg()
            dis_arr = np.append(dis_arr, dis)
            rel_spd_arr = np.append(rel_spd_arr, rel_spd)
            af_arr = np.append(af_arr, af)
            vf_arr = np.append(vf_arr, vf)
            ap_arr = np.append(ap_arr, ap)

            du_arr = np.append(du_arr, du)
        except Exception as e:
            print(f'Receive from simulink failed: {e}')

        distance_error = dis - (TIME_HEADWAY * vf + STOPPING_DISTANCE)
        mode = ACC_mode_decision(ACC_Status, dis, distance_error, vf)
        mode_arr = np.append(mode_arr, mode)

        # last_du = af

        # print(mode)
        if mode == FOLLOWING_MODE:
            total_f += 1
            print()
            mpc.x_init.value = np.array(
                [distance_error[0], rel_spd[0], af[0]]).T

            mpc.ap.value = np.array([ap]).flatten()
            mpc.vf.value = np.array([vf]).flatten()
            mpc.lastu.value = np.array(([last_du])).flatten()

            try:
                mpc_solution = mpc_problem.solve()
                mpc_status = mpc_problem.status

                if mpc_status != 'optimal':
                    print('Following failed')
                    du = last_du
                else:
                    du = mpc.u[0, 0].value
                    last_du = du.copy()
                    success_f += 1

            except Exception as e:
                du = last_du
                print(f'Error in following mode: {e}')

        elif mode == CRUISE_MODE:
            total_c += 1
            mpc.x_init.value = np.array(
                [0.0, ((TARGET_VELOCITY - vf)[0]), af[0]]).T
            # [[0.0], [(TARGET_VELOCITY - vf)[0]], [af[0]]]).flatten()
            mpc.vf.value = np.array([vf]).flatten()
            mpc.ap.value = np.array([0.0]).flatten()
            mpc.lastu.value = np.array([last_du]).flatten()

            try:
                cruise_solution = mpc_problem.solve()
                cruise_status = mpc_problem.status

                if cruise_status != 'optimal':
                    print('Cruise failed')
                    du = last_du
                else:
                    success_c += 1
                    du = mpc.u[0, 0].value
                    last_du = du.copy()

            except WindowsError as e:
                if e.winerror == 10053:
                    print(f'{e}')

            except Exception as e:
                du = last_du
                print(f'Error in cruise mode: {e}')
                pass
        else:
            du = last_du

        try:
            client.sendmsg(du*100)
            u_ = np.append(u_, du)
        except Exception as e:
            print(f'error: {e}')
            break

    # fig, ax = plt.subplots(3, 1, figsize=(12, 8))
    # ax[0].plot(np.linspace(0, 60, len(vf_arr)), vf_arr, label='vf')
    # ax[0].xaxis.set_visible(False)
    # ax[0].set_title('Velocity profile')
    # ax[0].set_ylabel('m/s')

    # ax[1].plot(np.linspace(0, 60, len(af_arr)), af_arr, label='Ax')
    # ax[1].xaxis.set_visible(False)
    # ax[1].set_title('acceleration profile')
    # ax[1].set_ylabel('m/s^2')

    # ax[2].plot(np.linspace(0, 60, len(af_arr[1:])),
    #            af_arr[1:]-af_arr[0:-1], label='Ax')
    # ax[2].set_title('acceleration increment')
    # ax[2].set_ylabel('m/s^2')
    # ax[2].set_xlabel('time(s)')

    # fig2, ax2 = plt.subplots(2, 1, figsize=(12, 5))
    # ax2[0].plot(np.linspace(0, 60, len(af_arr)), af_arr, label='Ax')
    # ax2[0].plot(np.linspace(0, 60, len(du_arr)), du_arr, label='ACC_AxCmd')
    # ax2[0].legend(loc='upper right')
    # ax2[0].set_ylabel('m/s^2')
    # ax2[0].xaxis.set_visible(False)

    # ax2[1].plot(np.linspace(0, 60, len(vf_arr)), vf_arr, label='Velocity')
    # ax2[1].legend(loc='upper right')
    # ax2[1].set_ylabel('m/s')
    # ax2[1].set_xlabel('time(s)')

    # fig3, ax3 = plt.subplots(3, 1, figsize=(12, 8))
    # ax3[0].plot(np.linspace(0, 60, len(mode_arr)), mode_arr)
    # ax3[0].xaxis.set_visible(False)
    # ax3[0].set_title(
    #     'ACC mode: -1: Driver mode, 0: Cruise mode, 1: following mode')

    # ax3[1].plot(np.linspace(0, 60, len(dis_arr)), dis_arr)
    # ax3[1].xaxis.set_visible(False)
    # ax3[1].set_title('Distance profile')
    # ax3[1].set_ylabel('m')

    # ax3[2].plot(np.linspace(0, 60, len(rel_spd_arr)), rel_spd_arr)
    # ax3[2].set_title('Relative velocity')
    # ax3[2].set_ylabel('m/s')
    # ax3[2].set_xlabel('time(s)')
