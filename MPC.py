# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 22:46:34 2023

@author: user
"""
from cvxpy import *
import numpy as np
import cvxpy as cp

# define parameters

P = 10  # predictive horizion
th = 2.5  # time headway
TL = 1  # time constant -- GLVD
KL = 1  # system gain
Ts = 100*1e-3  # sampling time: 100ms

Phi = np.array([[0, 1, -th],
                [0, 0, -1],
                [0, 0, -1/TL]], dtype='float')
Pi = np.array([[0],
               [0],
               [KL/TL]], dtype='float')
Gam = np.array([[0],
                [1],
                [0]])

TTC = -3.0  # time to collision
d0 = 5.0  # stopping distance for typical truck drivers
d_s0 = 5.0  # minimum safe distance

a_safe = np.array([[1, -TTC-th, 0], [1, -th, 0], [0, 0, 0]], dtype='float')
d_safe = np.array([-d0, d_s0-d0, 0], dtype='float')
tau_safe = np.array([[-th], [-th], [0]], dtype='float')

kV, kD = 0.25, 0.02  # the coefficient used to calculate reference acceleration
wy = np.diag([5, 4, 0])  # weighting coefficient

Sigma = np.array([[-1, 0, 0], [0, -1, 0], [kD, -kV, 1]])  # correct??
wdeltau = 0.1
wu = 1.0

rho = 10.0  # coefficient for slack variable


# x:[rel_d,rel_v,af], u:afdes, v:ap(preceding vehicle's acceleration)
def x_next_f(x, u, v):

    return x+(Phi*Ts)@x+Ts*Pi@u+Ts*Gam@v


class MPC:
    def __init__(self):
        # distance error, relevant velocity, af
        self.x = Variable((3, P+1), name='State')
        self.u = Variable((1, P), name='control input')  # afdes

        # inital distance error, relevant velocity, af
        self.x_init = Parameter(3, name='Initial state')
        self.ap = Parameter(1, name='ap')  # preceding vehicle's acceleration
        self.vf = Parameter(1, name='vf')
        self.lastu = Parameter(1, name='last_u')

        self.slackX = Variable((3, P), nonneg=True,
                               name='slackX')  # add slack variable
        self.slackU = Variable((1, P), nonneg=True, name='slack_u')

        self.constraint = [self.x[:, 0] == self.x_init]
        self.cost = 0

        vf = self.vf
        for i in range(P):  # predictive horizon
            print(f"{i=}")

            self.cost = self.cost+wy[0, 0]*self.x[0, i+1]**2+wy[1, 1]*self.x[1, i+1]**2 + \
                wu*self.u[0, i]**2+wy[2,
                                      2]*(self.u[0, i]-self.x[2, i+1])**2+rho*cp.sum_squares(self.slackX[:, i])+rho*cp.sum_squares(self.slackU[:, i])

            if i > 0:
                delta_u = self.u[0, i]-self.u[0, i-1]
                self.cost += (wdeltau*delta_u**2)

                # control input increment
                self.constraint += [delta_u >=
                                    np.array([-0.01])-self.slackU[:, i], delta_u <= np.array([0.01])+self.slackU[:, i]]

            # model of car-folowing constraint: notice that preceding vehicle's acc??
            self.constraint += [self.x[:, i+1] >=
                                x_next_f(self.x[:, i], self.u[:, i], self.ap)-self.slackX[:, i], self.x[:, i+1] <=
                                x_next_f(self.x[:, i], self.u[:, i], self.ap)+self.slackX[:, i]]

            # system output contraint
            self.constraint += [self.x[:, i+1] >= np.array(
                [-5, -1, -4.0])-self.slackX[:, i], self.x[:, i+1] <= np.array([6, 0.9, 1.50])+self.slackX[:, i]]

            # control input constraint
            self.constraint += [self.u[0, i] >=
                                np.array([-4])-self.slackU[:, i], self.u[0, i] <= np.array([1.50])+self.slackU[:, i]]

            # rear-end safety constraint
            vf += self.u[0, i]*Ts
            self.constraint += [a_safe@self.x[:, i+1]
                                >= d_safe.T+tau_safe@vf]

    def problem(self):
        return Problem(Minimize(self.cost), constraints=self.constraint)

    def solve(self):
        return self.solve(solver=OSQP, warm_start=True)
