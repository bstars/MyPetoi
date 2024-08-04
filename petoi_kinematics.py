import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import beta

class PetoiKinematics:
    def __init__(self, render_mode='3d') -> None:
        
        """ Static parameters """
        self.body_length = self.a = 10.5 # cm
        self.body_width = self.b = 10 # cm
        self.leg_length = self.c = 4.6 # cm
        self.foot_length = self.d = 4.6 # cm

        """ Dynamic parameters 
        Note that we make body angle and body height arrays to enable a smooth transition

        self.gamma is an interpolation between current angle and target angle
        """
        self.alphas = np.zeros([4]) # shoulder angles
        self.betas = np.zeros([4]) # knee angles
        self.gamma = np.array([0.]) # body angles
        self.gamma_granularity = np.deg2rad(2.) # increase/decrease body angle (roughly) 2deg each time
        self.h = np.array([0.]) # body vertial shift
        self.h_granularity = 0.1 # increase/decrease body height (roughly) 0.3cm each time
        self.T01_front = None
        self.T01_back = None
        self.T01_front_inv = None
        self.T01_back_inv = None

        """ Plotting initialization """
        self.render_mode = render_mode
        if self.render_mode == '3d':
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_box_aspect([1,1,0.5])
            self.ax.set_aspect('auto')
        elif self.render_mode == '2d':
            self.fig, self.ax = plt.subplots(1,1)
        else:
            self.fig, self.ax = None, None

        self.update_gamma_h(self.gamma[0], self.h[0])

    def update_gamma_h(self, gamma=None, h=None):
        """
        Update the target body angle and body height
        1. if gamma is given, replan the interpolation
        2. if gamma is not given, 
            2(a) if len(self.gamma)>1, select the next gamma in interpolation
            2(b) if len(self.gamma)=1, already at the target point, then don't update anything
        
        h is similar to gamma
        """

        if h is None:
            if len(self.h) == 1:
                pass
            else:
                self.h = self.h[1:]
        else:
            self.h = np.linspace(
                self.h[0], h, int(np.abs(h - self.h[0]) // self.h_granularity) + 1
            )
            if len(self.h) > 1: self.h = self.h[1:]

        if gamma is None:
            if len(self.gamma) == 1: 
                return
            else:
                self.gamma = self.gamma[1:]
                
        else:
            self.gamma = np.linspace(
                self.gamma[0], gamma, int(np.abs(gamma - self.gamma[0]) // self.gamma_granularity) + 1
            )
            if len(self.gamma) > 1: self.gamma = self.gamma[1:]


        # update the transformation matrix
        cg, sg = np.cos(self.gamma[0]), np.sin(self.gamma[0])
        self.T01_front = np.array([
            [cg, sg, -self.a/2*cg],
            [-sg, cg, self.a/2*sg],
            [0,  0,  1.]
        ])
        self.T01_back = np.array([
            [cg, sg,   self.a/2*cg],
            [-sg, cg, -self.a/2*sg],
            [0,  0,  1.]
        ])

        self.T01_front_inv = np.linalg.inv(self.T01_front)
        self.T01_back_inv = np.linalg.inv(self.T01_back)
    
    def render(self, alphas, betas):
        if self.render_mode != '3d' and self.render_mode != '2d':
            return
        plt.cla()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None]
        )

        """ plot legs """
        for i in range(4):
            ca, sa = np.cos(alphas[i]), np.sin(alphas[i])
            T01 = self.T01_front if i in [0,3] else self.T01_back
            y = -self.b/2 if i in [0,1] else self.b/2
            T02 = T01 @ np.array([
                [ca, -sa, self.c * sa],
                [sa,  ca, -self.c * ca],
                [0,    0, 1]
            ])
            foot = T02 @ np.array([
                -self.d * np.cos(betas[i]),
                self.d * np.sin(betas[i]),
                1
            ])

            if self.render_mode == '3d':
                self.ax.plot(
                    # shoulder  knee        foot
                    [T01[0, 2], T02[0, 2], foot[0]],    # x
                    [y,         y,          y],         # y
                    [T01[1, 2], T02[1, 2], foot[1]],    # z
                    linewidth=3
                )  
            elif self.render_mode == '2d':
                self.ax.plot(
                    # shoulder  knee        foot
                    [T01[0, 2], T02[0, 2], foot[0]], # x             
                    [T01[1, 2], T02[1, 2], foot[1]], # z
                    linewidth=3
                ) 
            else:
                pass
        # plot body
        """ plot body
        In the 1st version of the code, this comes before the leg plotting, 
        but then legs will cover the plot of body, so I put this after the leg plotting
        """
        if self.render_mode == '3d':
            self.ax.plot(
                [self.T01_front[0,2], self.T01_back[0,2]],
                [0, 0],
                [self.T01_front[1,2], self.T01_back[1,2]],
                'b-', linewidth=3
            )
            self.ax.plot(
                [self.T01_front[0,2], self.T01_front[0,2]],
                [-self.b/2, self.b/2],
                [self.T01_front[1,2], self.T01_front[1,2]],
                'b-', linewidth=3
            )

            self.ax.plot(
                [self.T01_back[0,2], self.T01_back[0,2]],
                [-self.b/2, self.b/2],
                [self.T01_back[1,2], self.T01_back[1,2]],
                'b-', linewidth=3
            )
            self.ax.scatter(0, 0, 0)

        elif self.render_mode == '2d':
            self.ax.plot(
                np.array([self.T01_front[0,2], self.T01_back[0,2]]),
                np.array([self.T01_front[1,2], self.T01_back[1,2]]),
                'b-', linewidth=3
            )
            self.ax.scatter(0,0) 
        
        else:
            pass

        if self.render_mode == '3d':
            self.ax.set_xlim3d([-10, 10])
            self.ax.set_ylim3d([-5, 5])
            self.ax.set_zlim3d([-9, 0])
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            self.ax.set_aspect("auto")
            plt.pause(0.0001)
        elif self.render_mode == '2d':
            self.ax.set_xlim([-10, 10])
            self.ax.set_ylim([-9, 5])
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_aspect("equal")
            plt.pause(0.001)


    def leg_ik(self, xz:np.array):
        """
        compute the inverse kinematics for all 4 legs
        :param xz:
            np.array with shape [4,2]
            each row is the xz coordinate of the end effector wrt the body frame
        :return
            alphas: shoulder angles, np.array with shape [4,]
            betas: knee angles, np.array with shape [4,]
        """
        xz0 = np.concatenate([xz, np.ones([4, 1])], axis=1) # [4, 3]
        xz0[:,1] -= self.h[0]
        alphas = []
        betas = []

        for i in range(4):
            T01_inv = self.T01_front_inv if i in [0,3] else self.T01_back_inv
            xz1 = T01_inv @ xz0[i]
            x1, z1 = xz1[0], xz1[1]

            L = np.sqrt(x1**2 + z1**2)
            alpha1_tilde = np.arccos((self.c**2 + L**2 - self.d**2) / (2 * self.c * L))
            alpha2_tilde = np.arcsin(x1/L)
            alphas.append(alpha1_tilde + alpha2_tilde)

            beta_tilde = np.arccos((self.c**2 + self.d**2 - L**2) / (2 * self.c * self.d))
            betas.append(np.pi/2 - beta_tilde)

        return np.array(alphas), np.array(betas)


standing_gait = np.array([
    [-5., -5],
    [5, -5],
    [5, -5],
    [-5, -5]
])


if __name__ == '__main__':
    dog = PetoiKinematics(render_mode='2d')
    # dog.update_gamma_h(gamma=np.pi/6, h=1)

    # print(dog.gamma)
    while True:
        alphas, betas = dog.leg_ik(standing_gait)
        dog.render(alphas, betas)
        dog.update_gamma_h()
        print(np.rad2deg(alphas))
        print(np.rad2deg(betas))
        print()
