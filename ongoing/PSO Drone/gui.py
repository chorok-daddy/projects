import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import matplotlib.animation as animation
import sys

class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads, func_num):
        self.fignum = 0
        self.quads = quads
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-50.0, 50.0])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylim3d([-50.0, 50.0])
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlim3d([0, 50.0])
        self.ax.set_zlabel('Z (m)')
        if func_num==0:
            self.ax.set_title('[Rosen Brock Function Simulation]')
            self.txt = self.ax.text2D(1, 1, '')
            self.ground_truth = self.ax.scatter(1, 1, 1,'-',c='red', marker='s')
            self.current_best = self.ax.scatter(0, -100, 0,'-',c='blue',marker='s')
            self.ax.legend([self.ground_truth, self.current_best], ['Ground truth', 'Current best-fit'])
        elif func_num==1:
            self.ax.set_title('[Air Pollutant Search Simulation]')
            self.txt = self.ax.text2D(1, 1, '')
            self.txt_wind_dir = self.ax.text(3,0,0, 'Wind (3.0 m/s) → → →', (1,0,0.04), color='black', fontsize='8')
            self.chimney, = self.ax.plot([0,0],[0,0],[0,8],color='gray',linewidth=3, antialiased=False)
            self.ground_truth = self.ax.scatter(0, 0, 10,'-',c='red', marker='s')
            self.current_best = self.ax.scatter(0, -100, 0,'-',c='blue',marker='s')
            self.ax.legend([self.chimney, self.ground_truth, self.current_best], ['Chimney', 'Ground truth', 'Current best-fit'])
        self.init_plot()

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def init_plot(self):
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],[],color='black',linewidth=1,antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],[],color='black',linewidth=1,antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=2,antialiased=False)
            #self.ax.scatter(40, 40, 40,'-',c='red',marker='o')
            #self.ax.scatter(20, 20, 20,'-',c='red',marker='o')
            #self.ax.scatter(20, 0, 20,'-',c='red',marker='o')
            #0.8,0.8,0.8),(0.4,0.4,0.4),(0.4,0.0,0.4),(0.2,-0.2,0.6)

    def update(self, args=''):
        for key in self.quads:
            R = self.rotation_matrix(self.quads[key]['orientation'])
            L = self.quads[key]['L']/2.0
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.quads[key]['position'][0]
            points[1,:] += self.quads[key]['position'][1]
            points[2,:] += self.quads[key]['position'][2]
            points = points * 10.0

            self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            self.quads[key]['l1'].set_3d_properties(points[2,0:2])
            self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            self.quads[key]['l2'].set_3d_properties(points[2,2:4])
            self.quads[key]['hub'].set_data(points[0,5],points[1,5])
            self.quads[key]['hub'].set_3d_properties(points[2,5])
        self.fignum+=1
        self.fig.savefig('./figs/fig'+str(self.fignum)+'.png', dpi=240)
        #plt.pause(0.000000000000001)

    def update_text(self, text, x, y, color):
        self.txt.remove()
        self.txt = self.ax.text2D(x, y, text, color=color, fontweight='bold', transform=self.ax.transAxes)

    def update_gb_marker(self, x, y, z):
        self.current_best.remove()
        self.current_best = self.ax.scatter(x, y, z,'-',c='blue',marker='s')

    def final_report(self, text, x, y, color):
        self.report1 = self.ax.text2D(0.4, 0.8, 'Simulation done', color='black', fontweight='bold', transform=self.ax.transAxes)
        self.report2 = self.ax.text2D(x, y, text, color=color, fontweight='bold', transform=self.ax.transAxes)
        for key in self.quads:
            self.quads[key]['l1'].remove()
            self.quads[key]['l2'].remove()
            self.quads[key]['hub'].remove()
        self.fignum+=1
        self.fig.savefig('./figs/fig'+str(self.fignum)+'.png', dpi=240)

