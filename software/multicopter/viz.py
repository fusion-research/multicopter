"""
To install Mayavi graphics software on Ubuntu >=14.04:
    sudo apt-get install python-pip
    sudo apt-get install python-vtk
    sudo apt-get install libqt4-svg
    sudo pip install mayavi pyside

"""
from __future__ import division
import numpy as np; npl = np.linalg
from mayavi import mlab
from tvtk.tools import visual
import motion

# Send pointlessly spammed mayavi warnings to the shadow realm
import os, vtk
if os.path.exists("/dev/null"): shadow_realm = "/dev/null"
else: shadow_realm = "c:\\nul"
mlab_warning_output = vtk.vtkFileOutputWindow()
mlab_warning_output.SetFileName(shadow_realm)
vtk.vtkOutputWindow().SetInstance(mlab_warning_output)


class Viz(object):
    """
    Allows easy creation of a figure window with multicopter and environment visualization.

    model:            Model object for the multicopter
    building_layout:  array of booleans describing where buildings are in a grid
    building_size:    tuple of (length, width, height) that applies to all buildings
    building_spacing: single number for the distance between adjacent buildings
    fig:              the Mayavi figure to draw to, a new one is made if left None

    """
    def __init__(self, model, building_layout, building_size, building_spacing, fig=None):
        self.model = model
        self.building_layout = np.array(building_layout)
        self.building_size = building_size
        self.building_spacing = np.float64(building_spacing)
        if fig is None: self.fig = mlab.figure(size=(500, 500), bgcolor=(0.1, 0.1, 0.1))
        else: self.fig = fig

        # Set figure for visual objects
        visual.set_viewer(self.fig)

        # Convenient local aliases
        nx, ny = self.building_layout.shape
        n = nx * ny

        # Beautiful colors
        self.building_colors = map(tuple, np.array((np.linspace(0.0, 0.0, n),
                                                    np.linspace(0.8, 0.3, n),
                                                    np.linspace(0.3, 0.8, n))).T)

        # For storing buildings and their locations
        self.buildings = []
        self.building_centers = np.zeros((n, 2))

        # Generate buildings
        for i, x in enumerate(np.linspace(0, (nx-1)*(self.building_size[0] + self.building_spacing), nx)):
            for j, y in enumerate(np.linspace(0, (ny-1)*(self.building_size[1] + self.building_spacing), ny)):
                if not self.building_layout[i, j]: continue
                idx = int(ny*i + j)
                self.building_centers[idx] = (x, y)
                self.buildings.append(visual.box(x=x, y=y, z=self.building_size[2]/2, size=self.building_size, color=self.building_colors[idx]))

        # Generate ground plane
        if self.buildings:
            ground_xx, ground_yy = map(np.transpose, np.meshgrid(np.linspace(np.min(self.building_centers[:, 0]-10), np.max(self.building_centers[:, 0]+10), 2),
                                                                 np.linspace(np.min(self.building_centers[:, 1]-10), np.max(self.building_centers[:, 1]+10), 2)))
            self.ground = mlab.surf(ground_xx, ground_yy, np.zeros_like(ground_xx), color=(0, 0.2, 0.2), warp_scale=1)
        else:
            self.ground = None

        # Generate multicopter
        self.copter_radius = 1.1 * npl.norm(self.model.thruster_positions[0, :2])
        self.copter_sticks = np.vstack(([(0, 0, 0), (self.copter_radius, 0, 0)], self.model.thruster_positions, (self.copter_radius, 0, 0))).T
        self.copter_sticks_plot = mlab.plot3d(self.copter_sticks[0, :], self.copter_sticks[1, :], self.copter_sticks[2, :], line_width=5, color=(1, 0, 1))
        self.copter_nodes = np.vstack(([0, 0, 0], self.model.thruster_positions)).T
        self.copter_nodes_plot = mlab.points3d(self.copter_nodes[0, :], self.copter_nodes[1, :], self.copter_nodes[2, :], scale_factor=0.15)
        self.copter_nodes_plot.glyph.scale_mode = "scale_by_vector"
        self.copter_nodes_plot.mlab_source.dataset.point_data.scalars = [0, 1, 0.7, 0.7, 0.7, 0.7, 1, 0]  # hack to individually color points
        self.copter_vecs = [np.vstack((thr.position, thr.position+1e-5*thr.direction)).T for thr in self.model.thruster_list]
        self.copter_vecs_plots = [mlab.plot3d(vec[0, :], vec[1, :], vec[2, :], color=(1, 1, 1)) for vec in self.copter_vecs]

        # If using self-created figure, apply this initial viewing angle (must be done after initial generations)
        if fig is None: mlab.view(azimuth=180)

        # Aliases for Mayavi animate decorator and show function
        self.animate = mlab.animate
        self.show = mlab.show

    def update_multicopter(self, state, efforts, follow=None):
        """
        Redraws the multicopter in fig according to the given state and efforts.

        state:   State object for the current multicopter state
        efforts: dictionary keyed by thruster ID for the current multicopter efforts
        follow:  distance the camera will keep from the multicopter while following it (None for no following)

        """
        # Transform body geometry to world coordinates (using rotation matrix is faster for multiple points)
        R = motion.rotmat_from_quaternion(state.pose.ang)
        copter_nodes_world = state.pose.lin.reshape(3, 1) + R.dot(self.copter_nodes)
        copter_sticks_world = state.pose.lin.reshape(3, 1) + R.dot(self.copter_sticks)
        copter_vecs_world = [state.pose.lin.reshape(3, 1) + R.dot(vec) for vec in self.copter_vecs]

        # Update plot objects with new world coordinate information
        self.copter_nodes_plot.mlab_source.set(x=copter_nodes_world[0, :], y=copter_nodes_world[1, :], z=copter_nodes_world[2, :])
        self.copter_sticks_plot.mlab_source.set(x=copter_sticks_world[0, :], y=copter_sticks_world[1, :], z=copter_sticks_world[2, :])
        for i, vec_plot in enumerate(self.copter_vecs_plots):
            thr = self.model.thruster_list[i]
            self.copter_vecs[i][:, 1] = thr.position + (efforts[self.model.thruster_keys[i]] / thr.max_effort) * thr.direction
            vec_plot.mlab_source.set(x=copter_vecs_world[i][0, :], y=copter_vecs_world[i][1, :], z=copter_vecs_world[i][2, :])

        # Set camera view
        if follow: mlab.view(focalpoint=state.pose.lin.tolist(), distance=follow)
