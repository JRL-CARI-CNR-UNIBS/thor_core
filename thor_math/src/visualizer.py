from curses import version
import numpy as np
import pandas as pd
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mg
import time

# ---- CONFIGURE YOUR PATHS HERE ----
print("Pinocchio version:", pin.__version__)
urdf_path = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10.urdf'
mesh_dir = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur_description/meshes'  # Folder that contains 'ur_description' package
traj_path = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/trajectory_log.csv'

# ---- LOAD TRAJECTORY LOG ----
df = pd.read_csv(traj_path)
q_cols = [c for c in df.columns if c.startswith('q')]
ph_cols = ['ph_x', 'ph_y', 'ph_z']

# ---- LOAD ROBOT MODEL (WITH GEOMETRY!) ----
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_path)
#     mesh_dir=mesh_dir
# )
print("Number of visual objects loaded:", len(visual_model.geometryObjects))  # Should be > 0

# ---- INITIALIZE MESHCAT VISUALIZER ----
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

# ---- ADD HUMAN SPHERE (RED) ----

viz.viewer['/human'].set_object(
    mg.Sphere(0.05),
    mg.MeshLambertMaterial(color=0xff0000)
)

# ---- MAIN ANIMATION LOOP ----
while True:
    for idx, row in df.iterrows():
        # Set robot configuration
        q = np.zeros(model.nq)
        q[:len(q_cols)] = row[q_cols].values.astype(float)
        viz.display(q)
        
        # Set human marker position
        ph = row[ph_cols].values.astype(float)
        T = pin.SE3(np.eye(3), ph)
        viz.viewer['/human'].set_transform(T.homogeneous)
        
        time.sleep(0.01)  # Adjust for speed
    print("Restarting animation from beginning.")
    time.sleep(0.5)
