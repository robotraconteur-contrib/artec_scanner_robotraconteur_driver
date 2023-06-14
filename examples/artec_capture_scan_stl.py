from RobotRaconteur.Client import *
import matplotlib.pyplot as plt

c = RRN.ConnectService('rr+tcp://localhost:64238?service=scanner')

tcp_transport.MaxMessageSize=int(100e6)

# capture() returns a Robot Raconteur Mesh 
# https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/0e1634e6cf547be9e50889ed281fe9892003a96e/group1/com.robotraconteur.geometry.shapes.robdef#L74
stl_mesh_bytes = c.capture_stl()

with open("captured_mesh.stl", "wb") as f:
    f.write(stl_mesh_bytes)