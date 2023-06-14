from RobotRaconteur.Client import *
import cv2
import matplotlib.pyplot as plt

c = RRN.ConnectService('rr+tcp://localhost:64238?service=scanner')

# capture() returns a Robot Raconteur Mesh 
# https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/0e1634e6cf547be9e50889ed281fe9892003a96e/group1/com.robotraconteur.geometry.shapes.robdef#L74
mesh = c.capture(True)

print(len(mesh.triangles))
print(len(mesh.vertices))
print(len(mesh.normals))
print(len(mesh.textures))

print(len(mesh.textures[0].image.data))
print(len(mesh.textures[0].uvs))

texture_opencv_img = cv2.imdecode(mesh.textures[0].image.data, cv2.IMREAD_COLOR)

points = RRN.NamedArrayToArray(mesh.vertices)

ax = plt.figure().add_subplot(projection='3d')
ax.scatter(points[:,0],points[:,1],points[:,2])

plt.figure()
plt.imshow(texture_opencv_img)
plt.show()

