from RobotRaconteur.Client import *
from contextlib import suppress

c = RRN.ConnectService('rr+tcp://localhost:64238?service=scanner')

N = 100

scan_handles = []

for i in range(N):
    scan_handles.append(c.capture_deferred(False))


prepare_gen = c.deferred_capture_prepare(scan_handles)
with suppress(RR.StopIterationException):
    prepare_res = prepare_gen.Next()
    print(prepare_res)

for i in range(N):
    mesh = c.getf_deferred_capture(scan_handles[i])

    # Do something with the mesh
    print(len(mesh.triangles))
    print(len(mesh.vertices))
    


