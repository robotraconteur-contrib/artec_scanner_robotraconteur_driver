from RobotRaconteur.Client import *
from contextlib import suppress

c = RRN.ConnectService('rr+tcp://localhost:64238?service=scanner')

N = 100

scan_handles = []

for i in range(N):
    scan_handles.append(c.capture_deferred(False))

prepare_gen = c.deferred_capture_prepare_stl(scan_handles)
with suppress(RR.StopIterationException):
    prepare_res = prepare_gen.Next()
    print(prepare_res)

for i in range(N):
    stl_mesh_bytes = c.getf_deferred_capture_stl(scan_handles[i])

    with open(f"deferred_captured_mesh_{i+1}.stl", "wb") as f:
        f.write(stl_mesh_bytes)
