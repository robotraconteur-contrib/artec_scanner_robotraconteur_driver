# Artec Scanner SDK

This repository contains a Robot Raconteur for Artec 3D scanners, based on the Artec SDK 2.0 and developed in C++.
The driver has been tested for use with the Artec Space Spider scanner, but it may also work with other scanners
supported by the Artec SDK. The driver supports scanning capture, scanning procedure, and processing algorithms
included in the Artec SDK.

Robot Raconteur is a powerful communication framework, with support for many languages and platforms. Supported
programming languages include Python, C++, C#, Java, Matlab, and LabView. Supported platforms include Windows, Linux, and MacOS.
Using Robot Raconteur allows the Artec scanner to be used by many programming languages and platforms.

## Building

Building has only been tested on Windows 10. Visual Studio 2019+ with the C++ desktop development workload must be installed.

The Artec SDK 2.0 must be installed. The Artec SDK 2.0 is available for download from the Artec website at
http://docs.artec-group.com/sdk/2.0/ . The Artec SDK 2.0 is not open source and is not included
in this repository. It should be installed to the default location `C:\Program Files\Artec\Artec 3D Scanning SDK`.

Clone the repository:

```
git clone https://github.com/johnwason/artec_scanner_robotraconteur_driver.git
```

Dependencies for building the driver must be installed using `vcpkg`:

```
cd artec_scanner_robotraconteur_driver
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
git clone https://github.com/robotraconteur/vcpkg-robotraconteur.git
vcpkg install --triplet=x64-windows-release --overlay-ports=vcpkg-robotraconteur\ports robotraconteur robotraconteur_companion
cd ..
```

Now, the driver can be built using CMake:

```
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=vcpkg\scripts\buildsystems\vcpkg.cmake
cmake --build build --config Release
```

Note that Debug configuration cannot be built because the Artec SDK does not provide Debug dlls. Use RelWithDebInfo
if debugging is required.

At this point, the driver is built and ready to use.

## Running the driver

The DLLs for the Artec SDK must be in the PATH. The easiest way to do this is to copy the DLLs from the Artec SDK
installation directory to the build directory. For example:

```
cd build
copy "C:\Program Files\Artec\Artec 3D Scanning SDK\bin\*.dll" .
```

The driver can now be run:

```
artec_scanner_robotraconteur_driver.exe --project-save-path=. --robotraconteur-jumbo-message=true
```

The driver will connect to the first Artec scanner it finds. The scanner must be connected to the computer via USB.
The scanner must be powered on and the Artec Studio software must not be running.

By default, the driver can be connected using the following url: `rr+tcp://localhost:64238?service=scanner`

The standard Robot Raconteur command line configuration flags are supported. See
 https://github.com/robotraconteur/robotraconteur/wiki/Command-Line-Options

## Using the driver

The driver exposes a custom `experimental.artec_scanner.ArtecScanner` object. The definitions for the scanner
object and the associated data types are in `robdef/experimental.artec_scanner.robdef`. The scanner service
is quite sophisticated, and exposes much of the underlying Artec Scanner SDK functionality. See the Artec SDK 2.0
documentation for more information on the various algorithms and interfaces.

### Simple Capture

A single scan can be captured returning a Robot Raconteur standard `com.robotraconteur.geometry.shapes.Mesh` structure:

```python
from RobotRaconteur.Client import *
import matplotlib.pyplot as plt

c = RRN.ConnectService('rr+tcp://localhost:64238?service=scanner')
mesh = c.capture(True)
```

The mesh contains information about the mesh, including `mesh.triangles` and `mesh.vertices`. The mesh may also have a
texture stored in the `mesh.texture` field. See the example `examples/artec_capture_scan.py` for a complete example
with plotting.

In most cases, a common mesh file format is desirable instead of a Robot Raconteur mesh structure. The function
`capture_stl()` can be used to capture a scan and return the bytes contents of a mesh file.

```python
from RobotRaconteur.Client import *
import matplotlib.pyplot as plt

c = RRN.ConnectService('rr+tcp://localhost:64238?service=scanner')

# Mesh files may be larger than 10 MB, increase the max message size
tcp_transport.MaxMessageSize=int(100e6)
stl_mesh_bytes = c.capture_stl()

with open("captured_mesh.stl", "wb") as f:
    f.write(stl_mesh_bytes)
```

See `examples/artec_capture_scan_stl.py` for a complete example.

### Simple Multi-capture

At times it may be required to capture multiple scans, and return each scan individually as a 
`com.robotraconteur.geometry.shapes.Mesh` structure or as mesh file bytes. When using a robot, it is also necessary
to synchronously capture the scans so they can be aligned with the robot. The driver has the concept of a "deferred"
capture, where the scan is captured, but not processed until all scans have been captured. The processing can take
some time, and can slow down the rate of the frame capture. Once all frames are captured, they are processed in
parallel if multi-core cpus are available, making the processing quite fast.

The example `examples/artec_multi_capture_scan.py` and `examples/artec_multi_capture_scan_str.py` demonstrates 
capturing multiple scans using deferred capture for mesh structures and mesh file bytes.

### Scanning Procedure

The Artec SDK supports a scanning procedure that can be used to capture multiple scans at a high framerate. The
scanning procedure is started, and the scanner is moved around the object, and stopped when a large number of
scans have been captured. The Artec SDK 2.0 provides a number of algorithms that can be used to process
the captured scans into a single mesh.

See `examples/artec_scanning_procedure.py` for a complete example of capturing a scanning procedure and saving to
file as an Artec Studio project.

### Processing Algorithms

The Artec SDK 2.0 provides a number of algorithms that can be used to process the captured scans into a single mesh.
To be executed by the driver, the algorithms are configured into a pipeline by creating a list of algorithm
configurations structures. Default configuration structures for algorithms are returned by the `initialize_algorithm()`
driver function. The default configuration structures can be modified as desired. The following algorithm configuration
types are supported:

* `AutoAlignAlgorithm`
* `FastFusionAlgorithm`
* `FastMeshSimplificationAlgorithm`
* `GlobalRegistrationAlgorithm`
* `LoopClosureAlgorithm`
* `MeshSimplificationAlgorithm`
* `OutliersRemovalAlgorithm`
* `PoissonFusionAlgorithm`
* `SerialRegistrationAlgorithm`
* `SmallObjectsFilterAlgorithm`
* `TexturizationAlgorithm`

See `examples/artec_run_algorithms.py` for a complete example of running algorithms on a previously saved
project file, and saving a new mesh file with results. Note that it is possible to run algorithms directly
on a captured scanning procedure, but it is recommended that the scanning procedure be saved to a project
file first, and the algorithms run on the project file. This allows the algorithms to be run multiple times
with different parameters without having to re-capture the scanning procedure.

## License

Apache 2.0