# Original build environment reconstruction

## Scope and conclusion

This reconstruction tested the two repository packages without changing their
source, `CMakeLists.txt`, or `package.xml` files. The copies placed in the clean
workspace were verified with `diff -qr` against commit `ca188dd`; both comparisons
were empty.

Both packages build successfully on Ubuntu 18.04 with ROS Melodic, GCC 7, OpenCV
3.2, and August 2020 revisions of the required MRS packages. No project source
change is required to compile them on that stack. Two build dependencies missing
from the historical `mrs_lib` manifest must be supplied by the environment:
`tf2_eigen` and `tf_conversions`.

The result is a compatible historical reconstruction, not a bit-for-bit replay
of an August 2020 machine. The available official ROS image was created on 17
November 2020 and uses the immutable Melodic `final` package repository; some ROS
binary package build stamps are therefore from 2022 or 2023. Source dependencies
were selected from their actual August 2020 Git history.

## Evidence for the baseline

- The thesis front matter is dated August 2020.
- The thesis links to the Melodic `nav_msgs/OccupancyGrid` API.
- The MRS UAV System README at the selected revision explicitly identifies ROS
  Melodic and Ubuntu 18.04 Bionic as its supported pairing.
- Ubuntu 18.04 supplies OpenCV 3.2 and GCC 7, matching the APIs and C++17 mode
  used by the preserved project.

Ubuntu 18.04 with ROS Melodic is therefore the strongest supported baseline.
The thesis does not explicitly name Ubuntu or record exact compiler, OpenCV, or
MRS commit versions.

## Selected source revisions

The cutoff was `2020-09-01T00:00:00Z`. Revisions were selected from the relevant
ROS 1 branch, not merely from the newest commit anywhere in each repository.

| Component | Branch | Revision | Commit date | Reason |
| --- | --- | --- | --- | --- |
| MRS UAV System | `master` | `35c84bcde45bd4a405f5dc454a09ef2a75564739` | 2020-08-19 | Last pre-September commit on the historical ROS 1 integration branch; its README confirms Bionic/Melodic. |
| `mrs_lib` 0.0.5 | `master` | `3baa3b6f3446c08128d4f68b7a6a440d88b9048f` | 2020-08-31 | Last pre-September commit on the ROS 1 branch. |
| `mrs_msgs` 0.0.5 | `master` | `2664a96b6bdb032fc2d9a4c64b4f28b6593eb135` | 2020-08-14 | Last pre-September commit on `master`. A newer 16 August commit exists only on the separate `aerial_core` branch and was not selected. |
| Hector SLAM / `hector_map_tools` 0.4.1 | `melodic-devel` | `3f63834181ea0078cbecf0030ae68ef67ea8a78b` | 2020-05-15 | Tip applicable to Melodic at the cutoff. The later July/August commits belong to Noetic development. |

The MRS UAV System meta-repository itself is evidence for the platform baseline;
it is not a Catkin package needed to compile these two packages. The build used
the pinned `mrs_lib`, `mrs_msgs`, and `hector_map_tools` source trees directly.

## Reconstructed toolchain

The isolated build used the official image
`docker.io/library/ros:melodic-ros-base-bionic`, local image ID
`cfbe13bc69f07b0ab80624cfd51ad7272d699b569dcf96088f3cda2e9646b279`.
The pulled manifest digest was
`sha256:b4366ca0536e2ad3654ed66fb4475bc43b7a0c15bd8a5be91fd5ac9dfaf935d0`.

Confirmed versions inside the successful container:

| Component | Confirmed version |
| --- | --- |
| Ubuntu | 18.04.6 LTS (Bionic) |
| ROS | Melodic; `ros-melodic-ros-base` 1.4.1 |
| GCC/G++ | 7.5.0 |
| CMake | 3.10.2 |
| Catkin tools | 0.6.1 |
| Python used by Catkin | 2.7.17 |
| OpenCV | 3.2.0 (`libopencv-dev` 3.2.0+dfsg-4ubuntu0.1) |
| Eigen | 3.3.4 |
| `cv_bridge` | 1.13.1 |

The exact installed ROS binary revisions include later rebuild stamps because the
official image points to the Melodic `final` snapshot:

- `ros-melodic-ros-base` 1.4.1-0bionic.20230215.205719
- `ros-melodic-cv-bridge` 1.13.1-1bionic.20221025.190044
- `ros-melodic-tf2-eigen` 0.6.5-0bionic.20221025.181036
- `ros-melodic-tf-conversions` 1.12.1-1bionic.20221025.203221

These timestamps limit claims about exact August 2020 binary reproducibility,
but do not change the Bionic/Melodic API baseline demonstrated by the build.

## Build procedure

The build ran in a rootless Podman container. The clean workspace contained only:

- the repository's `occupancy_grid_merger` and `relative_pose_estimator` trees;
- `mrs_lib` and `mrs_msgs` at the revisions above; and
- only `hector_map_tools` from the pinned Hector SLAM tree.

Dependencies were first resolved with:

```bash
rosdep install --from-paths /ws/src --ignore-src --rosdistro melodic -r -y
```

Then the unchanged sources were built with:

```bash
source /opt/ros/melodic/setup.bash
catkin build --no-status
```

## Observed failures and resolutions

### Dependency resolution

`rosdep` reported:

```text
mrs_lib: Cannot locate rosdep definition for [opencv]
```

This is an obsolete rosdep key in `mrs_lib` 0.0.5. It did not prevent the test
because `cv_bridge` installed Bionic's OpenCV development packages transitively.
For a deterministic reconstruction, install `libopencv-dev` explicitly rather
than relying on that transitive dependency.

### First compiler attempt

`hector_map_tools` and `mrs_msgs` built. `mrs_lib` failed first with:

```text
fatal error: tf2_eigen/tf2_eigen.h: No such file or directory
```

Installing `ros-melodic-tf2-eigen` exposed the next missing header:

```text
fatal error: tf_conversions/tf_eigen.h: No such file or directory
```

Installing `ros-melodic-tf-conversions` resolved it. Both dependencies are used
by the pinned `mrs_lib` source but are absent from its package manifest.

### Final result

The next build completed with no warnings or failures:

```text
Finished <<< hector_map_tools
Finished <<< mrs_msgs
Finished <<< mrs_lib
Finished <<< relative_pose_estimator
Finished <<< occupancy_grid_merger
[build] Summary: All 5 packages succeeded!
[build] Warnings: None.
[build] Failed: No packages failed.
```

## Implications for repository changes

- Do not change either project `CMakeLists.txt` to make the historical build pass;
  the existing definitions compile successfully.
- Do not add `tf2_eigen` or `tf_conversions` to these two project manifests solely
  because of this result. The missing declarations belong to the pinned external
  `mrs_lib` 0.0.5 package.
- A reproducible environment definition should explicitly install
  `libopencv-dev`, `ros-melodic-tf2-eigen`, and
  `ros-melodic-tf-conversions` before invoking rosdep and Catkin.
- Runtime behavior, nodelet loading, launch files, and experiment dependencies
  were outside this compile-only reconstruction and remain unverified.

