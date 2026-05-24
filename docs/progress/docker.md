# docker

Build environment for the workspace.

## Status

- Image: `uav-aruco:latest`, built from `Dockerfile` at repo root. Base: `ros:jazzy`.
- Pinned: `numpy<2`, `opencv-contrib-python<4.13` (the apt scipy / trimesh stack is built against numpy 1.x ABI; `opencv-contrib-python>=4.13` requires numpy>=2 so they're capped together).
- `compose.yml` mounts `./src`, `./build`, `./ros` and exports DISPLAY for Gazebo GUI. NVIDIA passthrough enabled.

## Open

- Rebuild `uav-aruco:latest` against the current Dockerfile (memory note from 2026-04-30 says the cached image predates the numpy pin and was inline-patched in-container).
- After rebuild, run `colcon build --packages-select fc_core fc_sim_msgs fc_sim line_tracer world && colcon test --packages-select fc_core line_tracer`.

## Done

### Pinned numpy / opencv (commit `4aec491`, 2026-04-29)
- `RUN pip install --break-system-packages --no-cache-dir --ignore-installed 'numpy<2' 'opencv-contrib-python<4.13' trimesh transforms3d` in Dockerfile.
