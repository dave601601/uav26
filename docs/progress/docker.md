# docker

Build environment for the workspace.

## Status

- Image: `uav-aruco:latest`, built from `Dockerfile` at repo root. Base: `ros:jazzy`.
- Pinned: `numpy<2`, `opencv-contrib-python<4.13` (the apt scipy / trimesh stack is built against numpy 1.x ABI; `opencv-contrib-python>=4.13` requires numpy>=2 so they're capped together).
- `compose.yml` mounts `./src`, `./build`, `./ros` and exports DISPLAY for Gazebo GUI. NVIDIA passthrough enabled.
- `compose.override.yml` (gitignored) adds local-only overrides — auto-loaded by `docker compose` with no flag. Defaults: persist `install/` across container restarts, and auto-run `hover_demo` on `docker compose up`.

## Open

- None blocking. Possible future: split `compose.yml` cleanly so `compose.dev.yml` (auto-run sim) and `compose.test.yml` (bash shell only) are distinct, instead of one user-local override.

## Done

### compose.override.yml convention + install/ persistence + auto-run (commit `f8ed189`, `00276a8`, 2026-05-25)

Two changes in one workflow:

- **`compose.override.yml` is the canonical local-override file.** Docker Compose auto-loads it on top of `compose.yml` without any `-f` flag. Gitignored so each developer keeps their own; the base `compose.yml` stays version-controlled and team-shared. Pattern in `.gitignore`: `compose.override.yml` plus `compose.*.yml` (with `!compose.yml` exception to preserve the base file).
- **`./install` bind mount** in the override so `colcon`'s install tree survives `docker compose down/up`. Without this, every container restart wipes `/workspace/install` and requires a full rebuild. With it: rebuild once, then launches are instant.
- **Auto-run command** in the override: `bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch fc_sim hover_demo.launch.py"`. Running `docker compose up uav-aruco` now brings up the sim + drone hovering in one step. Override for an interactive shell via `docker compose run --rm uav-aruco bash`.

### Pinned numpy / opencv (commit `4aec491`, 2026-04-29)
- `RUN pip install --break-system-packages --no-cache-dir --ignore-installed 'numpy<2' 'opencv-contrib-python<4.13' trimesh transforms3d` in Dockerfile.

## Decisions

- `compose.override.yml` is the user-customizable layer. Edit it freely for local conveniences (alternate auto-commands, different mounts, NVIDIA flags). The shared `compose.yml` should not change without team coordination.
- First-run bootstrap: `docker compose run --rm uav-aruco bash -lc "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --packages-ignore realsense2_camera realsense2_camera_msgs"` builds everything except the realsense submodule (which fails to build in this image and isn't needed for sim).
