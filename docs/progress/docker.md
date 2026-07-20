# docker

Build environment for the workspace.

## WSL DNS proxy failure blocks git push from the host (2026-07-14)

Symptom: `git push` fails with "Could not resolve host: github.com"
even though the network is up. Diagnosis that separates the failure
modes:

- `/etc/resolv.conf` points at WSL's NAT DNS proxy (10.255.255.254),
  which stops answering (`getent hosts github.com` times out).
- Raw TCP to an external IP still works, and querying 1.1.1.1 directly
  answers — only the WSL proxy hop is dead, not the network.
- Containers are unaffected: Docker Desktop injects its own resolver
  (192.168.65.7, routed via the Windows host), so in-container DNS keeps
  working while the host distro's is down. Pushing from the container is
  still not an option — compose mounts only `src/`, `build/`, `ros/`,
  so the container sees neither `.git` nor `.env`.

Fix (needs sudo, lasts until WSL restarts):
`sudo sh -c 'printf "nameserver 1.1.1.1\nnameserver 8.8.8.8\n" > /etc/resolv.conf'`.
Permanent: set `generateResolvConf=false` under `[network]` in
`/etc/wsl.conf`, then keep a static resolv.conf.

## dev.sh sweep was killing itself, not the strays (2026-07-09)

`sweep()` ran its patterns through `$EXEC bash -c "<text>"`. `pkill -f`
matches a whole command line, and that shell's command line CONTAINED
the patterns — so `pkill -9 -f 'gz sim'` SIGKILLed the sweeper. Measured:
`bash -c "pkill -9 -f 'gz sim'; echo x"` exits 137 and prints nothing,
and decoys named `fc_sim_node` / `rqt_image_view` planted beforehand
survive it; the same patterns fed on stdin execute all the way through
and both decoys die.

So for every `dev.sh gui` session only the opening
`pkill -INT -f 'ros2 launch'` ever ran. That tears most of a run down,
because launch propagates SIGTERM to its children on a catchable
shutdown — which is exactly why the bug hid. The by-name `-9` pass that
this whole contract exists for, the one whose absence poisoned r42..r51,
was dead code there.

Fix: feed the sweeper on stdin (`$EXEC bash -s <<'SWEEP'`), the way
`run_mission.sh` always has. Its command line is then just `bash -s`, so
no pattern can match it. Any future in-container script that pkills by
pattern must be piped, never inlined with `-c`.

## Process hygiene (2026-07-08) — read before running missions

- `pkill -9 -f 'ros2 launch'` ORPHANS the launch's children: SIGKILL
  is uncatchable, so launch never propagates shutdown to its nodes.
  Orphaned `fc_sim_node` + `parameter_bridge` re-activate on the next
  run's /clock+/imu (topics are absolute) and fight the new FC for
  the drone. This accumulated one zombie FC pair per run and poisoned
  r42..r51 — every "physics mystery" in that range was multi-FC
  interference. Same lesson as the r19-r22 zombie containers, one
  level down.
- Teardown contract, codified in `scripts/run_mission.sh`: sweep
  strays BEFORE starting (fail loud if any survive), SIGINT the
  launches first, then pkill stragglers BY NAME including fc_sim_node
  and parameter_bridge. fc_sim also fatals + zeroes output if it sees
  a second publisher on the motor topic.
- `compose.yml` pins `ROS_DOMAIN_ID=26` + `ROS_LOCALHOST_ONLY=1`:
  with network_mode host and default domain 0, any ROS node on the
  machine (other containers, host tools, orphans) joins the sim
  graph. Recreating the container drops `/workspace/install` (not a
  mounted volume) — rebuild after `docker compose up -d`.
- GUI under Docker Desktop (WSL2): absolute bind-mount paths resolve
  inside the docker-desktop utility VM, so `/tmp/.X11-unix` (and
  `/mnt/wslg`, and $HOME symlinks pointing at them) grab the VM's own
  invisible Xwayland — gz renders happily into a ghost session and no
  window ever appears. The abstract X socket is not listening, and a
  root bind through `/mnt/wsl` needs sudo per boot. What DOES bridge
  into the user distro is project-relative mounts, so compose mounts
  `./.x11-bridge:/tmp/.X11-unix` and `scripts/x11_bridge.py` (a
  sudo-less userspace relay, auto-started by `scripts/dev.sh gui`)
  serves a proxied `X0` there, forwarding to the real
  `/mnt/wslg/.X11-unix/X0`. Verified end to end: gz's Qt connections
  relay to the user-session Xwayland (raw X handshake accepted for a
  non-default user — client uid does not matter) and the window
  reaches the Weston compositor. Caveat: the relay cannot forward
  SCM_RIGHTS fds, so DRI3/MIT-SHM fall back to plain wire transport
  (slower, still correct).
- If no window appears ANYWHERE despite all of the above, check
  `/mnt/wslg/weston.log`: `CreateWndow(): rdp_peer is not initalized`
  means WSLg's Windows-side RDP client (msrdc.exe, see `tasklist.exe`)
  is dead — the compositor has no channel to the Windows desktop and
  no WSL GUI app from any user can display. Remedy: `wsl --shutdown`
  from Windows, reopen the terminal (kills all WSL sessions including
  running agents/containers).
  `scripts/dev.sh gui` = bridge + container up + rebuild-if-needed +
  sim + tracer with Ctrl+C teardown; `scripts/dev.sh mission rNN
  [dur]` is the headless equivalent.

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
