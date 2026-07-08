#!/usr/bin/env python3
"""Userspace X11 socket bridge for Docker Desktop on WSL2. No sudo.

Docker Desktop resolves absolute bind-mount sources inside its utility
VM, so a container can never mount the user distro's WSLg X socket
directly (/tmp/.X11-unix and /mnt/wslg both resolve to the VM's own
ghost Xwayland, and symlinks under $HOME are resolved VM-side too).
Project-RELATIVE mounts, however, are bridged into the user distro.

So: listen on <repo>/.x11-bridge/X0 (which compose mounts into the
container as /tmp/.X11-unix) and relay byte streams to the real WSLg
socket at /mnt/wslg/.X11-unix/X0.

Limitation: ancillary data (SCM_RIGHTS fd passing) is not forwarded,
so X extensions that hand file descriptors across (DRI3, MIT-SHM fd
variants) fail and clients fall back to plain wire transport. Qt/gz
handle that fallback; the GUI is somewhat slower but visible.

Started automatically by scripts/dev.sh gui; writes a pidfile and a
per-connection line to <repo>/.x11-bridge/log.
"""
import os
import socket
import sys
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIDGE_DIR = os.path.join(REPO, ".x11-bridge")
SOCK = os.path.join(BRIDGE_DIR, "X0")
PIDFILE = os.path.join(BRIDGE_DIR, "pid")
LOGFILE = os.path.join(BRIDGE_DIR, "log")
TARGET = "/mnt/wslg/.X11-unix/X0"


def log(msg: str) -> None:
    with open(LOGFILE, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} {msg}\n")


def pump(src: socket.socket, dst: socket.socket) -> None:
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def handle(client: socket.socket, conn_id: int) -> None:
    try:
        upstream = socket.socket(socket.AF_UNIX)
        upstream.connect(TARGET)
    except OSError as exc:
        log(f"conn {conn_id}: upstream connect failed: {exc}")
        client.close()
        return
    log(f"conn {conn_id}: relaying")
    t = threading.Thread(target=pump, args=(upstream, client), daemon=True)
    t.start()
    pump(client, upstream)
    t.join(timeout=1.0)
    for s in (client, upstream):
        try:
            s.close()
        except OSError:
            pass


def main() -> int:
    if not os.path.exists(TARGET):
        print(f"no WSLg X socket at {TARGET}", file=sys.stderr)
        return 1
    os.makedirs(BRIDGE_DIR, exist_ok=True)
    try:
        os.unlink(SOCK)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX)
    srv.bind(SOCK)
    os.chmod(SOCK, 0o777)
    srv.listen(32)
    with open(PIDFILE, "w") as f:
        f.write(str(os.getpid()))
    log(f"bridge up: {SOCK} -> {TARGET} (pid {os.getpid()})")
    conn_id = 0
    while True:
        client, _ = srv.accept()
        conn_id += 1
        threading.Thread(
            target=handle, args=(client, conn_id), daemon=True
        ).start()


if __name__ == "__main__":
    sys.exit(main())
