"""3-node distributed skrl PPO trainer across local (redstone), pa16@slave2, and pa10@slave3."""

from __future__ import annotations

import argparse
import io
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[2]
HUB_PORT = 29555
NODES = (
    {"name": "redstone", "ssh": None, "workdir": str(ROOT_DIR), "python": "/opt/IsaacLab/.venv/bin/python"},
    {"name": "slave2", "ssh": "pa16@slave2", "workdir": "/home/pa16/xsting_carpet_dist", "python": "/home/pa16/IsaacLab/_isaac_sim/python.sh"},
    {"name": "slave3", "ssh": "pa10@slave3", "workdir": "/home/pa10/xsting_carpet_dist", "python": "/home/pa10/pa-course/.venv-isaaclab/bin/python"},
)


def _free_vram_mib(ssh_host: str | None) -> int:
    q = ["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"]
    cmd = q if ssh_host is None else ["ssh", "-o", "BatchMode=yes", ssh_host, " ".join(q)]
    out = subprocess.check_output(cmd, text=True).strip().splitlines()[0]
    return int(out.strip())


def _select_active_nodes() -> list[dict]:
    active: list[dict] = []
    for n in NODES:
        free_mb = _free_vram_mib(n["ssh"])
        envs = 1024 if free_mb >= 5500 else (512 if free_mb >= 3000 else (192 if free_mb >= 1750 else 0))
        print(f"[CLUSTER] {n['name']}: {free_mb} MiB free VRAM -> {envs} envs", flush=True)
        if envs > 0:
            active.append({**n, "envs": envs})
    active.sort(key=lambda x: (x["ssh"] is not None, -x["envs"]))
    for idx, item in enumerate(active):
        item["rank"] = idx
    return active


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray(n)
    view = memoryview(buf)
    pos = 0
    while pos < n:
        chunk = sock.recv_into(view[pos:])
        if chunk == 0:
            raise ConnectionError("Cluster socket closed")
        pos += chunk
    return bytes(buf)


def _run_hub(server: socket.socket, active_nodes: list[dict]) -> None:
    world_size = len(active_nodes)
    conns: dict[int, socket.socket] = {}
    while len(conns) < world_size:
        conn, _ = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        (rank,) = struct.unpack("<I", _recv_exact(conn, 4))
        conns[rank] = conn
    ordered = [conns[r] for r in range(world_size)]
    print(f"[CLUSTER-HUB] All {world_size} GPU nodes connected: {[n['name'] for n in active_nodes]}", flush=True)

    try:
        while True:
            ops, payloads = [], []
            for c in ordered:
                op, size = struct.unpack("<BI", _recv_exact(c, 5))
                ops.append(op)
                payloads.append(_recv_exact(c, size) if size > 0 else b"")
            if ops[0] == 1:  # Broadcast from rank 0
                out = payloads[0]
            else:  # Float32 sum all_reduce
                acc = np.frombuffer(payloads[0], dtype=np.float32).copy()
                for p in payloads[1:]:
                    acc += np.frombuffer(p, dtype=np.float32)
                out = acc.tobytes()
            header = struct.pack("<I", len(out))
            for c in ordered:
                c.sendall(header + out)
    except ConnectionError:
        pass
    finally:
        for c in ordered:
            c.close()


def _run_worker(rank: int, world_size: int, weight_scale: float, forward_args: list[str]) -> int:
    import warp as wp

    wp.config.enable_backward = False
    sys.path.insert(0, str(ROOT_DIR))

    import skrl  # noqa: E402
    from skrl.models.torch.base import Model  # noqa: E402
    import torch  # noqa: E402
    import torch.distributed as dist  # noqa: E402

    sock = socket.create_connection(("127.0.0.1", HUB_PORT), timeout=180.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(None)
    sock.sendall(struct.pack("<I", rank))

    skrl.config.torch._rank = rank
    skrl.config.torch._local_rank = 0
    skrl.config.torch._world_size = world_size
    skrl.config.torch._is_distributed = True

    def _bcast_params(self: Model, *, rank: int = 0) -> None:
        if skrl.config.torch.rank == 0:
            buf = io.BytesIO()
            torch.save({k: v.cpu() for k, v in self.state_dict().items()}, buf)
            raw = buf.getvalue()
        else:
            raw = b""
        sock.sendall(struct.pack("<BI", 1, len(raw)) + raw)
        (size,) = struct.unpack("<I", _recv_exact(sock, 4))
        state = torch.load(io.BytesIO(_recv_exact(sock, size)), map_location=self.device, weights_only=True)
        self.load_state_dict(state)

    def _all_reduce_sum(tensor: torch.Tensor, scale: float = 1.0) -> torch.Tensor:
        cpu_bytes = (tensor.detach().contiguous().view(-1).float() * scale).cpu().numpy().tobytes()
        sock.sendall(struct.pack("<BI", 2, len(cpu_bytes)) + cpu_bytes)
        (size,) = struct.unpack("<I", _recv_exact(sock, 4))
        res = np.frombuffer(_recv_exact(sock, size), dtype=np.float32).copy()
        return torch.from_numpy(res).to(device=tensor.device, dtype=tensor.dtype).view_as(tensor)

    def _reduce_params(self: Model) -> None:
        grads = [p.grad.view(-1) for p in self.parameters() if p.grad is not None]
        flat = _all_reduce_sum(torch.cat(grads), scale=weight_scale) / world_size
        offset = 0
        for p in self.parameters():
            if p.grad is not None:
                n = p.numel()
                p.grad.data.copy_(flat[offset : offset + n].view_as(p.grad.data))
                offset += n

    def _dist_all_reduce(tensor: torch.Tensor, op: object = None, **_kwargs: object) -> None:
        tensor.copy_(_all_reduce_sum(tensor, scale=weight_scale))

    Model.broadcast_parameters = _bcast_params  # type: ignore[method-assign]
    Model.reduce_parameters = _reduce_params  # type: ignore[method-assign]
    dist.all_reduce = _dist_all_reduce  # type: ignore[assignment]

    from isaaclab_rl.entrypoints import run_train_cli  # noqa: E402
    import xsting_walk  # noqa: E402,F401

    try:
        base = ["--task", "Isaac-XSting-CarpetWalk-Direct-v0", "--rl_library", "skrl", "--viz", "none"]
        rc = run_train_cli([*base, *forward_args])
        sock.close()
        os._exit(rc)
    finally:
        sock.close()


def _launch_cluster(args: argparse.Namespace, extra: list[str]) -> int:
    active_nodes = _select_active_nodes()
    if not active_nodes:
        raise RuntimeError("No cluster GPU nodes have >= 1750 MiB free VRAM.")

    ckpt_abs = Path(args.checkpoint).resolve() if args.checkpoint else None
    for node in active_nodes:
        if node["ssh"] is None:
            continue
        host, wdir = node["ssh"], node["workdir"]
        print(f"[CLUSTER] Syncing code to {host}:{wdir} ...", flush=True)
        subprocess.run(["ssh", "-o", "BatchMode=yes", host, f"mkdir -p {wdir}/checkpoints"], check=True)
        subprocess.run(
            ["rsync", "-az", "--delete", str(ROOT_DIR / "xsting_description"), str(ROOT_DIR / "xsting_walk"), f"{host}:{wdir}/"],
            check=True,
        )
        if ckpt_abs:
            subprocess.run(["scp", "-q", str(ckpt_abs), f"{host}:{wdir}/checkpoints/warmup.pt"], check=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", HUB_PORT))
    server.listen(len(active_nodes))
    threading.Thread(target=_run_hub, args=(server, active_nodes), daemon=True).start()

    total_envs = sum(n["envs"] for n in active_nodes)
    procs: list[subprocess.Popen] = []
    log_files = []

    def _cleanup(*_sig: object) -> None:
        for p in procs:
            if p.poll() is None:
                p.terminate()

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    try:
        for node in reversed(active_nodes):
            r, envs = node["rank"], node["envs"]
            w_scale = (envs / total_envs) * len(active_nodes)
            worker_args = [
                "xsting_walk/scripts/train_cluster.py",
                f"--worker_rank={r}",
                f"--world_size={len(active_nodes)}",
                f"--weight_scale={w_scale:.6f}",
                "--num_envs",
                str(envs),
                "--seed",
                str(42 + r * 1000),
                f"agent.trainer.timesteps={args.timesteps}",
                *extra,
            ]
            if ckpt_abs:
                worker_args.extend(["--checkpoint", str(ckpt_abs) if node["ssh"] is None else "checkpoints/warmup.pt"])
            if r > 0:
                worker_args.extend(["agent.agent.experiment.write_interval=0", "agent.agent.experiment.checkpoint_interval=0"])

            if node["ssh"] is None:
                cmd = [node["python"], *worker_args]
                procs.append(subprocess.Popen(cmd, cwd=node["workdir"]))
            else:
                remote_cmd = f"cd {node['workdir']} && {node['python']} " + " ".join(worker_args)
                cmd = [
                    "ssh",
                    "-o",
                    "BatchMode=yes",
                    "-o",
                    "ExitOnForwardFailure=yes",
                    "-R",
                    f"{HUB_PORT}:127.0.0.1:{HUB_PORT}",
                    node["ssh"],
                    remote_cmd,
                ]
                if r == 0:
                    procs.append(subprocess.Popen(cmd))
                else:
                    log_f = open(ROOT_DIR / "logs" / f"cluster_{node['name']}.log", "w")
                    log_files.append(log_f)
                    procs.append(subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT))

        code0 = procs[-1].wait()
        _cleanup()
        rank0 = active_nodes[0]
        if code0 == 0 and rank0["ssh"] is not None:
            print(f"[CLUSTER] Pulling checkpoints from {rank0['ssh']} ...", flush=True)
            os.makedirs(ROOT_DIR / "logs" / "skrl" / "xsting_carpet_walk", exist_ok=True)
            subprocess.run(
                [
                    "rsync",
                    "-az",
                    f"{rank0['ssh']}:{rank0['workdir']}/logs/skrl/xsting_carpet_walk/",
                    str(ROOT_DIR / "logs" / "skrl" / "xsting_carpet_walk") + "/",
                ],
                check=True,
            )
        return code0
    finally:
        _cleanup()
        for lf in log_files:
            lf.close()
        server.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker_rank", type=int, default=None)
    parser.add_argument("--world_size", type=int, default=3)
    parser.add_argument("--weight_scale", type=float, default=1.0)
    parser.add_argument("--timesteps", type=int, default=16000)
    parser.add_argument("--checkpoint", type=str, default=None)
    args, extra = parser.parse_known_args()
    if args.worker_rank is not None:
        return _run_worker(args.worker_rank, args.world_size, args.weight_scale, extra)
    os.makedirs(ROOT_DIR / "logs", exist_ok=True)
    return _launch_cluster(args, extra)


if __name__ == "__main__":
    raise SystemExit(main())
