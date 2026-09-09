#!/usr/bin/env python3
"""Verify matching resource bundles and install relative to this checkout."""
from pathlib import Path
import hashlib
import shutil
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def checksum(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

def install(folder):
    version = (ROOT / "VERSION").read_text().strip()
    expected = {}
    for line in (folder / "SHA256SUMS.txt").read_text().splitlines():
        if line.strip():
            digest, name = line.split(maxsplit=1)
            expected[name.lstrip(" *")] = digest
    bundles = {
        f"patrol_sim-v{version}-runtime-linux-x86_64.tar.gz": "simulation/runtime",
        f"patrol_sim-v{version}-godot-assets.tar.gz": "simulation/godot/assets",
    }
    selected = [(folder / name, prefix) for name, prefix in bundles.items() if (folder / name).is_file()]
    if not selected:
        raise ValueError(f"未找到匹配 v{version} 的资源包，请选择包含 tar.gz 和 SHA256SUMS.txt 的文件夹")
    for path, prefix in selected:
        if checksum(path) != expected.get(path.name):
            raise ValueError(f"校验失败：{path.name}，未安装任何资源")
        with tarfile.open(path) as archive:
            members = archive.getmembers()
            if not members:
                raise ValueError(f"空资源包：{path.name}")
            for member in members:
                rel = Path(member.name)
                if (rel.is_absolute() or ".." in rel.parts or
                    not (member.name == prefix or member.name.startswith(prefix + "/")) or
                    not (member.isfile() or member.isdir())):
                    raise ValueError(f"资源包包含非法路径或文件类型：{member.name}")
                target = ROOT / rel
                if target.is_symlink() or any(p.is_symlink() for p in target.parents if p != ROOT and ROOT in p.parents):
                    raise ValueError(f"目标路径不能是符号链接：{target}")
    with tempfile.TemporaryDirectory(prefix=".resource-install-", dir=ROOT) as temp:
        staging = Path(temp)
        for path, prefix in selected:
            print(f"校验通过，安装 {path.name}", flush=True)
            with tarfile.open(path) as archive:
                archive.extractall(staging)  # All members validated above; links forbidden.
        for path in staging.rglob("*"):
            target = ROOT / path.relative_to(staging)
            if path.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
    print(f"资源已安装到：{ROOT / 'simulation'}")
    print("运行仿真：./start_sim.sh；启动 ROS 与 RViz：./start_ros2.sh")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("用法：./scripts/install_resources.sh 下载的资源文件夹路径")
    try:
        install(Path(sys.argv[1]).expanduser().resolve())
    except (OSError, ValueError, tarfile.TarError) as error:
        sys.exit(str(error))
