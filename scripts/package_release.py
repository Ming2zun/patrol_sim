#!/usr/bin/env python3
"""Build a source snapshot and one self-documented Baidu resource folder."""
from pathlib import Path
import hashlib
import shutil
import subprocess
import tarfile

root = Path(__file__).resolve().parents[1]
version = (root / "VERSION").read_text().strip()
output = root.parent / "releases" / f"v{version}"
github = output / "github" / "patrol_sim"
baidu = output / f"patrol_sim-v{version}-运行与开发资源"
if output.exists():
    raise SystemExit(f"发布目录已存在：{output}。请先归档旧发布目录或更新 VERSION。")
names = [n for n in subprocess.check_output(["git", "ls-files", "-z"], cwd=root).decode().split("\0") if n]
if not names:
    raise SystemExit("先执行 git add .，确认发布源码列表。")
for name in names:
    if any(p in {".codex", ".agents", ".openai", "build", "install", "log", "assets", "__pycache__"} for p in Path(name).parts):
        raise SystemExit(f"源码列表含生成文件或私有配置：{name}")
    if (root / name).is_symlink():
        raise SystemExit(f"源码不能依赖外部符号链接：{name}")
github.mkdir(parents=True)
baidu.mkdir()
for name in names:
    target = github / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(root / name, target)
bundles = {
    f"patrol_sim-v{version}-runtime-linux-x86_64.tar.gz": ["simulation/runtime/AlpinePatrol.x86_64", "simulation/runtime/AlpinePatrol.pck", "simulation/runtime/GODOT_LICENSE.txt"],
    f"patrol_sim-v{version}-godot-assets.tar.gz": ["simulation/godot/assets"],
}
checksums = []
for filename, members in bundles.items():
    print("打包：", filename, flush=True)
    path = baidu / filename
    with tarfile.open(path, "w:gz") as archive:
        for name in members:
            archive.add(root / name, arcname=name)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    checksums.append(f"{digest.hexdigest()}  {filename}\n")
    print(f"{path.stat().st_size / 1024**2:.2f} MiB", flush=True)
(baidu / "SHA256SUMS.txt").write_text("".join(checksums))
(baidu / "下载后先看.txt").write_text(f"""无人车巡检 patrol_sim v{version} 运行与开发资源

请搭配 GitHub v{version} 源码使用。本文件夹不是完整源码项目。
1. 下载 GitHub 源码并解压，进入包含 README.md、VERSION、start_sim.sh 的项目根目录。
2. 保持本文件夹内的 tar.gz 和 SHA256SUMS.txt 放在一起，不要分别解压到下载目录。
3. 在项目根目录打开终端，执行（路径按实际下载位置修改）：
   bash scripts/install_resources.sh "$HOME/Downloads/patrol_sim-v{version}-运行与开发资源"
4. 脚本自动校验并安装，然后执行 ./start_sim.sh。
   使用 ROS 2 时先安装 README 所列依赖，再执行 ./start_ros2.sh，首次自动编译。

runtime-linux-x86_64：直接运行仿真必需，安装到 simulation/runtime/。
godot-assets：使用 Godot 编辑器开发时需要，安装到 simulation/godot/assets/。
只运行可以仅下载 runtime 包和校验文件；安装脚本会安装文件夹里已下载的匹配版本资源包。
项目文件夹可以改名或放在其他位置，脚本按自身位置查找项目，不依赖开发者机器路径。

手动安装：先在本资源文件夹运行 sha256sum -c SHA256SUMS.txt --ignore-missing。
再回到项目根目录运行 tar -xzf 资源包的完整路径。
压缩包内部已包含 simulation/ 目录层级，不要在 simulation/ 里面再次解压。
安装资源会覆盖对应资源文件，修改过模型或程序请先备份。
""")
(output / "上传说明.txt").write_text("GitHub：上传 github/patrol_sim/ 里面的全部源码，包含隐藏的 .gitignore 与 .gitattributes。\n网盘：上传同级名称以“运行与开发资源”结尾的整个文件夹，无需解压 tar.gz。\n这些目录是发布快照，开发请回到 patrol_sim 项目，上传后补充 README 分享链接。\n")
print("发布完成：", output)
