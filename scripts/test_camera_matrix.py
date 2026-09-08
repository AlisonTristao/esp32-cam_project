"""Build/run the hardware-independent matrix and SD config tests."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--compiler", default="g++")
parser.add_argument("--btp", type=Path, default=root.parent / "BTP")
args = parser.parse_args()
compiler = shutil.which(args.compiler)
if compiler is None:
    parser.error(f"C++ compiler not found: {args.compiler}")
env = os.environ.copy()
env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
with tempfile.TemporaryDirectory(prefix="camera_matrix_") as directory:
    binary = Path(directory) / ("tests.exe" if os.name == "nt" else "tests")
    subprocess.run([
        compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Isrc", "-Ilib/CameraConfig", f"-I{args.btp / 'include'}",
        "test/test_camera_matrix.cpp", "src/CameraMatrix.cpp",
        "lib/CameraConfig/CameraConfig.cpp", str(args.btp / "src/codec.cpp"),
        "-o", str(binary),
    ], cwd=root, env=env, check=True)
    subprocess.run([str(binary)], env=env, check=True)
