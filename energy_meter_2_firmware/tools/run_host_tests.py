#!/usr/bin/env python3
"""
run_host_tests.py - build and run the host unit tests without PlatformIO's native platform.

`pio test -e native` is the normal way (needs g++ on PATH). This script does the same
job on a machine that only has Python: it uses g++ or clang++ if present, otherwise the
`ziglang` pip package (pip install ziglang), and downloads the three-file Unity test
library into .pio/unity on first use.

    python tools/run_host_tests.py
"""

import os
import shutil
import subprocess
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UNITY_VERSION = "v2.6.0"
UNITY_FILES = ("unity.c", "unity.h", "unity_internals.h")
SOURCES = ["diag_core"]


def find_compiler():
    for name in ("g++", "clang++"):
        if shutil.which(name):
            return [name], [shutil.which("gcc") or shutil.which("clang") or name]
    try:
        import ziglang  # noqa: F401
    except ImportError:
        sys.exit("No C++ compiler found. Install g++/clang++, or: pip install ziglang")
    return [sys.executable, "-m", "ziglang", "c++"], [sys.executable, "-m", "ziglang", "cc"]


def ensure_unity():
    folder = os.path.join(ROOT, ".pio", "unity")
    os.makedirs(folder, exist_ok=True)
    for name in UNITY_FILES:
        path = os.path.join(folder, name)
        if not os.path.isfile(path):
            url = "https://raw.githubusercontent.com/ThrowTheSwitch/Unity/%s/src/%s" % (UNITY_VERSION, name)
            print("fetching", url)
            urllib.request.urlretrieve(url, path)
    return folder


def run(cmd):
    result = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if result.returncode != 0:
        print(result.stdout)
    return result


def main():
    cxx, cc = find_compiler()
    unity = ensure_unity()
    out = os.path.join(ROOT, ".pio", "native")
    os.makedirs(out, exist_ok=True)

    unity_obj = os.path.join(out, "unity.o")
    if run(cc + ["-c", os.path.join(unity, "unity.c"), "-I", unity, "-o", unity_obj]).returncode:
        sys.exit("building Unity failed")

    exe = os.path.join(out, "test_core.exe" if os.name == "nt" else "test_core")
    build = cxx + ["-std=gnu++17", "-Wall", "-Wextra", "-Wno-nullability-completeness",
                   "-I", "include", "-I", unity, "-DUNIT_TEST"]
    build += ["src/%s.cpp" % name for name in SOURCES]
    build += ["test/test_core/test_core.cpp", unity_obj, "-o", exe]
    if run(build).returncode:
        sys.exit("build failed")

    sys.exit(subprocess.run([exe], cwd=ROOT).returncode)


if __name__ == "__main__":
    main()
