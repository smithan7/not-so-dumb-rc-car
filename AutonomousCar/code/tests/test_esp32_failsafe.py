"""Compile and run the ESP32 firmware's host-side unit tests (failsafe.h) with the local C++ compiler."""

import glob
import os
import shutil
import subprocess
import sys

import pytest

FW_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "esp32_autopilot_interface")


def _compile(src, exe):
    cmd = ["c++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", FW_DIR, src, "-o", exe]
    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0 and sys.platform == "darwin" and "tapi error" in build.stderr:
        # a broken default Command Line Tools SDK fails at link time; any other SDK will do
        for sdk in sorted(glob.glob("/Library/Developer/CommandLineTools/SDKs/MacOSX*.sdk"), reverse=True):
            build = subprocess.run(cmd + ["-isysroot", sdk], capture_output=True, text=True)
            if build.returncode == 0:
                break
    return build


@pytest.mark.skipif(shutil.which("c++") is None, reason="no C++ compiler")
def test_failsafe_logic(tmp_path):
    exe = str(tmp_path / "test_failsafe")
    build = _compile(os.path.join(FW_DIR, "test", "test_failsafe.cpp"), exe)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([exe], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
