#!/usr/bin/env python3
"""Commit and push README improvements for Autonomous-UGV."""
import subprocess, sys, os

REPO = "/home/ros2/Autonomous-UGV"
os.chdir(REPO)

def run(cmd):
    print(f"$ {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    if r.stdout: print(r.stdout.rstrip())
    if r.stderr: print("STDERR:", r.stderr.rstrip())
    if r.returncode != 0:
        print(f"[EXIT {r.returncode}]")
        sys.exit(r.returncode)
    return r

run(["git", "config", "user.name", "PrakashKumarSahu"])
run(["git", "config", "user.email", "jaysahu0201@gmail.com"])
run(["git", "status", "--short"])
run(["git", "add", "README.md"])
run(["git", "diff", "--cached", "--stat"])
run(["git", "commit",
     "--author=PrakashKumarSahu <jaysahu0201@gmail.com>",
     "-m",
     """docs(readme): improve structure, badges, layout; remove unnecessary elements

- Add shields.io badges (ROS2 Jazzy, Gazebo Harmonic, Python 3.12, Apache-2.0)
- Remove GitHub Stars badge and star-encouragement footer (unnecessary)
- Add centered header with tagline and in-page navigation links
- Add Quick Start TL;DR block at top for immediate usability
- Improve architecture diagram with cleaner box-drawing layout
- Add hardware requirements table (GPU VRAM, RAM, disk)
- Add depth source comparison table (accuracy, setup complexity, hardware)
- Restructure installation into numbered steps with callout blocks
- Add Nav2 + RTAB-Map parameter tables with rationale column per row
- Reformat troubleshooting: Issue / Cause / Fix headings + commands
- Add common launch variant examples section
- Add YOLO class and point-cloud tuning guidance to modify section
- Improve DA V3 Docker section with explicit numbered prerequisite steps
- Add NOTE/TIP/WARNING/CAUTION/IMPORTANT callouts throughout
- Clean package structure tree with emoji section markers
- Remove redundancy between sections; improve overall flow"""])
run(["git", "push", "origin", "main"])
run(["git", "log", "--oneline", "-3"])
print("\nDone.")
