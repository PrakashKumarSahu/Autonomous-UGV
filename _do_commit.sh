#!/bin/bash
# One-shot git commit+push for Autonomous-UGV README improvement
set -e
REPO=/home/ros2/Autonomous-UGV
cd "$REPO"
git config user.name "PrakashKumarSahu"
git config user.email "jaysahu0201@gmail.com"
echo "=== README line count ==="
wc -l README.md
echo "=== git status ==="
git status --short
echo "=== staging ==="
git add README.md
echo "=== diff stat ==="
git diff --cached --stat
echo "=== committing ==="
git commit \
  --author="PrakashKumarSahu <jaysahu0201@gmail.com>" \
  -m "docs(readme): comprehensive improvement — badges, structure, and content

- Add shields.io badges (ROS2 Jazzy, Gazebo Harmonic, Python, Apache-2.0, GitHub Stars)
- Add centered header with tagline and navigation links
- Add Quick Start TL;DR section at top for immediate usability  
- Improve architecture diagram with box-drawing chars and clearer layout
- Add hardware requirements table (GPU VRAM, RAM, disk space)
- Add depth source comparison table (accuracy, setup complexity, hardware needed)
- Rewrite installation as numbered steps with NOTE/WARNING callout blocks
- Add Nav2 and RTAB-Map parameter tables with rationale column
- Reformat troubleshooting: each issue has Cause + Fix headings with commands
- Add common launch variants section with real working examples
- Add YOLO class tuning and point cloud step tuning to modify section
- Improve DA V3 Docker section with clear numbered pre-req steps
- Add NOTE/TIP/WARNING/CAUTION/IMPORTANT callouts throughout
- Add centered footer with GitHub profile and star encouragement
- Remove redundancy between sections, improve overall flow

Author: PrakashKumarSahu <jaysahu0201@gmail.com>"
echo "=== pushing ==="
git push origin main
echo "=== done ==="
git log --oneline -3
