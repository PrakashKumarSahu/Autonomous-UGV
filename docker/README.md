# docker/ — Depth Anything V3 TRT Docker Integration

This directory contains everything needed to run the Depth Anything V3 TensorRT node inside a Docker container on the host GPU, with full ROS 2 topic integration.

## Why Docker?

The `depth_anything_v3` C++ TRT node requires:
- CUDA Toolkit (nvcc, headers)
- TensorRT SDK (NvInfer.h, libnvinfer.so)

These are large, driver-version-sensitive installations. The official prebuilt Docker image (`tillbeemelmanns/ros2-depth-anything-v3:latest-dev`) bundles everything — TRT 10.9, CUDA 13, ROS 2 Jazzy, and the compiled node — so **no host TRT installation is needed**.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│  Host machine (ROS 2 Jazzy, Ubuntu 24.04)           │
│                                                      │
│  ┌──────────────────────────────────────────────┐   │
│  │  Docker container (nvidia runtime)           │   │
│  │  Image: tillbeemelmanns/ros2-depth-anything │   │
│  │  -v3:latest-dev                              │   │
│  │                                              │   │
│  │  depth_anything_v3 node (C++ TRT)            │   │
│  │  - Subscribes: /camera/image_raw             │   │
│  │  - Subscribes: /camera/camera_info           │   │
│  │  - Publishes:  /depth_anything_v3/output/    │   │
│  │                depth_image (32FC1, 15fps)    │   │
│  └───────────────────┬──────────────────────────┘   │
│         ← --network host → ROS 2 DDS (multicast)    │
│                      │                              │
│  da3_relay_node.py   │                              │
│  - Subscribes: /depth_anything_v3/output/depth_image│
│  - Publishes: /perception/depth/image_raw    ───┐   │
│               /perception/depth/camera_info     │   │
│               /perception/depth/colorized        │   │
│                                                  │   │
│  depth_to_pointcloud_node ──────────────────────►│   │
│  - Publishes: /perception/depth/points           │   │
│                           → Nav2 costmap         │   │
└─────────────────────────────────────────────────────┘
```

## Files

| File | Purpose |
|------|---------|
| `da3_trt.compose.yml` | Docker Compose — defines container, GPU, network, volumes, command |
| `da3_docker.param.yaml` | DA V3 TRT node parameters (mounted read-only into container) |
| `.env` | Default environment variables (ROS_DOMAIN_ID, image tag) |

## Quick Start

### 1. One-time setup
```bash
bash scripts/setup_da3_docker.sh
```

### 2. Start DA V3 TRT container
```bash
# From repo root:
docker compose -f docker/da3_trt.compose.yml up
```

### 3. Start UGV stack with DA V3 as depth source
```bash
source install/setup.bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim camera_type:=da_v3_trt
```

## Environment Variables

Override in shell or edit `.env`:

```bash
export ROS_DOMAIN_ID=0            # Must match host UGV launch
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DA3_IMAGE=tillbeemelmanns/ros2-depth-anything-v3:latest-dev
```

## Notes

- **First launch**: TRT compiles ONNX → engine (~2-5 min). Engine is cached in the models volume.
- **Network**: `--network host` is used. Container DDS participant joins the host multicast group.
- **GPU**: Requires NVIDIA Container Toolkit (`nvidia-ctk`) and `--runtime nvidia`.
- **Model**: `DA3METRIC-LARGE.onnx` from `TillBeemelmanns/Depth-Anything-V3-ONNX` on HuggingFace.
