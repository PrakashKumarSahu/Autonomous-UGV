#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# setup_da3_docker.sh — One-time setup for Docker-based Depth Anything V3 TRT
# ══════════════════════════════════════════════════════════════════════════════
#
# Run this script ONCE before launching with camera_type:=da_v3_trt.
#
# What it does:
#   1. Verifies NVIDIA GPU and Docker access
#   2. Pulls the DA V3 TRT Docker image (~5-6 GB, one-time download)
#   3. Downloads the Depth Anything V3 ONNX model (~700 MB from HuggingFace)
#   4. Builds the depth_anything_v3 C++ ROS 2 node inside the container
#      (using CUDA 13 + TensorRT 10.13 from the image) and caches the install
#      in a named Docker volume (ugv_da3_build)
#   5. Verifies GPU access inside the container
#
# After this completes, launch with:
#   Terminal 1: docker compose -f docker/da3_trt.compose.yml up
#   Terminal 2: ros2 launch ugv_bringup ugv_complete.launch.py \
#                  mode:=sim camera_type:=da_v3_trt
#
# Prerequisites:
#   - NVIDIA GPU with driver 520+ (nvidia-smi must work)
#   - Docker Engine 20+ and user in docker group
#   - NVIDIA Container Toolkit (nvidia-ctk runtime configure --runtime=docker)
#
# Usage:
#   bash scripts/setup_da3_docker.sh              # full setup
#   bash scripts/setup_da3_docker.sh --skip-pull  # skip image pull
#   bash scripts/setup_da3_docker.sh --rebuild     # force rebuild of package
# ══════════════════════════════════════════════════════════════════════════════
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DA3_SRC="${REPO_ROOT}/third_party/ros2-depth-anything-v3-trt"
MODELS_DIR="${DA3_SRC}/depth_anything_v3/models"
DA3_IMAGE="${DA3_IMAGE:-tillbeemelmanns/ros2-depth-anything-v3:latest-dev}"
BUILD_VOLUME="ugv_da3_build"
SKIP_PULL="${1:-}"
REBUILD="${1:-}"

echo "══════════════════════════════════════════════════════════════════"
echo "  Depth Anything V3 TRT — Docker Setup"
echo "  Image  : ${DA3_IMAGE}"
echo "  Volume : ${BUILD_VOLUME}"
echo "  Repo   : ${REPO_ROOT}"
echo "══════════════════════════════════════════════════════════════════"

# ── Step 1: Verify GPU ────────────────────────────────────────────────────────
echo ""
echo "[1/5] Checking GPU..."
if ! command -v nvidia-smi &>/dev/null; then
    echo "  ERROR: nvidia-smi not found. Install NVIDIA drivers first."
    exit 1
fi
GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
DRIVER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
echo "  GPU: ${GPU_NAME} (driver ${DRIVER})"

# ── Step 2: Verify Docker ─────────────────────────────────────────────────────
echo ""
echo "[2/5] Checking Docker..."
if ! docker info &>/dev/null; then
    echo "  ERROR: Docker not accessible."
    echo "  Fix: sudo usermod -aG docker \$USER && newgrp docker"
    exit 1
fi
if ! docker info 2>/dev/null | grep -qi nvidia; then
    echo "  ERROR: NVIDIA runtime not in Docker."
    echo "  Fix:"
    echo "    sudo nvidia-ctk runtime configure --runtime=docker"
    echo "    sudo systemctl restart docker"
    exit 1
fi
echo "  Docker OK — NVIDIA runtime present"

# ── Step 3: Pull Docker image ────────────────────────────────────────────────
echo ""
if [[ "${SKIP_PULL}" == "--skip-pull" || "${REBUILD}" == "--rebuild" ]]; then
    echo "[3/5] Skipping image pull (flag specified)."
else
    echo "[3/5] Pulling DA V3 TRT Docker image (~5-6 GB, one-time)..."
    echo "      ${DA3_IMAGE}"
    docker pull "${DA3_IMAGE}"
    echo "  Image ready."
fi

# ── Step 4: Download ONNX model ──────────────────────────────────────────────
echo ""
echo "[4/5] Checking ONNX model..."
mkdir -p "${MODELS_DIR}"

MODEL_FILE="DA3METRIC-LARGE.onnx"
MODEL_PATH="${MODELS_DIR}/${MODEL_FILE}"
HF_URL="https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX/resolve/main/${MODEL_FILE}"

if [[ -f "${MODEL_PATH}" && -s "${MODEL_PATH}" ]]; then
    SIZE=$(du -sh "${MODEL_PATH}" | cut -f1)
    echo "  Model already present: ${MODEL_FILE} (${SIZE})"
else
    echo "  Downloading ${MODEL_FILE} from HuggingFace (~700 MB)..."
    if command -v wget &>/dev/null; then
        wget --show-progress -q -O "${MODEL_PATH}" "${HF_URL}"
    elif command -v curl &>/dev/null; then
        curl -L --progress-bar -o "${MODEL_PATH}" "${HF_URL}"
    else
        echo "  ERROR: wget or curl required."
        exit 1
    fi
    echo "  Downloaded: ${MODEL_PATH} ($(du -sh "${MODEL_PATH}" | cut -f1))"
fi

# ── Step 5: Build depth_anything_v3 inside container ─────────────────────────
echo ""
echo "[5/5] Building depth_anything_v3 C++ node inside container..."

# Check if build volume already exists and has content
VOLUME_HAS_BUILD=false
if docker volume inspect "${BUILD_VOLUME}" &>/dev/null; then
    CONTENT=$(docker run --rm -v "${BUILD_VOLUME}:/da3_install" \
        busybox ls /da3_install/ 2>/dev/null)
    if echo "${CONTENT}" | grep -q "depth_anything_v3"; then
        VOLUME_HAS_BUILD=true
    fi
fi

if [[ "${VOLUME_HAS_BUILD}" == "true" && "${REBUILD}" != "--rebuild" ]]; then
    echo "  Build volume already contains depth_anything_v3 install."
    echo "  Skipping build (use --rebuild to force rebuild)."
else
    echo "  Creating/clearing Docker build volume: ${BUILD_VOLUME}"
    docker volume rm "${BUILD_VOLUME}" 2>/dev/null || true
    docker volume create "${BUILD_VOLUME}"

    echo "  Building inside container (CUDA 13 + TRT 10.13)..."
    echo "  This takes ~1-5 minutes."
    docker run --rm \
        --runtime=nvidia \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e PATH="/usr/local/cuda-13.0/bin:/usr/bin:/usr/local/bin:/bin" \
        -e LD_LIBRARY_PATH="/usr/local/cuda-13.0/lib64:/usr/lib/x86_64-linux-gnu" \
        -v "${DA3_SRC}:/src/da3_src:ro" \
        -v "${BUILD_VOLUME}:/da3_install" \
        "${DA3_IMAGE}" \
        bash -c '
            set -e
            source /opt/ros/jazzy/setup.bash
            echo "  CUDA: $(nvcc --version 2>/dev/null | grep release)"
            echo "  TRT:  $(python3 -c "import tensorrt; print(tensorrt.__version__)" 2>/dev/null || echo not importable)"
            echo "  NvInfer.h: $(ls /usr/include/x86_64-linux-gnu/NvInfer.h 2>/dev/null || echo not found)"
            mkdir -p /da3_ws/src
            cp -r /src/da3_src/depth_anything_v3 /da3_ws/src/
            cd /da3_ws
            colcon build \
              --packages-select depth_anything_v3 \
              --cmake-args \
                -DCMAKE_BUILD_TYPE=Release \
                -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-13.0 \
                -DTENSORRT_ROOT=/usr \
              --install-base /da3_install \
              2>&1 | grep -v "^--" | tail -20
            echo "Build complete. Install contents:"
            ls /da3_install/
        '
    echo "  Build succeeded — install cached in volume '${BUILD_VOLUME}'"
fi

# ── Verify binary ─────────────────────────────────────────────────────────────
echo ""
echo "  Verifying binary..."
BINARY_CHECK=$(docker run --rm \
    -v "${BUILD_VOLUME}:/da3_install:ro" \
    busybox \
    find /da3_install -name "depth_anything_v3_main" 2>/dev/null)
if [[ -n "${BINARY_CHECK}" ]]; then
    echo "  Binary verified: ${BINARY_CHECK}"
else
    echo "  ERROR: depth_anything_v3_main not found in volume."
    exit 1
fi

# ── Summary ─────────────────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  Setup complete!"
echo ""
echo "  USAGE:"
echo "  ┌─────────────────────────────────────────────────────────────┐"
echo "  │ Terminal 1 (DA V3 TRT inference):                           │"
echo "  │   docker compose -f docker/da3_trt.compose.yml up          │"
echo "  │                                                             │"
echo "  │ Terminal 2 (UGV navigation stack):                          │"
echo "  │   source install/setup.bash                                 │"
echo "  │   ros2 launch ugv_bringup ugv_complete.launch.py \          │"
echo "  │       mode:=sim camera_type:=da_v3_trt                     │"
echo "  └─────────────────────────────────────────────────────────────┘"
echo ""
echo "  NOTES:"
echo "  • First launch: TRT compiles ONNX → engine (~2-5 min, one-time)"
echo "  • Engine cached at: ${MODELS_DIR}/${MODEL_FILE%.onnx}.engine"
echo "  • To rebuild node: bash scripts/setup_da3_docker.sh --rebuild"
echo "══════════════════════════════════════════════════════════════════"
