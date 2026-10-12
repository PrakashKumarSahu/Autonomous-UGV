#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# setup_da3_trt.sh — Set up Depth Anything V3 TensorRT for the UGV project
# ══════════════════════════════════════════════════════════════════════════════
#
# Run this script ONCE on a machine with an NVIDIA GPU to:
#   1. Check GPU + CUDA prerequisites
#   2. Install TensorRT (if not already installed)
#   3. Download the Depth Anything V3 ONNX model
#   4. Build the depth_anything_v3 ROS 2 package
#   5. (Optional) Pre-generate the TRT engine for faster first launch
#
# After this script completes, launch with:
#   ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim camera_type:=da_v3_trt
#
# Requirements:
#   - Ubuntu 22.04/24.04 with NVIDIA GPU
#   - CUDA 12.x/13.x drivers installed (nvidia-smi must work)
#   - ROS 2 Jazzy sourced (source /opt/ros/jazzy/setup.bash)
#   - Workspace built at least once (colcon build for the other packages)
#
# GPU tested: NVIDIA RTX 4050 Laptop (CUDA 13, TensorRT 10.x)
# ══════════════════════════════════════════════════════════════════════════════
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DA3_PKG="${REPO_ROOT}/third_party/ros2-depth-anything-v3-trt/depth_anything_v3"
MODELS_DIR="${DA3_PKG}/models"

echo "══════════════════════════════════════════════════════════════════"
echo "  Depth Anything V3 TensorRT — UGV Setup Script"
echo "══════════════════════════════════════════════════════════════════"
echo "  Repository root: ${REPO_ROOT}"
echo ""

# ── Step 1: Check GPU ─────────────────────────────────────────────────────────
echo "[1/5] Checking NVIDIA GPU..."
if ! command -v nvidia-smi &>/dev/null; then
    echo "  ERROR: nvidia-smi not found. Install NVIDIA drivers first."
    exit 1
fi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
echo ""

# ── Step 2: Check CUDA Toolkit (nvcc) ─────────────────────────────────────────
echo "[2/5] Checking CUDA Toolkit (nvcc)..."
if ! command -v nvcc &>/dev/null; then
    echo "  nvcc not found. Installing CUDA Toolkit..."
    echo "  NOTE: This installs cuda-toolkit from the NVIDIA Ubuntu repo."
    echo "        Adjust the version to match your driver (nvidia-smi shows CUDA version)."
    CUDA_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | \
               head -1 | awk -F. '{if($1>=550) print "12-6"; else print "12-4"}')
    sudo apt-get update -qq
    sudo apt-get install -y "cuda-toolkit-${CUDA_VER}" 2>/dev/null || \
    sudo apt-get install -y cuda-toolkit 2>/dev/null || {
        echo "  WARN: Could not auto-install CUDA toolkit via apt."
        echo "  Download manually from: https://developer.nvidia.com/cuda-downloads"
        echo "  Then re-run this script."
        exit 1
    }
    export PATH="/usr/local/cuda/bin:${PATH}"
    export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
    echo "  Add to ~/.bashrc:"
    echo "    export PATH=/usr/local/cuda/bin:\$PATH"
    echo "    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH"
else
    echo "  nvcc found: $(nvcc --version | grep release)"
fi
echo ""

# ── Step 3: Check / Install TensorRT ──────────────────────────────────────────
echo "[3/5] Checking TensorRT..."
TRT_HEADER=$(find /usr /opt -name "NvInfer.h" 2>/dev/null | head -1)
if [[ -z "${TRT_HEADER}" ]]; then
    echo "  NvInfer.h not found. Installing TensorRT..."
    echo "  TensorRT is available via:"
    echo "    A) NVIDIA Python wheel (pip) — easiest:"
    echo "       pip install tensorrt"
    echo "       (provides Python API only; C++ headers come with the wheel)"
    echo ""
    echo "    B) NVIDIA Developer Zone (full SDK with headers + libs):"
    echo "       https://developer.nvidia.com/tensorrt"
    echo ""
    echo "  Attempting pip install for latest compatible TensorRT..."
    python3 -m pip install --break-system-packages tensorrt 2>/dev/null || {
        echo "  WARNING: pip install tensorrt failed."
        echo "  Download the TensorRT tar from: https://developer.nvidia.com/tensorrt"
        echo "  Extract and set:"
        echo "    export TENSORRT_ROOT=/path/to/TensorRT-x.x.x"
        echo "    export LD_LIBRARY_PATH=\$TENSORRT_ROOT/lib:\$LD_LIBRARY_PATH"
        echo "  Then re-run this script."
        exit 1
    }
    # Locate headers installed by the wheel
    TRT_HEADER=$(python3 -c \
        "import tensorrt; import os; print(os.path.dirname(tensorrt.__file__))" \
        2>/dev/null)/include/NvInfer.h
    if [[ ! -f "${TRT_HEADER}" ]]; then
        # pip wheel puts headers in site-packages/tensorrt/include
        TRT_WHEEL_DIR=$(python3 -c \
            "import tensorrt; import os; print(os.path.dirname(tensorrt.__file__))")
        export TENSORRT_ROOT="${TRT_WHEEL_DIR}"
        echo "  TENSORRT_ROOT=${TENSORRT_ROOT}"
    fi
else
    echo "  TensorRT found at: ${TRT_HEADER}"
    export TENSORRT_ROOT=$(dirname $(dirname "${TRT_HEADER}"))
    echo "  TENSORRT_ROOT=${TENSORRT_ROOT}"
fi
echo ""

# ── Step 4: Download ONNX model ───────────────────────────────────────────────
echo "[4/5] Downloading Depth Anything V3 ONNX model..."
mkdir -p "${MODELS_DIR}"

# Available models (from HuggingFace: TillBeemelmanns/Depth-Anything-V3-ONNX):
#   DA3METRIC-SMALL.fp16.onnx  — faster (~15 FPS RTX 4050)  recommended
#   DA3METRIC-LARGE.fp16.onnx  — more accurate (~8 FPS RTX 4050)
MODEL_FILE="DA3METRIC-SMALL.fp16.onnx"
MODEL_PATH="${MODELS_DIR}/${MODEL_FILE}"
HF_BASE="https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX/resolve/main"

if [[ -f "${MODEL_PATH}" ]]; then
    echo "  Model already exists: ${MODEL_PATH}"
else
    echo "  Downloading ${MODEL_FILE} from HuggingFace..."
    if command -v wget &>/dev/null; then
        wget -q --show-progress -O "${MODEL_PATH}" "${HF_BASE}/${MODEL_FILE}"
    elif command -v curl &>/dev/null; then
        curl -L --progress-bar -o "${MODEL_PATH}" "${HF_BASE}/${MODEL_FILE}"
    else
        echo "  ERROR: wget or curl required. Install with: sudo apt install wget"
        exit 1
    fi
    echo "  Downloaded: ${MODEL_PATH}"
fi
echo ""

# ── Step 5: Build depth_anything_v3 package ───────────────────────────────────
echo "[5/5] Building depth_anything_v3 ROS 2 package..."

# Source ROS if not already sourced
if [[ -z "${ROS_DISTRO}" ]]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace install if present
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
    source "${REPO_ROOT}/install/setup.bash"
fi

# Set TRT/CUDA cmake paths if needed
CMAKE_EXTRA=""
if [[ -n "${TENSORRT_ROOT}" ]]; then
    CMAKE_EXTRA="-DTENSORRT_ROOT=${TENSORRT_ROOT}"
fi

cd "${REPO_ROOT}"
colcon build \
    --packages-select depth_anything_v3 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release ${CMAKE_EXTRA} \
    --symlink-install

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  Setup complete!"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "  Launch the UGV with Depth Anything V3 TensorRT:"
echo "    source ${REPO_ROOT}/install/setup.bash"
echo "    ros2 launch ugv_bringup ugv_complete.launch.py \\"
echo "        mode:=sim camera_type:=da_v3_trt"
echo ""
echo "  NOTE: On the FIRST launch, TRT will compile the engine from ONNX."
echo "        This takes 2-5 minutes. Subsequent launches load the cached engine."
echo ""
echo "  Engine will be cached at:"
echo "    ${MODELS_DIR}/${MODEL_FILE%.onnx}.engine"
echo ""
