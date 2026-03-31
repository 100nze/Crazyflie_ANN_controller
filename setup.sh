#!/usr/bin/env bash
set -euo pipefail
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[OK]${NC}    $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $1"; }
fail() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAI_VERSION="4.0"
STAI_INSTALL_DIR="/opt/ST/STEdgeAI/${STAI_VERSION}"
STAI_LIB_SRC="${STAI_INSTALL_DIR}/Middlewares/ST/AI/Lib/GCC/ARMCortexM4/NetworkRuntime1200_CM4_GCC.a"
STAI_LIB_DST="${REPO_DIR}/src/ST_nn/NetworkRuntime1200_CM4_GCC.a"

if command -v arm-none-eabi-gcc &>/dev/null; then
    ok "arm-none-eabi-gcc: $(arm-none-eabi-gcc --version | head -1)"
else
    fail "arm-none-eabi-gcc not found. Install it with: sudo apt install gcc-arm-none-eabi"
fi

if command -v python3 &>/dev/null; then
    ok "python3: $(python3 --version)"
else
    warn "python3 not found. Required for the ground station scripts."
fi
echo ""

if [ ! -f "${REPO_DIR}/external/crazyflie-firmware/Makefile" ]; then
    git -C "${REPO_DIR}" submodule update --init --recursive
    ok "Submodules initialized"
else
    ok "Submodules already initialized"
fi

echo ""
if [ ! -f "${STAI_LIB_SRC}" ]; then
    fail "Library not found at:\n  ${STAI_LIB_SRC}\nMake sure ST Edge AI Core ${STAI_VERSION} is installed."
fi

cp "${STAI_LIB_SRC}" "${STAI_LIB_DST}"

echo -e "${GREEN}Setup complete.${NC}"
echo ""
echo "Next steps:"
echo "  1. source ~/.bashrc     "         
echo "  2. make cf2_defconfig  "       
echo "  3. make -j\$(nproc)      "      