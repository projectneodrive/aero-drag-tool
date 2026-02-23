#!/bin/bash
set -e

# ------------------------------
# 1. Install dependencies (Debian/Ubuntu/PopOS tested)
# ------------------------------
sudo apt update
sudo apt install -y \
    build-essential git python3 python3-pip python3-venv \
    swig \
    libopenblas-dev liblapack-dev libeigen3-dev \
    gfortran libglu1-mesa-dev freeglut3-dev mesa-common-dev \
    libx11-dev libxt-dev pkg-config

# ------------------------------
# 2. Create Python virtual environment
# ------------------------------
python3 -m venv ~/su2-venv
source ~/su2-venv/bin/activate

pip install --upgrade pip
pip install numpy mpi4py

# ------------------------------
# 3. Clone SU2 repository
# ------------------------------
cd ~
git clone https://github.com/su2code/SU2.git
cd SU2

git checkout v8.4.0
git submodule update --init --recursive

# ------------------------------
# 4. Preconfigure (ensures correct Meson and dependencies)
# ------------------------------
python3 ./preconfigure.py --with-own-meson

# ------------------------------
# 5. Configure build (CRITICAL: use meson.py, disable MPI)
# ------------------------------
./meson.py setup build \
    --prefix=$HOME/su2-install \
    -Dwith-mpi=enabled \
    -Denable-pywrapper=true \
    --buildtype=release

# ------------------------------
# 6. Compile and install
# ------------------------------
./ninja -C build install

# ------------------------------
# 7. Environment variables
# ------------------------------
echo "" >> ~/.bashrc
echo "# SU2 environment" >> ~/.bashrc
echo "export SU2_RUN=$HOME/su2-install/bin" >> ~/.bashrc
echo "export PATH=\$SU2_RUN:\$PATH" >> ~/.bashrc
echo "export PYTHONPATH=$HOME/su2-install/lib/python3/dist-packages:\$PYTHONPATH" >> ~/.bashrc

# Apply immediately
export SU2_RUN=$HOME/su2-install/bin
export PATH=$SU2_RUN:$PATH
export PYTHONPATH=$HOME/su2-install/lib/python3/dist-packages:$PYTHONPATH

# ------------------------------
# 8. Verify installation
# ------------------------------
echo ""
echo "Verifying SU2 installation..."
which SU2_CFD
SU2_CFD --help

echo ""
echo "SU2 installation complete."
echo "Restart terminal or run: source ~/.bashrc"