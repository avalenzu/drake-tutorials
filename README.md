# Drake Tutorials

This repository contains a collection of tutorials built on top of the binary
install of Drake. It uses the CMake `find_package(drake)` mechanism to find an
installed instance of Drake.

# Instructions

These instructions are only supported for Ubuntu 16.04 (Xenial).

```
# Install Prerequisites

# Install Drake to /opt/drake
curl -O https://s3.amazonaws.com/drake-packages/drake/nightly/drake-latest-xenial.tar.gz
sudo tar -xvzf drake-latest-xenial.tar.gz -C /opt

# Clone & Build Everything
git clone https://github.com/avalenzu/drake-tutorials.git
cd drake-tutorials
mkdir build && cd build
cmake ..
make

# Run a demo
/opt/drake/bin/drake-visualizer &
(cd src/kuka && exec ./passive_kuka_sim)         
