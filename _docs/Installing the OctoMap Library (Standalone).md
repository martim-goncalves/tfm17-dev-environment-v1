# Installing the OctoMap Library (Standalone)

--- 
## Performing a fresh install
The following commands may be used to perform a fresh install. First, any dependencies are installed or updated. Next, the library's source code is cloned from the source repository, compiled and installed using build tools.

```bash
sudo apt-get update && apt-get install -y \
build-essential \
cmake \
libqt5opengl5-dev \
libqglviewer-dev-qt5 
```

```bash
git clone https://github.com/OctoMap/octomap.git /opt/octomap && \
cd /opt/octomap && \
mkdir build && \
cd build && \
cmake .. && \
make && \
make install
```

--- 
## Updating the library
When updating the library, it isn't necessary to clone the repository again. Simply change into the directory again, pull from the repository, skip creating the `build/` directory and install as usual. 
```bash
cd /opt/octomap && \
git pull && \
cd build && \
cmake .. && \
make && \
make install
```

