# SPDX-License-Identifier: MIT
# This snippet install Chrono in ${PACKAGE_DIR}/chrono
# It includes other snippets, where specific modules can be added or removed based on need

ARG CHRONO_BRANCH="feature/fsi"
ARG CHRONO_REPO="https://github.jpl.nasa.gov/rsvp/chrono-wisc-mirror"
ARG CHRONO_DIR="${USERHOME}/chrono"
ARG CHRONO_INSTALL_DIR="${USERHOME}/packages/chrono"
ARG PACKAGE_DIR="${USERHOME}/packages"
ARG CHRONO_CUDA_ARCHITECTURE

RUN mkdir -p ${PACKAGE_DIR}

# This variable will be used by snippets to add cmake options
ENV CMAKE_OPTIONS=""
# This variable is used before building (but in the same RUN command)
# This is useful for setting environment variables that are used in the build process
ENV PRE_BUILD_COMMANDS=""

USER root

# Install Chrono dependencies that are required for all modules (or some but are fairly small)
RUN sudo apt update && \
        sudo apt install --no-install-recommends -y \
        libirrlicht-dev \
        libeigen3-dev \
        git \
        cmake \
        build-essential \
        ninja-build \
        swig \
        libxxf86vm-dev \
        freeglut3-dev \
        python3-numpy \
        libglu1-mesa-dev \
        libglew-dev \
        libglfw3-dev \
        libblas-dev \
        liblapack-dev \
        openssh-client \ 
	wget \
        ca-certificates \
	xorg-dev && \
        sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

#RUN mkdir -p -m 0700 /root/.ssh \
# && ssh-keyscan -H github.jpl.nasa.gov >> /root/.ssh/known_hosts

RUN --mount=type=ssh ssh-add -l || true
RUN --mount=type=ssh ssh -vvv -o StrictHostKeyChecking=accept-new -T git@github.jpl.nasa.gov || true

RUN --mount=type=ssh git clone --recursive git@github.jpl.nasa.gov:rsvp/chrono-wisc-mirror.git /home/chrono/chrono
RUN --mount=type=ssh git clone --recursive git@github.jpl.nasa.gov:rsvp/drive-primer.git /home/chrono/drive-primer


# Include the snippets which install shared dependencies
# These can be commented out or removed if they are no longer needed
INCLUDE ./cuda.dockerfile
#INCLUDE ./ros.dockerfile

# Then include the snippets for the modules you want to install
#INCLUDE ./ch_ros.dockerfile
INCLUDE ./ch_vsg.dockerfile
#INCLUDE ./ch_irrlicht.dockerfile
INCLUDE ./ch_vehicle.dockerfile
#INCLUDE ./ch_sensor.dockerfile
INCLUDE ./ch_parser.dockerfile
INCLUDE ./ch_fsi.dockerfile

#INCLUDE ./ch_python.dockerfile

# Install Chrono
RUN ${PRE_BUILD_SCRIPTS} && \
    # Evaluate the cmake options to expand any $(...) commands or variables
    eval "_CMAKE_OPTIONS=\"${CMAKE_OPTIONS}\"" && \
    mkdir ${CHRONO_DIR}/build && \
    cd ${CHRONO_DIR}/build && \
    cmake ../ -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_DEMOS=ON \
        -DBUILD_BENCHMARKING=OFF \
        -DBUILD_TESTING=OFF \
        -DCMAKE_LIBRARY_PATH=$(find /usr/local/cuda/ -type d -name stubs) \
        -DEigen3_DIR=/usr/lib/cmake/eigen3 \
        -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_DIR} \
        -DNUMPY_INCLUDE_DIR=$(python3 -c 'import numpy; print(numpy.get_include())') \
        ${_CMAKE_OPTIONS} \
        && \
    ninja && ninja install

RUN mkdir /home/chrono/drive-primer/image-data/build && \
	cd /home/chrono/drive-primer/image-data/build && \
	cmake .. -DCMAKE_BUILD_TYPE=Release && \
	make -j$(nproc)

RUN mkdir /home/chrono/drive-primer/cmars/build && \
	cd /home/chrono/drive-primer/cmars/build && \
	cmake .. \
 		 -DBUILD_SHARED_LIBS=OFF \
 		 -DCMAKE_BUILD_TYPE=Release \
 		 -DCMAKE_CUDA_ARCHITECTURES=${CHRONO_CUDA_ARCHITECTURE} \
		 -DCHRONO_CUDA_ARCHITECTURES=${CHRONO_CUDA_ARCHITECTURE} \
 		 -DChrono_DIR="${CHRONO_DIR}/build/cmake" \
 		 -DIMAGE_DATA_INCLUDES="/home/chrono/drive-primer/image-data" \
 		 -DIMAGE_DATA_LIB="/home/chrono/drive-primer/image-data/build/libimage_data.a" \
 		 -DvsgImGui_DIR="/home/chrono/packages/vsg/lib/cmake/vsgImGui" \
 		 -DvsgXchange_DIR="/home/chrono/packages/vsg/lib/cmake/vsgXchange" \
 		 -Dvsg_DIR="/home/chrono/packages/vsg/lib/cmake/vsg" && \
	make -j$(nproc)

RUN cd /home/chrono/drive-primer/param-identification && \
	pip install -e . && \
    cd /home/chrono/drive-primer/slip-table && \
	pip install -e .  && \
    cd /home/chrono/drive-primer/dp-cli && \
	pip install -e . 

RUN echo "export PATH=\$PATH:/home/chrono/drive-primer/cmars/build/demos" >> ${USERSHELLPROFILE}

# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${CHRONO_INSTALL_DIR}/lib" >> ${USERSHELLPROFILE}
