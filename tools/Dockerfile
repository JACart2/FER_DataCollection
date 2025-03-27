# Use the specified base image
FROM stereolabs/zed:4.2-runtime-cuda12.1-ubuntu22.04

# Set environment variables to ensure correct behavior for installation and system setup
ENV DEBIAN_FRONTEND=noninteractive


# NVIDIA install
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
RUN mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN wget https://developer.download.nvidia.com/compute/cuda/12.5.0/local_installers/cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb
RUN dpkg -i cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb
RUN cp /var/cuda-repo-ubuntu2204-12-5-local/cuda-*-keyring.gpg /usr/share/keyrings/
RUN apt-get update
RUN apt-get -y install cuda-toolkit-12-5
RUN apt-get install -y cuda-drivers


# Update package lists and install dependencies
RUN apt-get update 
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-dev
RUN apt-get install -y python3-setuptools
RUN apt-get install -y python3-tk 
RUN apt-get install -y build-essential
RUN apt-get install -y cmake
RUN apt-get install -y git
RUN apt-get install -y cmake
RUN apt-get install -y libgl1-mesa-glx
RUN apt install -y libglib2.0-0


RUN rm -rf /var/lib/apt/lists/*

# Install the FER package (using pip)
RUN pip3 install fer
RUN pip3 install moviepy==1.0.3
RUN pip3 install opencv-python
RUN pip3 install tensorflow
RUN pip3 install ffmpeg
RUN pip3 install argon2-cffi
RUN pip3 install python-dotenv
RUN pip3 install openai

# Install Auth packages
RUN pip3 install mysql-connector-python


# Install PyZed from the copied SDK path
RUN cd /usr/local/zed/ && \
    python3 get_python_api.py

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility


WORKDIR /src

RUN git clone https://github.com/JACart2/FER_DataCollection.git

RUN cd FER_DataCollection && \
    git pull

