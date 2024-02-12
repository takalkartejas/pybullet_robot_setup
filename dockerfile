FROM python:3.8.18

#create working directory
WORKDIR /app

#Copy files form the local system
ADD . .
#change the default shell to bash

SHELL ["/bin/bash", "-c"]

RUN pip install --upgrade pip


# install libGL.so.1 shared library, which is part of the OpenGL implementation, nedded to run the taxim example code
RUN apt-get update
RUN apt-get install -y libgl1-mesa-glx
RUN ldconfig