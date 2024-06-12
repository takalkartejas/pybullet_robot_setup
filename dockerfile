FROM python:3.8.18

#create working directory
WORKDIR /app

#Copy files form the local system
ADD /requirements.txt /app/requirements.txt

#change the default shell to bash

SHELL ["/bin/bash", "-c"]
# install libGL.so.1 shared library, which is part of the OpenGL implementation, nedded to run the taxim example code
RUN apt-get update
RUN apt-get install -y libgl1-mesa-glx
RUN ldconfig

RUN pip install --upgrade pip
# RUN pip install tacto
RUN pip install -r requirements.txt
#this needs to be installed for virtual display as physical display is unavailable
#use xvfb-run -a python taxim_setup.py to run a with virtual display
RUN apt-get install -y xvfb 