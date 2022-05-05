FROM uobflightlabstarling/starling-controller-base:latest

RUN apt update
RUN apt-get install -y ros-foxy-vision-opencv python3-pip
RUN pip3 install opencv-python

RUN pip3 install scipy
RUN pip3 install pandas
RUN pip3 install geopandas
RUN pip3 install shapely
RUN pip3 install pyshp
RUN pip3 install haversine
RUN pip3 install matplotlib
RUN pip3 install mapclassify
RUN pip3 install tqdm
RUN pip3 install pyvisgraph
RUN pip3 install turtle

RUN sudo apt-get install -y python3-tk
RUN pip3 install pillow --upgrade

COPY . /ros_ws/src/fenswood_drone_controller

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "launch", "fenswood_drone_controller", "controller.launch.xml" ]


