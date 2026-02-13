###############################################################################
# edge_mission_controller - Docker image
# Base: ROS Noetic (Ubuntu 20.04)
# NOTE: il container si connette a un roscore GIA' in esecuzione sull'host.
#       Passare ROS_MASTER_URI e ROS_IP/ROS_HOSTNAME a runtime.
###############################################################################

FROM ros:noetic-ros-base

# Evita prompt interattivi durante l'installazione
ENV DEBIAN_FRONTEND=noninteractive

# ---------- Dipendenze di sistema e pacchetti ROS ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf-conversions \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-sbg-driver \
    && rm -rf /var/lib/apt/lists/*

# ---------- Dipendenze Python ----------
RUN pip3 install --no-cache-dir \
    gpxpy \
    psutil \
    utm

# ---------- Workspace catkin ----------
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

# ---- Pacchetti custom esterni (drone_sim, custom_msgs) ----
# Questi pacchetti forniscono messaggi usati dal codice:
#   - drone_sim      -> Telemetry.msg  (usato da mission_ext_interface.py)
#   - custom_msgs    -> SystemStatus.msg (usato da diagnostic.py)
#
# OPZIONE A: se i pacchetti sono in repository git, decommentare e adattare:
# RUN git clone https://<repo>/drone_sim.git      ${CATKIN_WS}/src/drone_sim
# RUN git clone https://<repo>/custom_msgs.git    ${CATKIN_WS}/src/custom_msgs
#
# OPZIONE B: copiare le cartelle dal contesto di build:
# COPY drone_sim/    ${CATKIN_WS}/src/drone_sim/
# COPY custom_msgs/  ${CATKIN_WS}/src/custom_msgs/
#
# OPZIONE C: se i pacchetti sono gia' installati nell'immagine base o
#            disponibili come .deb, aggiungerli sopra nella sezione apt.

# ---- Copia del pacchetto edge_mission_controller ----
COPY . ${CATKIN_WS}/src/edge_mission_controller/

# ---- Build ----
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && \
    cd ${CATKIN_WS} && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# ---------- Directory missioni (volume a runtime) ----------
RUN mkdir -p /home/usv/missions/mission_todo \
             /home/usv/missions/mission_done \
             /home/usv/missions/logs

# ---------- Entrypoint ----------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "edge_mission_controller", "mission_control.launch"]
