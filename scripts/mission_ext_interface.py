#!/usr/bin/env python3

# Mission External Interface
# Interfaccia tra il nodo mission_controller e l'esterno.
# Registra subscriber e callback per interpretare i comandi dalla stazione di terra,
# publisher per la pubblicazione dei dati di stato e telemetria del veicolo USV.
# Fornisce funzioni per il calcolo di SOG/COG, distanza dal goal, durata missione.
# Autore: Davide Domeneghetti
# Email: d.domenehetti@sitepitalia.it
# Versione: 1.1
# Data: 15/10/2025
# Note: Questo script richiede la libreria 'gpxpy'. Installala tramite 'pip install gpxpy'.

# TODO: aggiungere la lettura del topic della camera per lo streaming video.

import math
import os
import time
from collections import deque

import gpxpy
import rospy
from drone_sim.msg import Telemetry
from geometry_msgs.msg import Twist
from sbg_driver.msg import SbgGpsPos
from std_msgs.msg import Bool, Int32, String, Float32

# --- Costanti (default, sovrascritte da parametri ROS) ---
DEBUG = False
MS_TO_KN = 1.9438  # Fattore di conversione m/s -> nodi (costante fisica)
MEAN_SAMPLE = 5  # Numero di campioni per la media mobile
USV_MEAN_VEL = 1.02  # m/s corrispondente a 2 kn


class SlidingWindow:
    """Finestra scorrevole a dimensione fissa per il calcolo di medie mobili.

    Accumula valori fino a lenmax e poi scarta il piu' vecchio ad ogni push.
    """

    def __init__(self, lenmax):
        self.lenmax = lenmax
        self._buffer = deque(maxlen=self.lenmax)

    def push(self, value):
        """Aggiunge un valore alla finestra."""
        self._buffer.append(value)

    def mean_val(self):
        """Restituisce la media dei valori nella finestra.

        Nota: divide per lenmax (non per len attuale), quindi restituisce
        una media attenuata finche' la finestra non e' piena.
        """
        if not self._buffer:
            return 0.0
        return sum(self._buffer) / self.lenmax


class MissionExtInterface:
    """Interfaccia esterna per la missione USV.

    Gestisce la comunicazione con la stazione di terra:
    ricezione comandi, pubblicazione telemetria, calcolo SOG/COG.
    """

    def __init__(self, mission_file_path, drone_id):
        # Lettura parametri dal YAML (namespace assoluto, classe non standalone)
        global MEAN_SAMPLE, USV_MEAN_VEL
        MEAN_SAMPLE = rospy.get_param(
            '/mission_ext_interface/mean_sample', MEAN_SAMPLE
        )
        USV_MEAN_VEL = rospy.get_param(
            '/mission_ext_interface/usv_mean_vel', USV_MEAN_VEL
        )

        self.mission_file_path = mission_file_path
        self.prev_stamp = time.time()
        self.prev_pos = None
        self.istant_vel = 0.0
        self.mean_vel = 0.0
        self.mean_hdg = 0.0
        self.elapsed_time = 0.0
        self.vel_stack = SlidingWindow(MEAN_SAMPLE)
        self.hdg_stack = SlidingWindow(MEAN_SAMPLE)
        self.cc_cmd_vel = Twist()
        self.is_remote_control = False
        self.last_cmd_vel_time = None
        self.distance_to_goal = 0.0
        self.distance_to_finish = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0

        # Subscribers
        self.sub_cc_cmd_vel = rospy.Subscriber(
            f'/drone{drone_id}/cmd_vel', Twist, self.cmd_vel_cc_callback
        )
        self.sub_remote_request = rospy.Subscriber(
            f'/drone{drone_id}/get_cmd', Bool, self.control_required_callback
        )
        self.sub_mission_file = rospy.Subscriber(
            f'/drone{drone_id}/mission', String, self.mission_file_callback
        )
        self.sub_gps_pos = rospy.Subscriber(
            '/sbg/gps_pos', SbgGpsPos, self.gps_pos_callback
        )

        # Publishers telemetria
        self.pub_telemetry_data = rospy.Publisher(
            f'/drone{drone_id}/telemetry', Telemetry, queue_size=10
        )
        self.pub_status = rospy.Publisher(
            f'/drone{drone_id}/custom_topic_status', String, queue_size=10
        )
        self.pub_drone_id = rospy.Publisher(
            f'/drone{drone_id}/custom_topic_drone_id', Int32, queue_size=10,
            latch=True
        )
        # Pubblica drone_id una volta (latch mantiene il messaggio per nuovi subscriber)
        self.pub_drone_id.publish(Int32(data=drone_id))

    # ------------ CALLBACK ------------

    def mission_file_callback(self, msg):
        """Riceve e salva il file di missione GPX dal topic in mission_todo/."""
        rospy.loginfo("Ricevuto GPX, parsing in corso...")
        try:
            gpx = gpxpy.parse(msg.data)
            # Determina il nome del file dal nome della track
            if gpx.tracks and gpx.tracks[0].name:
                mission_name = gpx.tracks[0].name
            else:
                mission_name = 'mission_file'

            # Costruisci il path completo con os.path.join
            file_path = os.path.join(self.mission_file_path, f"{mission_name}.gpx")

            if DEBUG:
                for track in gpx.tracks:
                    for segment in track.segments:
                        for point in segment.points:
                            rospy.loginfo(
                                f"Punto: {point.latitude}, "
                                f"{point.longitude}, {point.elevation}"
                            )
            else:
                with open(file_path, 'w') as f:
                    f.write(msg.data)
                rospy.loginfo(
                    f"File di missione '{mission_name}.gpx' salvato in {self.mission_file_path}"
                )

        except Exception as e:
            rospy.logerr(f"Errore nel parsing del GPX: {e}")

    def cmd_vel_cc_callback(self, msg_twist):
        """Riceve i comandi di velocita' dal controllo remoto.

        Salva il timestamp dell'ultimo comando per il timeout.
        """
        self.cc_cmd_vel = msg_twist
        if rospy.core.is_initialized():
            self.last_cmd_vel_time = rospy.get_time()

    def control_required_callback(self, msg):
        """Riceve la richiesta di passaggio a controllo remoto."""
        self.is_remote_control = msg.data

    def gps_pos_callback(self, msg):
        """Riceve i dati di posizione GPS dal sensore SBG.

        Args:
            msg (SbgGpsPos): messaggio con latitudine e longitudine.
        """
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    # ------------ TELEMETRIA ------------

    def pub_telemetry(self, usv_status, now_pos):
        """Calcola e pubblica i dati di telemetria (SOG, COG, stato, GPS).

        Args:
            usv_status: stato corrente dell'USV (stringa).
            now_pos: posizione corrente come tupla (x, y) in coordinate cartesiane.
        """
        now_stamp = time.time()
        delta_time = now_stamp - self.prev_stamp

        if usv_status == "in navigazione":
            self.elapsed_time += delta_time
        else:
            self.elapsed_time = 0.0

        # Calcola SOG e COG solo se la posizione precedente e' disponibile
        if self.prev_pos is not None:
            # Usa coordinate cartesiane per il calcolo dell'heading
            self.get_mean_hdg_cart(self.prev_pos, now_pos)
            self.get_mean_vel(self.prev_pos, now_pos, delta_time)

        self.prev_pos = now_pos
        self.prev_stamp = now_stamp

        # Pubblica i dati verso la stazione di terra
        tel_msg = Telemetry()
        tel_msg.header.stamp = rospy.Time.now()
        tel_msg.heading = self.mean_hdg
        tel_msg.latitudine = self.current_lat
        tel_msg.longitudine = self.current_lon
        tel_msg.velocity = self.mean_vel
        self.pub_telemetry_data.publish(tel_msg)
        self.pub_status.publish(usv_status)

    # ------------ CALCOLO SOG (Speed Over Ground) ------------

    def get_istant_vel(self, prev_pos, now_pos, delta_t=1):
        """Calcola la velocita' istantanea tra due posizioni.

        Returns:
            Tupla (vel_ms, vel_kn) in m/s e nodi.
        """
        delta_s = self.dist(prev_pos, now_pos)

        if delta_s == 0 or delta_t == 0:
            self.istant_vel = (0.0, 0.0)
            return self.istant_vel

        vel_ms = delta_s / delta_t
        vel_kn = vel_ms * MS_TO_KN
        self.istant_vel = (vel_ms, vel_kn)
        self.vel_stack.push(vel_ms)

        return self.istant_vel

    def get_mean_vel(self, prev_pos, now_pos, delta_time=1):
        """Velocita' media su MEAN_SAMPLE campioni di velocita' istantanee."""
        self.get_istant_vel(prev_pos, now_pos, delta_time)
        self.mean_vel = self.vel_stack.mean_val()

    # ------------ CALCOLO COG (Course Over Ground) ------------

    def get_istant_hdg(self, prev_pos, now_pos):
        """Calcola il bearing istantaneo tra due posizioni GPS.

        Usa la formula:
        theta = atan2(sin(d_lon)*cos(lat2),
                      cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(d_lon))

        Returns:
            Heading in gradi [0, 360) dove Nord=0, Est=90.
        """
        lat1 = math.radians(prev_pos[0])
        lon1 = math.radians(prev_pos[1])
        lat2 = math.radians(now_pos[0])
        lon2 = math.radians(now_pos[1])

        delta_lon = lon2 - lon1

        num = math.sin(delta_lon) * math.cos(lat2)
        denom = (
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        )

        heading_rad = math.atan2(num, denom)
        heading_deg = math.degrees(heading_rad)

        # Normalizzazione a [0, 360)
        heading = (heading_deg + 360) % 360
        self.hdg_stack.push(heading)

        return heading

    def get_mean_hdg(self, prev_pos, now_pos):
        """Heading medio su MEAN_SAMPLE campioni (coordinate GPS)."""
        self.get_istant_hdg(prev_pos, now_pos)
        self.mean_hdg = self.hdg_stack.mean_val()

    def get_istant_hdg_cart(self, prev_pos, now_pos):
        """Calcola il heading istantaneo in coordinate cartesiane locali.

        Args:
            prev_pos: posizione precedente (x, y).
            now_pos: posizione corrente (x, y).

        Returns:
            Heading in gradi [0, 360) dove Nord=0, Est=90.
        """
        a = now_pos[0] - prev_pos[0]
        b = now_pos[1] - prev_pos[1]
        heading_rad = math.atan2(a, b)
        heading_deg = math.degrees(heading_rad)
        # Normalizzazione a [0, 360)
        heading = (heading_deg + 360) % 360
        self.hdg_stack.push(heading)
        return heading

    def get_mean_hdg_cart(self, prev_pos, now_pos):
        """Heading medio su MEAN_SAMPLE campioni (coordinate cartesiane)."""
        self.get_istant_hdg_cart(prev_pos, now_pos)
        self.mean_hdg = self.hdg_stack.mean_val()

    # ------------ INFORMAZIONI DI TELEMETRIA AGGIUNTIVE ------------

    def set_distance_to_goal(self, distance):
        """Imposta la distanza rimanente al waypoint corrente."""
        self.distance_to_goal = distance

    def set_distance_to_finish(self, total_distance):
        """Imposta la distanza rimanente alla fine della missione."""
        self.distance_to_finish = total_distance

    def get_time_to_completion(self):
        """Calcola il tempo stimato per completare la missione."""
        if self.mean_vel > 0:
            self.time_to_completion = self.distance_to_finish / self.mean_vel

    def get_time_to_wp(self):
        """Calcola il tempo stimato per raggiungere il waypoint corrente."""
        if self.mean_vel > 0:
            self.time_to_wp = self.distance_to_goal / self.mean_vel

    # ------------ UTILITA' ------------

    @staticmethod
    def dist(point1, point2):
        """Calcola la distanza euclidea tra due punti.

        Accetta sia oggetti con attributi .x/.y sia liste/tuple [x, y].
        """
        if hasattr(point1, 'x') and hasattr(point2, 'x'):
            x1, y1 = point1.x, point1.y
            x2, y2 = point2.x, point2.y
        elif isinstance(point1, (list, tuple)) and len(point1) >= 2:
            x1, y1 = point1[0], point1[1]
            x2, y2 = point2[0], point2[1]
        else:
            raise TypeError(
                "Formato punto non supportato. Usa [x, y] o oggetto con .x, .y"
            )

        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
