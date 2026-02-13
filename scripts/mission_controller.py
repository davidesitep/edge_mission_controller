#!/usr/bin/env python3

# Mission Controller Node
# Viene lanciato all'avvio del sistema e si occupa di scegliere la missione e
# portarla avanti. Controlla in una specifica cartella se ci sono missioni da
# eseguire; se presenti avvia la piu' recente, altrimenti resta in attesa.
# Legge i waypoint dal file GPX e li invia uno ad uno a move_base.
# Autore: Davide Domeneghetti
# Email: d.domenehetti@sitepitalia.it
# Versione: 1.1
# Data: 10/09/2025
# Note: Questo script richiede la libreria 'gpxpy'. Installala tramite 'pip install gpxpy'.

# --- Standard library ---
import glob
import json
import math
import os
import sys

# Questo aggiunge la cartella 'scripts' al path dove Python cerca i file
script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)

import random
import time
from collections import deque
from typing import NamedTuple

# --- Third-party / ROS ---
import actionlib
import gpxpy
import psutil
import rospy
import tf2_geometry_msgs  # noqa: F401 - Necessario per tf_buffer.transform con PoseStamped
import tf2_ros
import tf_conversions
import utm
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseActionFeedback,
    MoveBaseGoal,
)
from nav_msgs.srv import GetPlan
from rosgraph import Master, MasterException
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Header, Int32, String

# --- Moduli locali ---
from diagnostic import UsvDiagnostic
from mission_ext_interface import MissionExtInterface

# Import USVLogger dalla directory 'utility' che è allo stesso livello di 'src'
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from utility.usv_logger import USVLogger

# --- Costanti (valori di default, sovrascritte da parametri ROS in __init__) ---
DEBUG = False
MISSION_BASE_PATH = "/home/usv/missions"
MISSION_TODO_PATH = os.path.join(MISSION_BASE_PATH, "mission_todo")
MISSION_DONE_PATH = os.path.join(MISSION_BASE_PATH, "mission_done")
LOG_PATH = os.path.join(MISSION_BASE_PATH, "logs")
MISSIONS_JSON_FILE = os.path.join(MISSION_DONE_PATH, "missions_history.json")
DRONE_ID = 1

# Timeout di sistema
CMD_VEL_TIMEOUT = 30        # Secondi senza comandi velocita' prima di tornare in idle
INACTIVITY_TIMEOUT = 900    # Secondi (15 min) di inattivita'
MB_SERVER_TIMEOUT = 5       # Secondi di attesa per connessione a move_base
PENDING_TIMEOUT = 120       # Secondi massimi in PENDING prima di re-inviare il goal
MAX_MBS_RETRY_CNT = 3       # Tentativi massimi di connessione a move_base
MAX_DOCKING_RETRIES = 3     # Tentativi massimi di docking prima di arrendersi

# Flag conversione coordinate
USE_GPS_WAYPOINTS = False

# Coordinate docking (formato dipende da USE_GPS_WAYPOINTS)
DOCKING_TARGET = (0.0, 0.0)

# Frame di riferimento
FRAME_ID_MAP = "map"
FRAME_ID_UTM = "utm"
FRAME_BASE_ID = "base_link"

TO_SEC = 1_000_000_000
ROSTIMEOUT = 600  # Secondi di attesa per roscore

# --- Mappe di stato ---
MAPPA_DEGLI_STATI_USV = {
    0: "In attesa",
    1: "Avvio nav. autonoma",
    2: "Navigazione autonoma",
    3: "Navigazione da remoto",
    4: "Errore minore",
    5: "Errore critico",
    6: "Docking",
    7: "In attesa comando remoto"
}

MAPPA_MOVE_BASE_STATUS = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
   -1: "UNKNOWN"
}

MAPPA_ERRORI = {
    0: "sensore fuori uso",
    1: "motore fuori uso",
    2: "errore planner",
    3: "errore del localizzatore"
}

MAPPA_ERRORI_PLANNER = {
    0: "Errore del navigation server",
    1: "Errore di calcolo percorso"
}

MAPPA_ERRORE_SENSORI = {
    0: "IMU fuori uso",
    1: "GPS fuori uso",
    2: "Bussola fuori uso",
    3: "Impossibile calcolare l'odometria"
}


class MissionInfo(NamedTuple):
    """Struttura con informazioni sulle missioni eseguite."""

    mission_id: str
    mission_status: str
    total_waypoints: int
    waypoint_reached: int
    total_distance: float
    distance_traveled: float
    mission_duration: float  # Secondi


# =============================================================================
# MISSION CONTROLLER
# =============================================================================

class MissionController:
    """Nodo principale per il controllo delle missioni dell'USV.

    Implementa una macchina a stati (FSM) con 7 stati per gestire:
    navigazione autonoma, controllo remoto, gestione errori e docking.
    """

    def __init__(self):
        # Variabili di missione
        self.mission_files = None
        self.waypoints = None
        self.mission_directory = MISSION_TODO_PATH  # Cerca missioni in mission_todo/
        self.usv_status = MAPPA_DEGLI_STATI_USV[0]
        self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]  # UNKNOWN
        self.move_base_client = None
        self.mission_index = 0
        self.preemption_counter = 0
        self.new_mission_id = None
        self.current_mission_file = None  # Path reale del file GPX caricato
        self.errori_minori = []
        self.sensor_error = False
        self.planner_error = False
        self.consecutive_aborts = 0
        self.docking_goal_sent = False
        self.docking_retry_count = 0
        self.docking_target = DOCKING_TARGET
        self.docking_completed = False
        self.missions = []
        self.load_missions_from_json()
        self.waypoints_reached = 0
        self.can_jump = False
        self.lunghezza_segmenti = []
        self.prev_stamp = time.time()
        self.durata_missione = 0.0
        self.prev_pos = (0.0, 0.0)
        self.vel_media = deque(maxlen=10)

        # Variabili di stato
        self.is_valid_mission = False
        self.is_remote_control = False
        self.autonomous_nav_failed = False  # True quando in stato 7
        self.is_timed_out = False
        self.nav_system_active = [False, False]  # [move_base, tf]
        self.retry_cnt = [1, 1]  # [move_base, tf]
        self.cc_cmd_vel = Twist()
        self.move_base_cmd_vel = Twist()
        self.mc_cmd_vel = Twist()
        self.minor_error = False
        self.major_error = False
        self.startup_retry_count = 0  # Contatore tentativi connessione in stato 1
        self.system_status = None
        self.current_position = None
        self.current_heading = None
        self._perf_counter = 0

        # Inizializzazione nodo ROS (deve precedere publishers/subscribers/tf)
        rospy.init_node('mission_controller', anonymous=True)

        # --- Lettura parametri ROS (sovrascrivono i default a livello modulo) ---
        self._load_ros_params()

        # TF (richiede nodo ROS inizializzato)
        self.tf_buffer = tf2_ros.Buffer()

        # Classi di supporto
        self.usv_diagnostic = UsvDiagnostic()
        self.gnd_station_if = MissionExtInterface(
            MISSION_TODO_PATH, DRONE_ID  # Salva missioni ricevute in mission_todo/
        )

        # Publishers
        self.pub_status = rospy.Publisher(
            '/usv_status', Int32, queue_size=10
        )
        self.pub_vel_cmd = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10
        )
        self.pub_mc_cmd_vel = rospy.Publisher(
            '/mission_controller/cmd_vel', Twist, queue_size=10
        )
        self.gps_goal_2convert = rospy.Publisher(
            '/initial_navsat_fix', NavSatFix, queue_size=1
        )
        self.pub_abort_current_goal = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=1
        )
        self.heartbeat = rospy.Publisher(
            '/mission_controller/heartbeat', Header, queue_size=10
        )

        # Subscribers
        self.sub_cc_cmd_vel = rospy.Subscriber(
            f'/drone{DRONE_ID}/cmd_vel', Twist, self.cmd_vel_cc_callback
        )
        self.sub_move_base_cmd_vel = rospy.Subscriber(
            '/move_base/cmd_vel', Twist, self.cmd_vel_move_base_callback
        )
        self.sub_goal_feedback = rospy.Subscriber(
            '/move_base/feedback', MoveBaseActionFeedback,
            self.usv_feedback_callback
        )
        self.sub_remote_request = rospy.Subscriber(
            f'/drone{DRONE_ID}/get_cmd', Bool, self.remote_cmd_callback
        )
        self.sub_nav_status = None  # Creato in _handle_state_startup

        self.last_cmd_vel_time = None
        self.last_cmd_vel_time_move_base = None
        self.pending_time = None
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Timer FSM a 1 Hz
        self.timer = rospy.Timer(rospy.Duration(1.0), self.run)

        # Logger
        self.logger = USVLogger(
            log_dir=LOG_PATH,  # Salva log in mission_test/logs/
            node_name="mission_controller"
        )
        self.logger.system_logger.info("=" * 50)
        self.logger.system_logger.info("Mission Controller avviato")
        self.logger.system_logger.info("=" * 50)

    # ------------ CARICAMENTO PARAMETRI ROS ------------

    def _load_ros_params(self):
        """Carica parametri dal server ROS e sovrascrive le costanti a livello modulo.

        I parametri del veicolo e i path vengono dal launch file (<param>).
        Timeout e soglie vengono dal YAML (caricato con <rosparam>).
        I valori di default sono quelli definiti in cima al file.
        """
        global DEBUG, DRONE_ID, USE_GPS_WAYPOINTS, DOCKING_TARGET
        global MISSION_BASE_PATH, MISSION_TODO_PATH, MISSION_DONE_PATH
        global LOG_PATH, MISSIONS_JSON_FILE
        global FRAME_ID_MAP, FRAME_ID_UTM, FRAME_BASE_ID
        global CMD_VEL_TIMEOUT, INACTIVITY_TIMEOUT, MB_SERVER_TIMEOUT
        global PENDING_TIMEOUT, MAX_MBS_RETRY_CNT, MAX_DOCKING_RETRIES
        global ROSTIMEOUT

        # --- Parametri dal launch file ---
        MISSION_BASE_PATH = rospy.get_param('~mission_dir', MISSION_BASE_PATH)
        MISSION_TODO_PATH = os.path.join(MISSION_BASE_PATH, "mission_todo")
        MISSION_DONE_PATH = os.path.join(MISSION_BASE_PATH, "mission_done")
        LOG_PATH = os.path.join(MISSION_BASE_PATH, "logs")
        MISSIONS_JSON_FILE = os.path.join(MISSION_DONE_PATH, "missions_history.json")

        DRONE_ID = rospy.get_param('~drone_id', DRONE_ID)
        DEBUG = rospy.get_param('~debug', DEBUG)
        USE_GPS_WAYPOINTS = rospy.get_param('~use_gps_waypoints', USE_GPS_WAYPOINTS)

        dock_x = rospy.get_param('~docking_target_x', DOCKING_TARGET[0])
        dock_y = rospy.get_param('~docking_target_y', DOCKING_TARGET[1])
        DOCKING_TARGET = (dock_x, dock_y)

        FRAME_ID_MAP = rospy.get_param('~frame_map', FRAME_ID_MAP)
        FRAME_ID_UTM = rospy.get_param('~frame_utm', FRAME_ID_UTM)
        FRAME_BASE_ID = rospy.get_param('~frame_base', FRAME_BASE_ID)

        # --- Parametri dal YAML (caricati nel namespace del nodo) ---
        CMD_VEL_TIMEOUT = rospy.get_param('~cmd_vel_timeout', CMD_VEL_TIMEOUT)
        INACTIVITY_TIMEOUT = rospy.get_param('~inactivity_timeout', INACTIVITY_TIMEOUT)
        MB_SERVER_TIMEOUT = rospy.get_param('~mb_server_timeout', MB_SERVER_TIMEOUT)
        PENDING_TIMEOUT = rospy.get_param('~pending_timeout', PENDING_TIMEOUT)
        MAX_MBS_RETRY_CNT = rospy.get_param('~max_mbs_retry', MAX_MBS_RETRY_CNT)
        MAX_DOCKING_RETRIES = rospy.get_param('~max_docking_retries', MAX_DOCKING_RETRIES)
        ROSTIMEOUT = rospy.get_param('~rostimeout', ROSTIMEOUT)

        # Crea le directory se non esistono
        os.makedirs(MISSION_TODO_PATH, exist_ok=True)
        os.makedirs(MISSION_DONE_PATH, exist_ok=True)
        os.makedirs(LOG_PATH, exist_ok=True)

        # Aggiorna riferimenti interni
        self.mission_directory = MISSION_TODO_PATH
        self.docking_target = DOCKING_TARGET

        rospy.loginfo(f"Parametri caricati: mission_dir={MISSION_BASE_PATH}, "
                      f"drone_id={DRONE_ID}, debug={DEBUG}, "
                      f"use_gps={USE_GPS_WAYPOINTS}")

    # ------------ FUNZIONI INTERNE ------------

    def start_control_node(self):
        """Avvia il nodo e gestisce i callback ROS.

        NOTA: se use_sim_time e' attivo (simulazione), rospy.Timer usa il
        clock pubblicato su /clock da Gazebo. Senza Gazebo attivo il Timer
        non scatta e run() non viene mai chiamato.
        """
        if rospy.get_param('/use_sim_time', False):
            clock_topics = [t for t, _ in rospy.get_published_topics()
                            if t == '/clock']
            if not clock_topics:
                rospy.logwarn(
                    "use_sim_time=True ma /clock non pubblicato. "
                    "Il Timer FSM non scattera' finche' un nodo "
                    "(es. Gazebo) non pubblica /clock."
                )
        rospy.loginfo("Mission controller avviato.")
        rospy.spin()

    def jump_to_state(self, state):
        """Permette di rieseguire il loop prima che scada il timer.

        Args:
            state: indice dello stato (0-6) in MAPPA_DEGLI_STATI_USV.
        """
        if state in MAPPA_DEGLI_STATI_USV:
            self.usv_status = MAPPA_DEGLI_STATI_USV[state]
            self.can_jump = True
        else:
            rospy.logerr(f"Stato {state} non valido.")

    # ------------ CALLBACK ------------

    def cmd_vel_cc_callback(self, msg_twist):
        """Riceve comandi di velocita' dal controllo remoto.

        In stato remoto (3), rilancia immediatamente su /cmd_vel per preservare
        la frequenza di aggiornamento del GCS senza dover aspettare il tick FSM.
        """
        self.cc_cmd_vel = msg_twist
        if rospy.core.is_initialized():
            has_velocity = (
                msg_twist.linear.x != 0.0 or msg_twist.linear.y != 0.0
                or msg_twist.angular.z != 0.0
            )
            if has_velocity:
                self.last_cmd_vel_time = rospy.get_time()
            if self.usv_status in (
                MAPPA_DEGLI_STATI_USV[3], MAPPA_DEGLI_STATI_USV[7]
            ):
                self.pub_vel_cmd.publish(msg_twist)

    def remote_cmd_callback(self, msg):
        """Riceve la richiesta di passaggio a controllo remoto."""
        self.is_remote_control = msg.data

    def cmd_vel_move_base_callback(self, msg_twist):
        """Riceve comandi di velocita' da move_base.

        In navigazione autonoma (stati 2 e 6), rilancia immediatamente su /cmd_vel
        per preservare la frequenza del planner senza dover aspettare il tick FSM.
        """
        self.move_base_cmd_vel = msg_twist
        if rospy.core.is_initialized():
            self.last_cmd_vel_time_move_base = rospy.get_time()
            if self.usv_status in (MAPPA_DEGLI_STATI_USV[2], MAPPA_DEGLI_STATI_USV[6]):
                self.pub_vel_cmd.publish(msg_twist)

    def usv_status_callback(self, msg):
        """Callback per lo stato di move_base."""
        if msg.header.stamp:
            self.planner_active = True

    def usv_feedback_callback(self, msg):
        """Callback per il feedback di navigazione (non implementato)."""
        pass

    def nav_result_callback(self, msg):
        """Aggiorna lo stato di navigazione dal topic /move_base/status."""
        if not msg or not msg.status_list:
            return
        latest_status_code = msg.status_list[-1].status
        self.navigation_status = MAPPA_MOVE_BASE_STATUS.get(
            latest_status_code, MAPPA_MOVE_BASE_STATUS[-1]
        )

    # ------------ DIAGNOSTICA ------------

    def check_system_status(self):
        """Interpreta lo stato dei sensori e imposta i flag di errore."""
        sensor_fail = [False, False]  # [GPS, IMU]

        if self.usv_diagnostic.status.sensor_presence:
            # --- GPS ---
            if self.usv_diagnostic.status.gps_presence:
                sensor_fail[0] = not self.usv_diagnostic.status.is_gpspos
                if sensor_fail[0]:
                    rospy.logwarn("GPS fail: impossibile calcolare la soluzione")

                if self.usv_diagnostic.status.is_gpsvel:
                    rospy.loginfo(
                        f"GPS: qualita' soluzione velocita': "
                        f"{self.usv_diagnostic.status.gps_vel_type}"
                    )

                if self.usv_diagnostic.status.is_gps_hdt:
                    rospy.loginfo("GPS: HDT disponibile per il calcolo del COG.")
                else:
                    rospy.logwarn(
                        "GPS: COG da GPS non disponibile, COG calcolata dal planner."
                    )

                rospy.loginfo(
                    f"GPS: qualita' segnale: "
                    f"{self.usv_diagnostic.status.gps_type}"
                )
            else:
                sensor_fail[0] = True
                rospy.logerr(
                    "GPS interno al sensore Ellipse non disponibile."
                )

            # --- IMU ---
            if self.usv_diagnostic.status.imu_presence:
                sensor_fail[1] = not self.usv_diagnostic.status.is_imu
                if sensor_fail[1]:
                    rospy.logwarn("IMU fail: soluzione non affidabile.")
            else:
                sensor_fail[1] = True
                rospy.logerr(
                    "IMU interna al sensore Ellipse non disponibile."
                )

            self.sensor_error = any(sensor_fail)
            if self.sensor_error and "sensore fuori uso" not in self.errori_minori:
                self.errori_minori.append("sensore fuori uso")
            elif not self.sensor_error and "sensore fuori uso" in self.errori_minori:
                self.errori_minori.remove("sensore fuori uso")
        else:
            self.sensor_error = True
            if "sensore fuori uso" not in self.errori_minori:
                self.errori_minori.append("sensore fuori uso")
            rospy.logerr("Sensore Ellipse non disponibile.")

        # --- Motore ---
        if not self.usv_diagnostic.status.is_motor_ok:
            self.major_error = True
            if "motore fuori uso" not in self.errori_minori:
                self.errori_minori.append("motore fuori uso")
            rospy.logerr("Rilevato guasto motore!")

    # ------------ MULTIPLEXER VELOCITA' ------------

    def vel_multiplexer(self):
        """Seleziona il comando di velocita' in base allo stato corrente.

        Stato 0, 1      -> Twist() zero (idle / avvio, nessun movimento)
        Stato 2, 6      -> move_base/cmd_vel (navigazione autonoma e docking)
        Stato 3, 7      -> CC/cmd_vel (controllo remoto GCS)
        Stato 4, 5      -> mc_cmd_vel (inibizione diretta dal mission controller)

        Nota: per gli stati 2, 3 e 6 il relay ad alta frequenza avviene
        direttamente nei rispettivi callback; questo metodo viene usato
        dagli handler degli stati 4/5 e come riferimento della logica.
        """
        if self.usv_status in (MAPPA_DEGLI_STATI_USV[2], MAPPA_DEGLI_STATI_USV[6]):
            return self.move_base_cmd_vel
        elif self.usv_status in (
            MAPPA_DEGLI_STATI_USV[3], MAPPA_DEGLI_STATI_USV[7]
        ):
            return self.cc_cmd_vel
        elif self.usv_status in (MAPPA_DEGLI_STATI_USV[4], MAPPA_DEGLI_STATI_USV[5]):
            return self.mc_cmd_vel
        else:
            return Twist()

    # ------------ GESTIONE MISSIONI (FILE GPX) ------------

    def search_for_mission(self, mission_directory):
        """Cerca file GPX nella cartella specificata."""
        if not os.path.isdir(mission_directory):
            rospy.logerr(f"Directory {mission_directory} non trovata.")
            return None
        return glob.glob(os.path.join(mission_directory, '*.gpx'))

    def check_last_mission(self, mission_files):
        """Restituisce il file di missione piu' recente."""
        if not mission_files:
            rospy.loginfo("Nessun file GPX trovato.")
            return None
        return max(mission_files, key=os.path.getctime)

    def load_mission(self, mission_path):
        """Carica la missione dal file GPX.

        Returns:
            Numero di waypoint caricati, oppure 0 in caso di errore.
        """
        try:
            rospy.loginfo(f"Caricamento della missione da {mission_path}...")
            self.current_mission_file = mission_path
            with open(mission_path, 'r') as file:
                gpx_content = file.read()
                gpx = gpxpy.parse(gpx_content)
                self.waypoints = [
                    (point.latitude, point.longitude)
                    for track in gpx.tracks
                    for segment in track.segments
                    for point in segment.points
                ]
                if gpx.tracks and gpx.tracks[0].name:
                    self.new_mission_id = gpx.tracks[0].name
                elif gpx.name:
                    self.new_mission_id = gpx.name
                else:
                    self.new_mission_id = os.path.splitext(
                        os.path.basename(mission_path)
                    )[0]

                if DEBUG:
                    rospy.loginfo(
                        f"Missione caricata con {len(self.waypoints)} waypoints."
                    )
                return len(self.waypoints)

        except Exception as e:
            rospy.logerr(f"Errore nel caricamento della missione: {e}")
            self._handle_corrupted_mission(mission_path, e)
            return 0

    def _handle_corrupted_mission(self, mission_path, error):
        """Gestisce un file GPX corrotto: lo rimuove e lo registra nel JSON."""
        # Ricava mission_id dal nome del file
        corrupted_id = os.path.splitext(
            os.path.basename(mission_path)
        )[0]
        rospy.logerr(
            f"Missione corrotta '{corrupted_id}': {error}. "
            f"File rimosso da mission_todo/."
        )

        # Registra nel JSON come "corrupted"
        mission = MissionInfo(
            mission_id=corrupted_id,
            mission_status="corrupted",
            total_waypoints=0,
            waypoint_reached=0,
            total_distance=0.0,
            distance_traveled=0.0,
            mission_duration=0.0
        )
        self.missions.append(mission)
        self.save_missions_to_json()

        # Rimuovi il file corrotto
        try:
            os.remove(mission_path)
            self.logger.log_mission(
                f"Missione '{corrupted_id}' corrotta, "
                f"file GPX rimosso da mission_todo/"
            )
        except Exception as rm_err:
            rospy.logerr(
                f"Impossibile rimuovere file corrotto "
                f"{mission_path}: {rm_err}"
            )
        self.current_mission_file = None

    # ------------ PERSISTENZA MISSIONI (JSON) ------------

    def load_missions_from_json(self):
        """Carica lo storico delle missioni dal file JSON."""
        try:
            if os.path.isfile(MISSIONS_JSON_FILE):
                with open(MISSIONS_JSON_FILE, 'r') as f:
                    missions_data = json.load(f)
                    self.missions = [
                        MissionInfo(**m) for m in missions_data
                    ]
                    rospy.loginfo(
                        f"Caricate {len(self.missions)} missioni dal JSON."
                    )
            else:
                rospy.loginfo(
                    "File di storia missioni non trovato. Lista vuota."
                )
                self.missions = []
        except Exception as e:
            rospy.logerr(
                f"Errore nel caricamento delle missioni dal JSON: {e}"
            )
            self.missions = []

    def save_missions_to_json(self):
        """Salva lo storico delle missioni nel file JSON."""
        try:
            missions_data = [m._asdict() for m in self.missions]
            os.makedirs(os.path.dirname(MISSIONS_JSON_FILE), exist_ok=True)
            with open(MISSIONS_JSON_FILE, 'w') as f:
                json.dump(missions_data, f, indent=4)
            rospy.loginfo(
                f"Missioni salvate nel file JSON: {MISSIONS_JSON_FILE}"
            )
        except Exception as e:
            rospy.logerr(
                f"Errore nel salvataggio delle missioni nel JSON: {e}"
            )

    def save_mission_info(self, mission_status="completed"):
        """Salva informazioni della missione corrente nel registro JSON.

        Se la missione e' completata o fallita, elimina il file GPX da mission_todo/.
        """
        pose = self.get_usv_pose()
        if pose and self.waypoints:
            dist_to_start = math.sqrt(
                (pose[0] - self.waypoints[0][0]) ** 2
                + (pose[1] - self.waypoints[0][1]) ** 2
            )
        else:
            dist_to_start = 0.0

        lunghezza_totale = sum(self.lunghezza_segmenti) + dist_to_start

        mission = MissionInfo(
            mission_id=self.new_mission_id,
            mission_status=mission_status,
            total_waypoints=len(self.waypoints) if self.waypoints else 0,
            waypoint_reached=self.waypoints_reached,
            total_distance=lunghezza_totale,
            distance_traveled=0.0,
            mission_duration=0.0  # TODO: implementare durata
        )

        # Controlla se la missione e' gia' presente
        existing_ids = [m.mission_id for m in self.missions]
        if self.new_mission_id in existing_ids:
            for idx, m in enumerate(self.missions):
                if m.mission_id == self.new_mission_id:
                    if m.mission_status != "completed":
                        self.missions.pop(idx)
                        self.missions.append(mission)
                        rospy.loginfo(
                            f"Aggiornato stato missione "
                            f"{self.new_mission_id} a {mission_status}."
                        )
                    else:
                        rospy.loginfo(
                            f"Missione {self.new_mission_id} gia' registrata "
                            f"come {mission_status}."
                        )
                    break
        else:
            self.missions.append(mission)
            rospy.loginfo(
                f"Registrata nuova missione {self.new_mission_id} "
                f"con stato {mission_status}."
            )

        self.save_missions_to_json()

        # Rimuovi file GPX da mission_todo/ se missione conclusa (completata o fallita)
        if mission_status in ["completed", "failed"] and self.current_mission_file:
            if os.path.exists(self.current_mission_file):
                try:
                    os.remove(self.current_mission_file)
                    rospy.loginfo(
                        f"File missione {self.current_mission_file} rimosso "
                        f"(status: {mission_status})."
                    )
                    self.logger.log_mission(
                        f"Missione {self.new_mission_id} conclusa ({mission_status}), "
                        f"file GPX rimosso da mission_todo/"
                    )
                    self.current_mission_file = None
                except Exception as e:
                    rospy.logerr(
                        f"Errore nella rimozione del file "
                        f"{self.current_mission_file}: {e}"
                    )
            else:
                rospy.logwarn(
                    f"File missione {self.current_mission_file} "
                    f"non trovato per la rimozione."
                )

    def update_mission_info(self, index, new_mission_status):
        """Aggiorna lo stato di una missione esistente nel registro."""
        mission_to_update = self.missions[index]
        pose = self.get_usv_pose()
        current_pos = pose[:2] if pose else (0.0, 0.0)
        new_mission = mission_to_update._replace(
            mission_status=new_mission_status,
            waypoint_reached=self.waypoints_reached,
            distance_traveled=(
                mission_to_update.total_distance
                - self.distanza_rimanente(current_pos)
            )
        )
        self.missions[index] = new_mission
        rospy.loginfo("Missione aggiornata")

    # ------------ POSE E POSIZIONE ------------

    def get_usv_pose(self):
        """Calcola la posizione dell'USV nel frame map.

        Returns:
            Tupla (x, y, yaw) oppure None se la TF non e' disponibile.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rospy.Time(0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            q = trans.transform.rotation
            euler = tf_conversions.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            yaw = euler[2]

            return (x, y, yaw)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn("Trasformazione map->base_link non disponibile.")
            return None

    def get_usv_pos_global(self, x_map, y_map):
        """Converte coordinate map in WGS84 lat/long.

        Args:
            x_map: coordinata X nel frame map.
            y_map: coordinata Y nel frame map.

        Returns:
            Tupla (lat, long).

        Note:
            Richiede self.origin (coordinate UTM dell'origine del frame map)
            che deve essere impostato dalla trasformata navsat_transform_node.
        """
        # TODO: self.origin deve essere calcolato dalla TF utm->map
        zone_number = 32
        zone_letter = 'T'

        x_utm = x_map + self.origin[0]
        y_utm = y_map + self.origin[1]

        lat, lon = utm.to_latlon(x_utm, y_utm, zone_number, zone_letter)
        return lat, lon

    # ------------ GEOMETRIA E NAVIGAZIONE ------------

    @staticmethod
    def calculate_pose(waypoint1, waypoint2):
        """Calcola l'orientamento in quaternioni tra due punti cartesiani.

        Returns:
            Tupla (qx, qy, qz, qw).
        """
        delta_x = waypoint2[0] - waypoint1[0]
        delta_y = waypoint2[1] - waypoint1[1]
        yaw = math.atan2(delta_y, delta_x)

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def convert_gps_to_map(self, lat, lon):
        """Converte coordinate GPS (lat/lon) in coordinate (x, y) nel frame 'map'.

        Converte lat/lon in UTM tramite la libreria utm, poi usa la trasformata
        TF utm->map (pubblicata da navsat_transform_node) per ottenere le
        coordinate nel frame map.

        Args:
            lat: latitudine in gradi decimali.
            lon: longitudine in gradi decimali.

        Returns:
            Tupla (x_map, y_map) oppure None se la conversione fallisce.
        """
        easting, northing, _, _ = utm.from_latlon(lat, lon)
        rospy.loginfo(
            f"GPS ({lat:.6f}, {lon:.6f}) -> "
            f"UTM (E={easting:.2f}, N={northing:.2f})"
        )

        # Crea PoseStamped nel frame UTM con coordinate UTM pure
        pose_utm = PoseStamped()
        pose_utm.header.stamp = rospy.Time.now()
        pose_utm.header.frame_id = FRAME_ID_UTM
        pose_utm.pose.position.x = easting
        pose_utm.pose.position.y = northing
        pose_utm.pose.position.z = 0.0
        pose_utm.pose.orientation.w = 1.0

        try:
            # Trasforma da utm a map usando la TF pubblicata da navsat_transform_node
            pose_map = self.tf_buffer.transform(
                pose_utm, FRAME_ID_MAP, rospy.Duration(5.0)
            )
            x_map = pose_map.pose.position.x
            y_map = pose_map.pose.position.y
            rospy.loginfo(
                f"Conversione UTM -> map: X={x_map:.2f}, Y={y_map:.2f}"
            )
            return (x_map, y_map)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Errore trasformazione utm->map: {e}")
            return None

    def _convert_all_waypoints_to_map(self):
        """Converte tutti i waypoint da GPS (lat, lon) a coordinate map (x, y).

        Chiamato una sola volta all'inizio della navigazione (stato 2), quando
        la TF utm->map e' disponibile. Sovrascrive self.waypoints in-place.
        Converte anche self.docking_target.

        Returns:
            True se la conversione e' riuscita per tutti i waypoint, False altrimenti.
        """
        if not USE_GPS_WAYPOINTS:
            return True

        rospy.loginfo(
            f"Conversione {len(self.waypoints)} waypoint GPS -> map..."
        )
        converted = []
        for i, wp in enumerate(self.waypoints):
            result = self.convert_gps_to_map(wp[0], wp[1])
            if result is None:
                rospy.logerr(
                    f"Conversione fallita per waypoint {i}: "
                    f"({wp[0]:.6f}, {wp[1]:.6f})"
                )
                return False
            converted.append(result)

        self.waypoints = converted
        rospy.loginfo(
            f"Conversione completata: {len(self.waypoints)} waypoint "
            f"convertiti in frame map."
        )

        # Converti anche le coordinate di docking
        dock_result = self.convert_gps_to_map(
            self.docking_target[0], self.docking_target[1]
        )
        if dock_result is not None:
            self.docking_target = dock_result
            rospy.loginfo(
                f"Docking target convertito: "
                f"X={dock_result[0]:.2f}, Y={dock_result[1]:.2f}"
            )
        else:
            rospy.logwarn(
                "Conversione docking target fallita. "
                "Verra' ritentata all'ingresso in stato 6."
            )

        return True

    # ------------ HELPER MOVE_BASE_GOAL ------------

    def _create_move_base_goal(self, x, y, seq=0):
        """Crea un MoveBaseGoal nel frame 'map'.

        Args:
            x: coordinata X nel frame map.
            y: coordinata Y nel frame map.
            seq: numero di sequenza per l'header.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.seq = seq
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = FRAME_ID_MAP
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def _try_connect_move_base(self):
        """Tenta la connessione a move_base se non gia' connesso.

        Usato al rilascio del controllo remoto per abilitare il docking
        autonomo nel caso l'USV sia stato spostato dall'operatore.
        Timeout breve (1s) per non bloccare il ciclo.
        """
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient(
                'move_base', MoveBaseAction
            )
        if not self.nav_system_active[0]:
            mb_connected = self.move_base_client.wait_for_server(
                rospy.Duration(1.0)
            )
            if mb_connected:
                self.nav_system_active[0] = True
                rospy.loginfo(
                    "move_base connesso al rilascio del controllo remoto."
                )

    def _send_waypoint_goal(self, waypoint, seq=None):
        """Crea e invia un MoveBaseGoal per il waypoint specificato.

        NOTA: i waypoint devono essere gia' in coordinate frame map (x, y).
        Se USE_GPS_WAYPOINTS=True, la conversione avviene in batch tramite
        _convert_all_waypoints_to_map() all'inizio della navigazione.

        Args:
            waypoint: tupla (x, y) nel frame map.
            seq: numero di sequenza (default: self.mission_index).
        """
        if seq is None:
            seq = self.mission_index
        goal = self._create_move_base_goal(waypoint[0], waypoint[1], seq)
        rospy.loginfo(
            f"Invio Goal ActionLib: X={waypoint[0]:.2f}, Y={waypoint[1]:.2f} "
            f"nel frame '{FRAME_ID_MAP}'"
        )
        self.move_base_client.send_goal(goal)

        # Log waypoint tramite logger
        if self.waypoints:
            self.logger.log_waypoint(
                wp_index=self.mission_index,
                total_wp=len(self.waypoints),
                lat=waypoint[0],
                lon=waypoint[1],
                status="SENT"
            )

    # ------------ CALCOLO DISTANZE E PERCORSI ------------

    def distanza_rimanente(self, pos_attuale):
        """Calcola la distanza rimanente alla fine della missione.

        Args:
            pos_attuale: posizione corrente come tupla (x, y).
        """
        if len(self.lunghezza_segmenti) <= 1:
            return 0.0
        if not self.waypoints or self.mission_index > len(self.waypoints):
            return 0.0

        distanza = sum(
            self.lunghezza_segmenti[i]
            for i in range(self.mission_index - 1, len(self.waypoints) - 1)
        )
        distanza += MissionExtInterface.dist(
            pos_attuale, self.waypoints[self.mission_index - 1]
        )
        return distanza

    def calc_posa(self, x, y, frame=FRAME_ID_MAP):
        """Crea un PoseStamped con le coordinate e il frame specificati."""
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def calc_lunghezza(self, path):
        """Calcola la lunghezza di un path generato da make_plan."""
        if not path or len(path.poses) < 2:
            return 0.0
        lunghezza = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            lunghezza += math.sqrt(
                (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2
            )
        return lunghezza

    def get_path_between(self, start_pose, goal_pose):
        """Richiede un path tra due punti al servizio make_plan.

        Returns:
            Il path trovato oppure None.
        """
        rospy.loginfo("In attesa del servizio make_plan...")
        try:
            make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
            plan = make_plan(start_pose, goal_pose, 0.5)
            return plan.plan
        except rospy.ServiceException as e:
            rospy.logerr(
                f"Errore durante la chiamata del servizio make_plan: {e}"
            )
            return None

    def lunghezza_missione(self, pos_attuale):
        """Calcola la lunghezza totale della missione dal planner.

        Args:
            pos_attuale: posizione corrente come tupla (x, y).
        """
        if not self.waypoints or len(self.waypoints) < 2:
            return 0.0

        lunghezza_totale = 0.0
        for i in range(self.mission_index, len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            start_point = self.calc_posa(wp1[0], wp1[1], FRAME_ID_MAP)
            end_point = self.calc_posa(wp2[0], wp2[1], FRAME_ID_MAP)

            path = self.get_path_between(start_point, end_point)
            if path:
                lunghezza = self.calc_lunghezza(path)
                self.lunghezza_segmenti.append(lunghezza)
                rospy.loginfo(f"Segmento {i + 1}: {lunghezza:.2f} m")
                lunghezza_totale += lunghezza
            else:
                rospy.logwarn(
                    f"Impossibile calcolare path tra waypoint {i} e {i + 1}"
                )

        lunghezza_totale += MissionExtInterface.dist(
            pos_attuale, self.waypoints[self.mission_index - 1]
        )
        rospy.loginfo(
            f"Lunghezza totale stimata della missione: "
            f"{lunghezza_totale:.2f} m"
        )
        return lunghezza_totale

    # ------------ GESTIONE ERRORI ------------

    def handle_error(self, error_type, error_msg):
        """Gestisce e logga un errore con il logger e rospy."""
        self.logger.log_error(
            error_type=error_type,
            error_msg=error_msg,
            context={
                'state': self.usv_status,
                'mission_index': self.mission_index,
                'timestamp': time.time()
            }
        )
        rospy.logerr(f"[{error_type}] {error_msg}")

    # ==========================================================================
    # CICLO PRINCIPALE E MACCHINA A STATI
    # ==========================================================================

    def run(self, event):
        """Ciclo principale del mission controller (chiamato dal timer a 1 Hz)."""
        self.can_jump = True
        while self.can_jump and not rospy.is_shutdown():
            loop_start_time = time.time()
            old_status = self.usv_status
            self.can_jump = False

            # Diagnostica
            self.system_status = self.usv_diagnostic.get_usv_status()
            self.check_system_status()
            self.minor_error = self.sensor_error or self.planner_error

            # Macchina a stati
            self.state_machine()

            # Telemetria
            pose = self.get_usv_pose()
            if pose:
                self.current_position = pose[:2]
                self.gnd_station_if.pub_telemetry(
                    self.usv_status, pose[:2]
                )

            # --- Logging ---
            if old_status != self.usv_status:
                self.logger.log_state_change(old_status, self.usv_status)
                rospy.loginfo(
                    f"Stato cambiato: {old_status} -> {self.usv_status}"
                )

            if self.current_position:
                self.logger.log_telemetry(
                    lat=self.current_position[0],
                    lon=self.current_position[1],
                    sog=self.gnd_station_if.mean_vel,
                    cog=self.gnd_station_if.mean_hdg,
                    state=self.usv_status
                )

            # Performance (ogni 10 cicli)
            self._perf_counter += 1
            if self._perf_counter >= 10:
                self._perf_counter = 0
                process = psutil.Process()
                cpu_percent = process.cpu_percent()
                mem_mb = process.memory_info().rss / 1024 / 1024
                loop_time_ms = (time.time() - loop_start_time) * 1000
                self.logger.log_performance(cpu_percent, mem_mb, loop_time_ms)

    def state_machine(self):
        """Dispatcher della macchina a stati."""
        self.heartbeat.publish(Header(stamp=rospy.Time.now()))

        if self.usv_status == MAPPA_DEGLI_STATI_USV[0]:
            self._handle_state_idle()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[1]:
            self._handle_state_startup()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[2]:
            self._handle_state_navigation()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[3]:
            self._handle_state_remote()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[4]:
            self._handle_state_minor_error()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[5]:
            self._handle_state_critical_error()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[6]:
            self._handle_state_docking()
        elif self.usv_status == MAPPA_DEGLI_STATI_USV[7]:
            self._handle_state_awaiting_remote()
        else:
            rospy.logwarn(f"Stato non gestito: {self.usv_status}")
            self.jump_to_state(0)

    # ------------ STATO 0: INATTIVO IN ATTESA ------------

    def _handle_state_idle(self):
        """Stato 0: cerca missioni e controlla richieste di controllo remoto."""
        rospy.loginfo("STATO: inattivo in attesa")

        # Reset contatori quando entriamo in idle
        self.startup_retry_count = 0
        self.docking_goal_sent = False
        self.docking_retry_count = 0

        # Inactivity counter (seconds since entering idle without activity)
        now = rospy.get_time() if rospy.core.is_initialized() else time.time()
        if not hasattr(self, "_idle_start_time"):
            self._idle_start_time = now

        # Reset inactivity counter on activity
        if self.is_remote_control or self.is_valid_mission:
            self._idle_start_time = now
            self.is_timed_out = False

        # Check inactivity timeout -> docking (solo se non gia' rientrato)
        if not self.docking_completed:
            if (now - self._idle_start_time) > INACTIVITY_TIMEOUT:
                self.is_timed_out = True
                rospy.logwarn(
                    "Timeout inattivita' superato. Passaggio a docking."
                )
                self.jump_to_state(6)
                self.pub_status.publish(0)
                return

        if self.is_remote_control and self.autonomous_nav_failed:
            self.jump_to_state(7)

        elif self.is_remote_control:
            self.jump_to_state(3)

        elif self.is_valid_mission:
            self.jump_to_state(1)

        else:
            self.mission_files = self.search_for_mission(
                self.mission_directory
            )
            if self.mission_files:
                last_mission = self.check_last_mission(self.mission_files)
                if last_mission:
                    if self.load_mission(last_mission):
                        self._check_mission_history()
                        # New mission found: reset inactivity e docking
                        self._idle_start_time = now
                        self.is_timed_out = False
                        if self.is_valid_mission:
                            self.docking_completed = False
                    else:
                        self.is_valid_mission = False
                        rospy.logerr(
                            "Errore nel caricamento della missione."
                        )
                else:
                    self.is_valid_mission = False
                    rospy.loginfo("Nessuna missione trovata.")
            else:
                self.is_valid_mission = False
                rospy.loginfo("Nessuna missione trovata.")

        self.pub_status.publish(0)

    def _check_mission_history(self):
        """Verifica se la missione caricata e' gia' stata eseguita o fallita."""
        existing_ids = [m.mission_id for m in self.missions]
        if self.new_mission_id in existing_ids:
            try:
                active_mission = next(
                    m for m in self.missions
                    if m.mission_id == self.new_mission_id
                )
                if active_mission.mission_status == "completed":
                    self.is_valid_mission = False
                    rospy.loginfo(
                        "Missione gia' eseguita. In attesa di nuova missione."
                    )
                elif active_mission.mission_status == "failed":
                    self.is_valid_mission = False
                    rospy.logwarn(
                        "Missione precedentemente fallita per problemi di sistema. "
                        "Inviare una nuova missione."
                    )
                elif active_mission.mission_status == "corrupted":
                    self.is_valid_mission = False
                    rospy.logwarn(
                        "Missione corrotta. Inviare una nuova missione."
                    )
                else:
                    self.is_valid_mission = True
                    rospy.loginfo(
                        "Riprendo missione precedentemente abortita."
                    )
            except StopIteration:
                rospy.loginfo(
                    f"Missione {self.new_mission_id} non ancora iniziata."
                )
                self.is_valid_mission = True
        else:
            self.is_valid_mission = True
            rospy.loginfo("Nuova missione disponibile.")
            self.mission_index = 0

    # ------------ STATO 1: AVVIO NAVIGAZIONE AUTONOMA ------------

    def _handle_state_startup(self):
        """Stato 1: verifica che planner e TF siano attivi prima della navigazione."""
        rospy.loginfo("STATO: avvio planner")

        if self.is_remote_control:
            rospy.loginfo("Passaggio in modalita' controllo remoto.")
            self.startup_retry_count = 0  # Reset contatore
            self.jump_to_state(3)
            self.pub_status.publish(1)
            return

        if all(self.nav_system_active):
            rospy.loginfo(
                "Planner attivo: passaggio in navigazione autonoma."
            )
            self.startup_retry_count = 0  # Reset contatore

            # Converti waypoint GPS -> map se necessario
            if not self._convert_all_waypoints_to_map():
                rospy.logerr(
                    "Conversione waypoint GPS->map fallita. "
                    "Missione marcata come failed."
                )
                self.save_mission_info("failed")
                self.is_valid_mission = False
                self.planner_error = True
                self.jump_to_state(4)
                self.pub_status.publish(1)
                return

            self.save_mission_info("iniziata")
            self.jump_to_state(2)
            self.pub_status.publish(1)
            return

        # Verifica se abbiamo superato il numero massimo di tentativi globali
        if self.startup_retry_count >= MAX_MBS_RETRY_CNT:
            rospy.logerr(
                f"Impossibile connettersi ai server di navigazione dopo "
                f"{MAX_MBS_RETRY_CNT} tentativi. Missione marcata come fallita."
            )
            self.logger.log_error(
                "STARTUP_FAILED",
                f"Connessione a move_base/TF fallita dopo {MAX_MBS_RETRY_CNT} tentativi"
            )
            # Marca la missione come fallita
            self.save_mission_info("failed")
            self.startup_retry_count = 0
            self.is_valid_mission = False
            self.planner_error = False
            self.is_remote_control = True
            self.autonomous_nav_failed = True
            self.jump_to_state(7)  # Attesa comando remoto
            self.pub_status.publish(1)
            return

        if not self.nav_system_active[0] and self.minor_error:
            if self.is_valid_mission:
                self.jump_to_state(4)
                rospy.logerr(
                    "Planner non funzionante, passare in controllo remoto."
                )
            else:
                self.jump_to_state(0)
                rospy.logwarn("Condizione anomala. Ritorno in idle.")
            self.pub_status.publish(1)
            return

        if not self.nav_system_active[1] and self.minor_error:
            if self.is_valid_mission:
                self.jump_to_state(4)
                rospy.logerr(
                    "Sistema TF non funzionante, passare in controllo remoto."
                )
            else:
                self.jump_to_state(0)
                rospy.logwarn("Condizione anomala. Ritorno in idle.")
            self.pub_status.publish(1)
            return

        # Connessione a move_base
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient(
                'move_base', MoveBaseAction
            )

        if not self.nav_system_active[0]:
            rospy.loginfo(
                f"Tentativo {self.retry_cnt[0]} di connessione a move_base..."
            )
            mb_connected = self.move_base_client.wait_for_server(
                rospy.Duration(MB_SERVER_TIMEOUT)
            )
            if mb_connected:
                self.nav_system_active[0] = True
                self.retry_cnt[0] = 1
                rospy.loginfo("Server move_base connesso.")
            else:
                self.nav_system_active[0] = False
                if self.retry_cnt[0] > MAX_MBS_RETRY_CNT:
                    self.planner_error = True
                    if "errore planner" not in self.errori_minori:
                        self.errori_minori.append("errore planner")
                self.retry_cnt[0] += 1
                rospy.logwarn("Timeout: move_base non risponde.")

        # Attivazione fittizia TF per test
        self.nav_system_active[1] = True

        # Subscriber per lo stato di move_base (evita duplicazione)
        if self.sub_nav_status is None:
            self.sub_nav_status = rospy.Subscriber(
                '/move_base/status', GoalStatusArray,
                self.nav_result_callback
            )
        elif self.sub_nav_status.get_num_connections() == 0:
            self.sub_nav_status = rospy.Subscriber(
                '/move_base/status', GoalStatusArray,
                self.nav_result_callback
            )

        # Incrementa contatore tentativi globale se i sistemi non sono ancora attivi
        if not all(self.nav_system_active):
            self.startup_retry_count += 1
            rospy.logwarn(
                f"Sistemi navigazione non pronti. Tentativo "
                f"{self.startup_retry_count}/{MAX_MBS_RETRY_CNT}"
            )

        self.pub_status.publish(1)

    # ------------ STATO 2: NAVIGAZIONE AUTONOMA ------------

    def _check_tf_available(self):
        """Verifica non bloccante della TF map->base_link.

        Returns:
            True se la trasformazione e' disponibile, False altrimenti.
        """
        try:
            self.tf_buffer.lookup_transform(
                FRAME_ID_MAP, FRAME_BASE_ID, rospy.Time(0)
            )
            return True
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False

    def _handle_state_navigation(self):
        """Stato 2: gestisce la navigazione waypoint-by-waypoint."""
        rospy.loginfo("STATO: in navigazione autonoma")

        if not self._check_tf_available():
            rospy.logerr(
                "TF map->base_link non disponibile durante la navigazione."
            )
            self.planner_error = True
            self.jump_to_state(4)
            self.pub_status.publish(2)
            return

        if self.is_remote_control:
            self.pub_abort_current_goal.publish(GoalID())
            self.save_mission_info("suspended")
            self.jump_to_state(3)

        elif not self.is_valid_mission:
            self.jump_to_state(0)

        elif self.minor_error:
            self.jump_to_state(4)

        elif self.major_error:
            self.jump_to_state(5)

        else:
            self._execute_navigation()

        self.pub_status.publish(2)

    def _execute_navigation(self):
        """Logica interna di navigazione: invio waypoint e gestione status."""
        # Primo waypoint
        if self.mission_index == 0:
            rospy.loginfo("Inizio della missione...")
            self._send_waypoint_goal(self.waypoints[self.mission_index])
            self.mission_index += 1

        elif self.mission_index <= len(self.waypoints):
            self._handle_navigation_status()

    def _handle_navigation_status(self):
        """Gestisce le risposte di move_base durante la navigazione."""
        # SUCCEEDED: waypoint raggiunto
        if self.navigation_status == MAPPA_MOVE_BASE_STATUS[3]:
            self.consecutive_aborts = 0  # Reset contatore abort consecutivi
            self.waypoints_reached += 1
            rospy.loginfo(f"Prossimo waypoint {self.mission_index}")
            if self.mission_index < len(self.waypoints):
                rospy.loginfo("Procedo al waypoint successivo.")
                self._send_waypoint_goal(
                    self.waypoints[self.mission_index]
                )
                self.mission_index += 1
                self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]
            else:
                rospy.loginfo(
                    "Ultimo waypoint raggiunto. Missione completata."
                )
                self.is_valid_mission = False
                self.save_mission_info("completed")
                self.mission_index = 0

        # ABORTED: planner non riesce a raggiungere il waypoint
        # Strategia: salta al waypoint successivo senza passare per Stato 4.
        # Un singolo waypoint irraggiungibile non e' un errore di sistema.
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[4]:
            self.consecutive_aborts += 1
            rospy.logwarn(
                f"Waypoint {self.mission_index} irraggiungibile "
                f"({self.consecutive_aborts} abort consecutivi). "
                f"Planner: {self.navigation_status}"
            )
            self.logger.log_error(
                "WP_ABORTED",
                f"Waypoint {self.mission_index} irraggiungibile, "
                f"abort consecutivi: {self.consecutive_aborts}"
            )

            if self.mission_index < len(self.waypoints):
                rospy.loginfo(
                    f"Salto al waypoint successivo ({self.mission_index + 1})"
                )
                self._send_waypoint_goal(
                    self.waypoints[self.mission_index]
                )
                self.mission_index += 1
                # Reset status per evitare re-trigger al prossimo tick
                self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]
            else:
                rospy.logwarn(
                    "Ultimo waypoint irraggiungibile. Missione terminata."
                )
                self.is_valid_mission = False
                self.save_mission_info("completed")
                self.mission_index = 0                

        # ACTIVE: planner sta lavorando
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[1]:
            rospy.loginfo(
                f"Prossimo waypoint {self.mission_index}. "
                f"Planner: {self.navigation_status}."
            )

        # PREEMPTED: goal interrotto, trova il waypoint piu' vicino
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[2]:
            self._handle_preempted_goal()

        # PENDING: in attesa che il goal venga elaborato
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[0]:
            self._handle_pending_goal()

        # REJECTED: goal rifiutato dal planner
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[5]:
            rospy.logerr(
                "Il planner ha rifiutato il goal. "
                "Passaggio in errore minore."
            )
            self.planner_error = True
            if "errore planner" not in self.errori_minori:
                self.errori_minori.append("errore planner")

        # Missione completata o nessun waypoint
        elif (not self.waypoints
              or self.mission_index >= len(self.waypoints)):
            rospy.loginfo("Missione completata.")
            self.is_valid_mission = False

    def _handle_preempted_goal(self):
        """Gestisce un goal preempted: trova e invia il waypoint piu' vicino."""
        rospy.loginfo("Goal preempted.")
        pose = self.get_usv_pose()
        if not pose:
            return

        distanze = [
            math.sqrt((pose[0] - wp[0]) ** 2 + (pose[1] - wp[1]) ** 2)
            for wp in self.waypoints
        ]
        wp_piu_vicino = distanze.index(min(distanze))

        self._send_waypoint_goal(self.waypoints[wp_piu_vicino])
        self.mission_index = wp_piu_vicino + 1

    def _handle_pending_goal(self):
        """Gestisce un goal in PENDING: attende o re-invia dopo timeout."""
        rospy.loginfo(
            "Goal pending: in attesa che il goal venga elaborato..."
        )
        if self.pending_time is None:
            self.pending_time = rospy.get_time()

        elapsed = rospy.get_time() - self.pending_time
        if elapsed > PENDING_TIMEOUT:
            rospy.logerr(
                "Goal bloccato: cancellazione e re-invio."
            )
            self.pending_time = None
            self.move_base_client.cancel_goal()
            rospy.sleep(0.5)
            rospy.loginfo(
                f"Re-invio del goal {self.mission_index - 1}..."
            )
            waypoint = self.waypoints[self.mission_index - 1]
            self._send_waypoint_goal(waypoint, seq=self.mission_index)

    # ------------ STATO 3: CONTROLLO REMOTO ------------

    def _handle_state_remote(self):
        """Stato 3: gestisce il timeout e le transizioni del controllo remoto.

        Il relay dei comandi CC avviene in cmd_vel_cc_callback() ad alta frequenza.
        Questo handler (1 Hz) gestisce solo:
        - uscita dallo stato se is_remote_control viene rimosso
        - timeout CMD_VEL_TIMEOUT se nessun comando ricevuto
        """
        rospy.loginfo("STATO: controllo remoto")

        if not self.is_remote_control:
            self._try_connect_move_base()
            self.last_cmd_vel_time = None
            self.jump_to_state(0)
            self.pub_status.publish(3)
            return

        if self.last_cmd_vel_time is None:
            self.last_cmd_vel_time = rospy.get_time()
        else:
            elapsed = rospy.get_time() - self.last_cmd_vel_time
            if elapsed > CMD_VEL_TIMEOUT:
                self._try_connect_move_base()
                rospy.loginfo("Timeout comandi remoti. Ritorno in idle.")
                self.last_cmd_vel_time = None
                self.is_remote_control = False
                self.pub_vel_cmd.publish(Twist())

        self.pub_status.publish(3)

    # ------------ STATO 4: ERRORE MINORE ------------

    def _handle_state_minor_error(self):
        """Stato 4: identifica l'errore e decide la modalita' di recupero.

        Casi gestiti:
        - Errore planner: passaggio a stato 7 (attesa comando remoto)
        - Sensore fuori uso: passaggio a stato 7 (attesa comando remoto)
        - Motore fuori uso: escalation a errore critico (stato 5)
        - Falso allarme: riprende la missione (stato 0)

        Nota: gli ABORTED di singoli waypoint sono gestiti direttamente
        in _handle_navigation_status() senza passare per questo stato.
        """
        rospy.loginfo("STATO: errore minore. Identificazione dell'errore.")

        if not self.minor_error:
            self.jump_to_state(0)

        elif self.major_error:
            self.jump_to_state(5)

        elif self.is_remote_control and self.autonomous_nav_failed:
            self.jump_to_state(7)

        elif self.is_remote_control:
            self.jump_to_state(3)

        else:
            # Fermata di emergenza
            self.mc_cmd_vel = Twist()
            self.pub_vel_cmd.publish(self.mc_cmd_vel)
            self.pub_mc_cmd_vel.publish(self.mc_cmd_vel)

            if "errore planner" in self.errori_minori:
                rospy.logerr(
                    "Errore del planner: passaggio in attesa comando remoto."
                )
                self.planner_error = False
                self.is_valid_mission = False
                self.errori_minori.remove("errore planner")
                self.is_remote_control = True
                self.autonomous_nav_failed = True
                self.jump_to_state(7)

            elif "sensore fuori uso" in self.errori_minori:
                rospy.logerr(
                    "Errore sensore Ellipse: passaggio in attesa comando remoto."
                )
                self.sensor_error = False
                self.errori_minori.remove("sensore fuori uso")
                self.is_remote_control = True
                self.autonomous_nav_failed = True
                self.jump_to_state(7)

            elif "motore fuori uso" in self.errori_minori:
                self.major_error = True

            else:
                rospy.loginfo(
                    "Falso allarme: nessun malfunzionamento rilevato. "
                    "La missione puo' riprendere."
                )
                self.sensor_error = False
                self.planner_error = False

        self.pub_status.publish(4)

    # ------------ STATO 5: ERRORE CRITICO ------------

    def _handle_state_critical_error(self):
        """Stato 5: errore critico, USV non recuperabile autonomamente.

        Pubblica inibizione motori ad ogni tick finche' si resta in questo stato.
        """
        rospy.logerr(
            "Errore critico! USV non risponde. Intervento richiesto."
        )

        self.mc_cmd_vel = Twist()
        self.pub_vel_cmd.publish(self.mc_cmd_vel)
        self.pub_mc_cmd_vel.publish(self.mc_cmd_vel)

        if not self.major_error and self.minor_error:
            self.jump_to_state(4)
        elif not self.major_error:
            self.jump_to_state(0)

        self.pub_status.publish(5)

    # ------------ STATO 6: DOCKING ------------

    def _handle_state_docking(self):
        """Stato 6: naviga verso il punto di docking.

        Invia il goal una sola volta e gestisce la risposta del planner:
        - SUCCEEDED: docking completato, torna in idle
        - ABORTED: riprova fino a MAX_DOCKING_RETRIES tentativi
        - REJECTED: riprova una volta, poi prova coordinate con offset +-10%
        - ACTIVE/PENDING: attende
        """
        rospy.loginfo("STATO: docking in corso")

        if self.is_remote_control:
            self.pub_abort_current_goal.publish(GoalID())
            self.docking_goal_sent = False
            self.jump_to_state(3)
            self.pub_status.publish(6)
            return

        if self.move_base_client is None:
            rospy.logerr(
                "move_base_client non inizializzato. "
                "Impossibile eseguire docking."
            )
            self.docking_goal_sent = False
            self.docking_completed = True  # Evita loop idle-docking
            self.jump_to_state(0)
            self.pub_status.publish(6)
            return

        if not self._check_tf_available():
            rospy.logerr(
                "TF map->base_link non disponibile durante il docking. "
                "Passaggio in attesa comando remoto."
            )
            self.docking_goal_sent = False
            self.is_remote_control = True
            self.autonomous_nav_failed = True
            self.jump_to_state(7)
            self.pub_status.publish(6)
            return

        # Invio goal solo la prima volta
        if not self.docking_goal_sent:
            rospy.loginfo(
                f"Invio goal docking: "
                f"X={self.docking_target[0]}, Y={self.docking_target[1]}"
            )
            self._send_waypoint_goal(self.docking_target, seq=0)
            self.docking_goal_sent = True
            self.docking_retry_count = 0
            self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]
        else:
            self._handle_docking_status()

        self.pub_status.publish(6)

    def _handle_docking_status(self):
        """Gestisce le risposte di move_base durante il docking."""
        # SUCCEEDED: docking completato
        if self.navigation_status == MAPPA_MOVE_BASE_STATUS[3]:
            rospy.loginfo("Docking completato con successo.")
            self.docking_goal_sent = False
            self.docking_completed = True
            self.jump_to_state(0)

        # ABORTED: riprova fino a MAX_DOCKING_RETRIES
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[4]:
            self.docking_retry_count += 1
            if self.docking_retry_count <= MAX_DOCKING_RETRIES:
                rospy.logwarn(
                    f"Docking ABORTED. Tentativo "
                    f"{self.docking_retry_count}/{MAX_DOCKING_RETRIES}"
                )
                self._send_waypoint_goal(self.docking_target, seq=0)
                self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]
            else:
                rospy.logerr(
                    f"Docking fallito dopo {MAX_DOCKING_RETRIES} tentativi. "
                    f"Ritorno in idle."
                )
                self.docking_goal_sent = False
                self.docking_completed = True  # Evita loop idle-docking
                self.jump_to_state(0)

        # REJECTED: riprova una volta, poi prova con offset +-10%
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[5]:
            self.docking_retry_count += 1
            if self.docking_retry_count == 1:
                rospy.logwarn(
                    "Goal docking rifiutato. Riprovo stesse coordinate."
                )
                self._send_waypoint_goal(self.docking_target, seq=0)
                self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]
            elif self.docking_retry_count == 2:
                offset_target = (
                    self.docking_target[0]
                    * (1.0 + random.uniform(-0.1, 0.1)),
                    self.docking_target[1]
                    * (1.0 + random.uniform(-0.1, 0.1))
                )
                rospy.logwarn(
                    f"Goal docking rifiutato di nuovo. "
                    f"Provo coordinate con offset: "
                    f"X={offset_target[0]:.2f}, Y={offset_target[1]:.2f}"
                )
                self._send_waypoint_goal(offset_target, seq=0)
                self.navigation_status = MAPPA_MOVE_BASE_STATUS[-1]
            else:
                rospy.logerr(
                    "Goal docking rifiutato ripetutamente. "
                    "Ritorno in idle."
                )
                self.docking_goal_sent = False
                self.docking_completed = True  # Evita loop idle-docking
                self.jump_to_state(0)

        # ACTIVE: planner sta navigando verso il docking
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[1]:
            rospy.loginfo("Docking: navigazione in corso...")

        # PENDING: in attesa di elaborazione
        elif self.navigation_status == MAPPA_MOVE_BASE_STATUS[0]:
            rospy.loginfo("Docking: goal in attesa di elaborazione...")

    # ------------ STATO 7: IN ATTESA COMANDO REMOTO ------------

    def _handle_state_awaiting_remote(self):
        """Stato 7: navigazione autonoma non disponibile, in attesa di comandi remoti.

        Si entra in questo stato quando un errore (planner, TF, docking fallito)
        impedisce la navigazione autonoma. is_remote_control viene settato a True
        prima di entrare: l'operatore esce inviando remote_cmd=False.

        Non ha timeout: l'operatore decide quando e come procedere.
        Non pubblica Twist zero per non interferire con eventuali comandi remoti
        in arrivo dal callback cmd_vel_cc_callback.

        Uscite:
        - Nuova missione valida -> Stato 1 (riprova navigazione autonoma)
        - Operatore rimuove is_remote_control -> Stato 0 (idle, timer resettato)
        """
        rospy.loginfo(
            "STATO: in attesa comando remoto (navigazione autonoma non disponibile)"
        )

        # Controlla se e' arrivata una nuova missione
        self.mission_files = self.search_for_mission(self.mission_directory)
        if self.mission_files:
            last_mission = self.check_last_mission(self.mission_files)
            if last_mission:
                if self.load_mission(last_mission):
                    self._check_mission_history()
                    if self.is_valid_mission:
                        rospy.loginfo(
                            "Nuova missione ricevuta. "
                            "Tentativo di navigazione autonoma."
                        )
                        self.is_remote_control = False
                        self.autonomous_nav_failed = False
                        self.docking_completed = False
                        self.jump_to_state(1)
                        self.pub_status.publish(7)
                        return

        # Operatore rimuove is_remote_control -> torna in idle
        if not self.is_remote_control:
            self.autonomous_nav_failed = False
            # Reset timer inattivita' per evitare docking immediato
            self._idle_start_time = (
                rospy.get_time() if rospy.core.is_initialized()
                else time.time()
            )
            self.jump_to_state(0)
            self.pub_status.publish(7)
            return

        self.pub_status.publish(7)


# =============================================================================
# FUNZIONI MODULE-LEVEL
# =============================================================================

def is_roscore_running_master_check():
    """Controlla se roscore e' attivo tentando di connettersi al Master ROS."""
    try:
        master = Master('/rospy_master_check')
        master.getUri()
        return True
    except MasterException:
        return False
    except Exception:
        return False


def get_ascii_art():
    """Restituisce l'ASCII art dell'USV."""
    return r"""
               .--.              .--.
             .'_\/_'._        _.'_\/_'._
            '. /\ /\ .'      '. /\ /\ .'
              "|| ||"          "|| ||"
               || ||            || ||
          /\   || ||  /\    /\  || ||   /\
         /  \  || || /  \  /  \ || ||  /  \
        /====\ ||_||/====\/====\||_|| /====\
       /  []  \====/  []  \  []  \====/  [] \
      /________\__/________\_____/__\________\
       /  /  \  \  /  /  \  \  /  /  \  \  /  \
      /__/____\__\/__/____\__\/__/____\__\/__\_\
         \_/  \_/    \_/  \_/    \_/  \_/  \_/
             \_/        \_/        \_/
              / \  .-----.  / \
             / _ \/  .-.  \/ _ \
            | (_) | (   ) | (_) |
             \___/   `-'   \___/
               |             |
               |  \     /    |
                \  `---'    /
                 `.______.'
                    |  |
                 ___|  |___
                /          \
               /            \
              /              \
             /                \
    """


def get_ascii_art_sitep():
    """Restituisce l'ASCII art del logo Sitepi."""
    return r"""
__          __    _____  _____  _______  ______  _____
\ \        / /   / ____||_   _||__   __||  ____||  __ \
 \ \  /\  / /   | (___    | |     | |   | |__   | |__) |
  \ \/  \/ /     \___ \   | |     | |   |  __|  |  __ /
   \  /\  /      ____) | _| |_    | |   | |____ | |
    \/  \/      |_____/ |_____|   |_|   |______||_|
"""


# =============================================================================
# MAIN
# =============================================================================

if __name__ == '__main__':
    wait_counter = 0
    while (not is_roscore_running_master_check()
           and wait_counter < ROSTIMEOUT):
        print("In attesa del ROS core...")
        wait_counter += 1
        time.sleep(1)

    if wait_counter < ROSTIMEOUT:
        mission_controller = MissionController()
        mission_controller.start_control_node()
    else:
        print("Timeout scaduto: il ROS core non risponde.")
        print("Programma terminato, exit code: 0")
        print()
        print(get_ascii_art_sitep())
