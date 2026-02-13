#!/usr/bin/env python3

# USV Diagnostic Node
# Controlla lo stato dei sensori, dell'odometria e dei motori al fine di capire
# se ci sono errori di sistema.
# Autore: Davide Domeneghetti
# Email: d.domeneghetti@sitepitalia.it
# Versione: 1.1
# Data: 16/09/2025
# Note: Questo script richiede il pkg 'sbg_driver'.
#       Installalo tramite sudo apt-get install ros-noetic-sbg-driver.

import math
import time
from dataclasses import dataclass

import rospy
from custom_msgs.msg import SystemStatus
from geometry_msgs.msg import Twist
from sbg_driver.msg import (
    SbgEkfEuler,
    SbgEkfNav,
    SbgGpsHdt,
    SbgGpsPos,
    SbgGpsVel,
    SbgImuData,
    SbgMag,
    SbgStatusGeneral,
)

# --- Costanti ---
MAPPA_STATI_ERROR = {
    0: 'NO_ERRORE',
    1: 'SENSORE_DEGRADATO',
    3: 'SENSORE_ASSENTE',
    4: 'EMERGENCY_STOP'
}

# Costanti rilevamento guasto motore (default, sovrascritte da parametri ROS)
MOTOR_CHECK_INTERVAL = 10.0   # Secondi tra un rilevamento e l'altro
MOTOR_FAIL_THRESHOLD = 3      # Rilevamenti consecutivi per dichiarare guasto
MIN_DISPLACEMENT = 2.0        # Metri: sotto questa soglia il veicolo e' "fermo"
MIN_HEADING_CHANGE = 3.0      # Gradi: sotto questa soglia heading e' "costante"
DRIFT_ANGLE_THRESHOLD = 90.0  # Gradi: discrepanza direzione per rilevare deriva
EARTH_RADIUS = 6371000.0      # Raggio terrestre in metri (costante fisica)


@dataclass
class SystemStatusStruct:
    """Struttura dati per lo stato dei sensori del sistema."""

    sensor_presence: bool = True
    imu_presence: bool = True
    gps_presence: bool = True
    is_gpspos: bool = True
    gps_type: int = 0
    is_gpsvel: bool = True
    gps_vel_type: int = 0
    is_gps_hdt: bool = True
    is_magnetometer: bool = True
    is_imu: bool = True
    ekf_status: int = 0
    is_motor_ok: bool = True


class UsvDiagnostic:
    """Classe dedicata al monitoraggio dello stato dei sensori dell'USV.

    Sottoscrive i topic del sensore SBG Ellipse e valuta lo stato
    di GPS, IMU, EKF e magnetometro.
    """

    def __init__(self):
        # Timeout e soglie (gia' parametrizzati)
        self.imu_timeout = rospy.get_param("~imu_timeout", 1.0)
        self.gps_timeout = rospy.get_param("~gps_timeout", 2.0)
        self.max_accel = rospy.get_param("~max_accel", 50.0)
        self.min_satellites = rospy.get_param("~min_satellites", 6)

        # Parametri rilevamento guasto motore (dal YAML)
        # Usa namespace assoluto /usv_diagnostic/ perche' questa classe viene
        # istanziata dentro mission_controller (~ risolverebbe al namespace sbagliato)
        global MOTOR_CHECK_INTERVAL, MOTOR_FAIL_THRESHOLD
        global MIN_DISPLACEMENT, MIN_HEADING_CHANGE, DRIFT_ANGLE_THRESHOLD
        MOTOR_CHECK_INTERVAL = rospy.get_param(
            "/usv_diagnostic/motor_check_interval", MOTOR_CHECK_INTERVAL
        )
        MOTOR_FAIL_THRESHOLD = rospy.get_param(
            "/usv_diagnostic/motor_fail_threshold", MOTOR_FAIL_THRESHOLD
        )
        MIN_DISPLACEMENT = rospy.get_param(
            "/usv_diagnostic/min_displacement", MIN_DISPLACEMENT
        )
        MIN_HEADING_CHANGE = rospy.get_param(
            "/usv_diagnostic/min_heading_change", MIN_HEADING_CHANGE
        )
        DRIFT_ANGLE_THRESHOLD = rospy.get_param(
            "/usv_diagnostic/drift_angle_threshold", DRIFT_ANGLE_THRESHOLD
        )

        # Stato interno dei sottosistemi
        self._diagnostic_stato_gps = None
        self._diagnostic_stato_imu = None
        self._diagnostic_stato_ekf = None
        self._diagnostic_stato_gpsvel = None
        self._diagnostic_stato_gpshdt = None
        self._diagnostic_stato_mag = None
        self._diagnostic_stato_shipm = None
        self._diagnostic_stato_generale = None
        self._diagnostic_stato_motore = None
        self.status = SystemStatusStruct()

        # Stato rilevamento motore
        self._motor_check_time = None
        self._prev_gps_pos = None           # (lat, lon) al check precedente
        self._prev_heading = None           # heading (gradi) al check precedente
        self._current_gps_pos = None        # (lat, lon) aggiornato dal callback GPS
        self._current_heading = None        # heading (gradi) aggiornato dal callback EKF
        self._cmd_vel_active_linear = False  # Comandi lineari ricevuti nel periodo
        self._cmd_vel_active_angular = False  # Comandi angolari ricevuti nel periodo
        self._translation_fail_count = 0
        self._rotation_fail_count = 0
        self._drift_fail_count = 0

        # Subscribers
        self.sub_imu = rospy.Subscriber(
            "/sbg/imu_data", SbgImuData, self.imu_callback
        )
        self.sub_gps_pos = rospy.Subscriber(
            "/sbg/gps_pos", SbgGpsPos, self.gps_callback
        )
        self.sub_ekf_euler = rospy.Subscriber(
            "/sbg/ekf_euler", SbgEkfEuler, self.ekf_callback
        )
        self.sub_gps_vel = rospy.Subscriber(
            "/sbg/gps_vel", SbgGpsVel, self.gps_vel_callback
        )
        self.sub_mag = rospy.Subscriber(
            "/sbg/mag", SbgMag, self.mag_callback
        )
        self.sub_gen_status = rospy.Subscriber(
            "/sbg/status_general", SbgStatusGeneral,
            self.general_status_callback
        )
        self.sub_cmd_vel = rospy.Subscriber(
            "/cmd_vel", Twist, self.cmd_vel_callback
        )

        # Publisher
        self.status_pub = rospy.Publisher(
            "/usv_status_monitor/diagnostic/status",
            SystemStatus, queue_size=10
        )

    # ------------ CALLBACK DEI SENSORI ------------

    def cmd_vel_callback(self, msg):
        """Registra comandi di velocita' per il rilevamento guasto motore.

        Setta i flag attivi se il comando contiene velocita' non nulla.
        I flag vengono resettati ad ogni ciclo di check_motor_status().
        """
        if msg.linear.x != 0.0 or msg.linear.y != 0.0:
            self._cmd_vel_active_linear = True
        if msg.angular.z != 0.0:
            self._cmd_vel_active_angular = True

    def general_status_callback(self, msg):
        """Valuta lo stato generale del sensore Ellipse (alimentazione)."""
        main_power = not msg.main_power
        imu_power = not msg.imu_power
        gps_power = not msg.gps_power

        if main_power:
            self._diagnostic_stato_generale = 0  # Sistema spento
        elif imu_power and gps_power:
            self._diagnostic_stato_generale = 3  # Entrambi i sensori spenti
        elif imu_power:
            self._diagnostic_stato_generale = 1  # IMU spenta
        elif gps_power:
            self._diagnostic_stato_generale = 2  # GPS spento
        else:
            self._diagnostic_stato_generale = 4  # Sensori OK

    def imu_callback(self, msg):
        """Valuta lo stato della IMU."""
        if not msg.status.imu_com:
            self._diagnostic_stato_imu = 0  # Nessuna comunicazione
        elif not msg.status.imu_status:
            self._diagnostic_stato_imu = 1  # Non calibrata (scarso)
        elif (not msg.status.imu_accels_in_range
              and not msg.status.imu_gyros_in_range):
            self._diagnostic_stato_imu = 4  # Entrambe fuori range
        elif not msg.status.imu_accels_in_range:
            self._diagnostic_stato_imu = 2  # Accelerazione lineare fuori range
        elif not msg.status.imu_gyros_in_range:
            self._diagnostic_stato_imu = 3  # Accelerazione angolare fuori range
        else:
            self._diagnostic_stato_imu = 5  # IMU OK

    def gps_callback(self, msg):
        """Valuta lo stato del GPS e aggiorna la posizione per il check motore."""
        # Salva posizione GPS per rilevamento motore
        self._current_gps_pos = (msg.latitude, msg.longitude)

        if msg.status.status != 0:
            self._diagnostic_stato_gps = 0  # Errore interno
            return

        if msg.status.type >= 6:
            self._diagnostic_stato_gps = 1  # Ottimo
        elif msg.status.type in [3, 5]:
            self._diagnostic_stato_gps = 2  # Buono
        elif msg.status.type in [2, 4]:
            self._diagnostic_stato_gps = 3  # Scarso
        else:
            self._diagnostic_stato_gps = 4  # Assente

    def ekf_callback(self, msg):
        """Valuta lo stato del filtro EKF e aggiorna heading per il check motore."""
        # Salva heading (gradi) per rilevamento motore
        self._current_heading = math.degrees(msg.angle.z)

        fault_count = sum([
            not msg.status.attitude_valid,
            not msg.status.heading_valid,
            not msg.status.velocity_valid,
            not msg.status.position_valid,
        ])

        # 0 = OK, 1 = sufficiente, 2 = scarso, 3 = assente
        self._diagnostic_stato_ekf = min(fault_count, 3)

    def gps_vel_callback(self, msg):
        """Valuta lo stato della velocita' GPS."""
        if msg.status.vel_status != 0:
            self._diagnostic_stato_gpsvel = 0  # Errore interno
            return

        vel_type_map = {
            0: 1,  # Nessuna velocita' valida (assente)
            1: 2,  # Soluzione sconosciuta (molto scarsa)
            2: 3,  # Velocita' Doppler (buona)
            4: 4,  # Velocita' differenziale (ottima)
        }
        self._diagnostic_stato_gpsvel = vel_type_map.get(
            msg.status.vel_type, 1
        )

    def gps_hdt_callback(self, msg):
        """Valuta lo stato del GPS heading (HDT)."""
        if msg.status == 0:
            self._diagnostic_stato_gpshdt = 0  # Soluzione calcolata
        else:
            self._diagnostic_stato_gpshdt = 1  # Errore interno

    def mag_callback(self, msg):
        """Valuta lo stato del magnetometro."""
        if not msg.status.mags_in_range:
            self._diagnostic_stato_mag = 0  # Fuori range
        elif not msg.status.accels_in_range:
            self._diagnostic_stato_mag = 1  # Accelerazione fuori range
        elif not msg.status.calibration:
            self._diagnostic_stato_mag = 2  # Non calibrato
        else:
            self._diagnostic_stato_mag = 3  # OK

    # ------------ RILEVAMENTO GUASTO MOTORE ------------

    @staticmethod
    def _haversine_distance(pos1, pos2):
        """Calcola la distanza in metri tra due coordinate GPS (lat, lon).

        Args:
            pos1: tupla (lat, lon) in gradi.
            pos2: tupla (lat, lon) in gradi.
        Returns:
            Distanza in metri.
        """
        lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
        lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (math.sin(dlat / 2) ** 2
             + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
        return EARTH_RADIUS * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing_between(pos1, pos2):
        """Calcola il bearing (gradi, 0-360) tra due coordinate GPS.

        Args:
            pos1: tupla (lat, lon) in gradi — punto di partenza.
            pos2: tupla (lat, lon) in gradi — punto di arrivo.
        Returns:
            Bearing in gradi (0 = Nord, 90 = Est).
        """
        lat1, lon1 = math.radians(pos1[0]), math.radians(pos1[1])
        lat2, lon2 = math.radians(pos2[0]), math.radians(pos2[1])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2)
             - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        bearing = math.degrees(math.atan2(x, y))
        return bearing % 360

    @staticmethod
    def _angle_difference(a, b):
        """Calcola la differenza angolare minima tra due angoli in gradi.

        Returns:
            Differenza in gradi (0-180).
        """
        diff = abs(a - b) % 360
        return min(diff, 360 - diff)

    def check_motor_status(self):
        """Verifica il funzionamento dei motori confrontando comandi e movimento.

        Eseguito ogni MOTOR_CHECK_INTERVAL secondi. Rileva tre condizioni:
        1. Traslazione bloccata: comandi lineari ma veicolo fermo
        2. Rotazione bloccata: comandi angolari ma heading costante
        3. Deriva: comandi lineari ma movimento incoerente con la direzione

        Dopo MOTOR_FAIL_THRESHOLD rilevamenti consecutivi, dichiara guasto motore.
        """
        now = time.time()

        # Inizializzazione al primo ciclo
        if self._motor_check_time is None:
            self._motor_check_time = now
            self._prev_gps_pos = self._current_gps_pos
            self._prev_heading = self._current_heading
            return

        # Controlla solo ogni MOTOR_CHECK_INTERVAL secondi
        if (now - self._motor_check_time) < MOTOR_CHECK_INTERVAL:
            return

        self._motor_check_time = now

        # GPS non disponibile: non posso valutare, skip
        if self._current_gps_pos is None or self._prev_gps_pos is None:
            self._prev_gps_pos = self._current_gps_pos
            self._prev_heading = self._current_heading
            self._cmd_vel_active_linear = False
            self._cmd_vel_active_angular = False
            return

        displacement = self._haversine_distance(
            self._prev_gps_pos, self._current_gps_pos
        )

        # --- Check 1: Traslazione bloccata ---
        if self._cmd_vel_active_linear:
            if displacement < MIN_DISPLACEMENT:
                self._translation_fail_count += 1
                rospy.logwarn(
                    f"Check motore: comando lineare attivo ma veicolo fermo "
                    f"({self._translation_fail_count}/{MOTOR_FAIL_THRESHOLD})"
                )
            else:
                self._translation_fail_count = 0
        else:
            self._translation_fail_count = 0

        # --- Check 2: Rotazione bloccata ---
        if (self._cmd_vel_active_angular
                and self._current_heading is not None
                and self._prev_heading is not None):
            heading_change = self._angle_difference(
                self._current_heading, self._prev_heading
            )
            if heading_change < MIN_HEADING_CHANGE:
                self._rotation_fail_count += 1
                rospy.logwarn(
                    f"Check motore: comando angolare attivo ma heading costante "
                    f"({self._rotation_fail_count}/{MOTOR_FAIL_THRESHOLD})"
                )
            else:
                self._rotation_fail_count = 0
        else:
            self._rotation_fail_count = 0

        # --- Check 3: Deriva ---
        if self._cmd_vel_active_linear and displacement >= MIN_DISPLACEMENT:
            # Il veicolo si muove: verifico se nella direzione giusta
            if self._current_heading is not None:
                actual_bearing = self._bearing_between(
                    self._prev_gps_pos, self._current_gps_pos
                )
                # Confronto heading attuale con bearing effettivo
                discrepancy = self._angle_difference(
                    self._current_heading, actual_bearing
                )
                if discrepancy > DRIFT_ANGLE_THRESHOLD:
                    self._drift_fail_count += 1
                    rospy.logwarn(
                        f"Check motore: possibile deriva, discrepanza "
                        f"{discrepancy:.0f} gradi "
                        f"({self._drift_fail_count}/{MOTOR_FAIL_THRESHOLD})"
                    )
                else:
                    self._drift_fail_count = 0
            else:
                self._drift_fail_count = 0
        elif not self._cmd_vel_active_linear:
            # Nessun comando lineare: non e' deriva, e' corrente
            self._drift_fail_count = 0

        # --- Valutazione finale ---
        if (self._translation_fail_count >= MOTOR_FAIL_THRESHOLD
                or self._rotation_fail_count >= MOTOR_FAIL_THRESHOLD
                or self._drift_fail_count >= MOTOR_FAIL_THRESHOLD):
            self._diagnostic_stato_motore = "failure"
            self.status.is_motor_ok = False
            failure_type = []
            if self._translation_fail_count >= MOTOR_FAIL_THRESHOLD:
                failure_type.append("traslazione bloccata")
            if self._rotation_fail_count >= MOTOR_FAIL_THRESHOLD:
                failure_type.append("rotazione bloccata")
            if self._drift_fail_count >= MOTOR_FAIL_THRESHOLD:
                failure_type.append("deriva")
            rospy.logerr(
                f"GUASTO MOTORE RILEVATO: {', '.join(failure_type)}"
            )
        else:
            self._diagnostic_stato_motore = None
            self.status.is_motor_ok = True

        # Aggiorna stato precedente e resetta flag comandi
        self._prev_gps_pos = self._current_gps_pos
        self._prev_heading = self._current_heading
        self._cmd_vel_active_linear = False
        self._cmd_vel_active_angular = False

    # ------------ VALUTAZIONE STATO SISTEMA ------------

    def _evaluate_gps_subsystems(self):
        """Valuta lo stato dei sottosistemi GPS e aggiorna self.status."""
        self.status.is_gpsvel = self._diagnostic_stato_gpsvel in [3, 4]
        self.status.is_gps_hdt = (self._diagnostic_stato_gpshdt == 0)
        self.status.is_gpspos = self._diagnostic_stato_gps in [1, 2, 3]
        self.status.gps_type = self._diagnostic_stato_gps
        self.status.gps_vel_type = self._diagnostic_stato_gpsvel

    def _evaluate_imu_subsystems(self):
        """Valuta lo stato dei sottosistemi IMU e aggiorna self.status."""
        self.status.is_imu = bool(self._diagnostic_stato_imu)
        self.status.is_magnetometer = self._diagnostic_stato_mag in [2, 3]

    def get_usv_status(self):
        """Determina lo stato complessivo del sistema in base ai sensori.

        Returns:
            SystemStatusStruct: struttura con lo stato di ogni sottosistema.
        """
        # TODO PRODUZIONE: gestire _diagnostic_stato_generale = None (boot).
        # Prima che i callback dei sensori vengano chiamati, i default di
        # SystemStatusStruct riportano tutti i sensori come attivi.
        # Impostare sensor_presence=False come default o aggiungere guard None.

        # Sensore Ellipse spento o entrambi i sotto-sensori spenti
        if self._diagnostic_stato_generale in [0, 3]:
            self.status.sensor_presence = False

        # Solo IMU spenta: valuto i sottosistemi GPS
        elif self._diagnostic_stato_generale == 1:
            self.status.imu_presence = False
            self.status.gps_presence = True
            self._evaluate_gps_subsystems()

        # Solo GPS spento: valuto i sottosistemi IMU
        elif self._diagnostic_stato_generale == 2:
            self.status.gps_presence = False
            self.status.imu_presence = True
            self._evaluate_imu_subsystems()

        # Entrambi i sensori attivi: valuto tutti i sottosistemi
        elif self._diagnostic_stato_generale == 4:
            self.status.gps_presence = True
            self.status.imu_presence = True
            self._evaluate_gps_subsystems()
            self._evaluate_imu_subsystems()

        # Verifica stato motore (indipendente dai sensori)
        self.check_motor_status()

        return self.status
