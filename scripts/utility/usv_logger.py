# USV Logger
# Sistema di logging per il monitoraggio delle attività del mission_controller e di alcune funzionalità dell'USV.
# In particolare Crea log organizzati per:
# - Sistema (eventi generali)
# - Missione (telemetria e waypoint)
# - Errori (solo problemi)
# - Performance (CPU, RAM, latenza)
# Versione: 1.0
# Data: 06/02/2026

#!/usr/bin/env python3

import logging
from logging.handlers import RotatingFileHandler, TimedRotatingFileHandler
import os
from datetime import datetime
import json

class USVLogger:
    """
    Gestisce 4 tipi di log separati:
    1. system.log - Eventi sistema (cambio stato, comandi)
    2. mission_YYYYMMDD.log - Telemetria missione (rotazione giornaliera)
    3. error.log - Solo errori e warning critici
    4. performance.log - Metriche performance sistema
    """

    def __init__(self, log_dir="/home/usv/missions/logs/", node_name="mission_controller"):
        self.log_dir = log_dir
        self.node_name = node_name

        # Crea directory log se non esiste
        os.makedirs(log_dir, exist_ok=True)

        # Inizializza i 4 logger
        self.system_logger = self._setup_system_logger()
        self.mission_logger = self._setup_mission_logger()
        self.error_logger = self._setup_error_logger()
        self.performance_logger = self._setup_performance_logger()

    def _setup_system_logger(self):
        logger = logging.getLogger(f"{self.node_name}_system")
        logger.setLevel(logging.DEBUG)

        # File handler con rotazione giornaliera e dimensione massima di 10MB e 5 backup
        handler = RotatingFileHandler(
            filename=os.path.join(self.log_dir, "system.log"),
            maxBytes=10*1024*1024, # 10MB
            backupCount=5
        )

        # Fomato log con timestamp
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)-8s] [%(name)s] - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger
    
    def _setup_mission_logger(self):
        """ Log per la telemetria della missione con rotazione giornaliera """
        logger = logging.getLogger(f'{self.node_name}_mission')
        logger.setLevel(logging.INFO)

        # File handler con rotazione giornaliera, idealmente mezzanotte
        handler = TimedRotatingFileHandler(
            filename=os.path.join(self.log_dir, "mission.log"),
            when='midnight',
            interval=1,
            backupCount=3  # Mantiene log degli ultimi 3 giorni
        )

        handler.suffix = "%Y%m%d"  # Aggiunge data al nome del file

        # Fomrato della telemetria compatibile con formato CSV per analisi successiva
        formatter = logging.Formatter(
            '%(asctime)s,%(levelname)s,%(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger
    
    def _setup_error_logger(self):
        logger = logging.getLogger(f"{self.node_name}_error")
        logger.setLevel(logging.WARNING)

        handler = RotatingFileHandler(
            filename=os.path.join(self.log_dir, 'error.log'),
            maxBytes=5*1024*1024,  # 5 MB
            backupCount=10
        )
    
        # Formato deve essere molto dettagliato per gli errori
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)-8s] [%(name)s:%(lineno)d] %(message)s\n'
            'Function: %(funcName)s\n'
            '---',
            datefmt='%Y-%m-%d %H:%M:%S.%f'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        
        return logger
    
    def _setup_performance_logger(self):
        """ Log metriche di performance """
        logger = logging.getLogger(f'{self.node_name}.performance')
        logger.setLevel(logging.INFO)

        handler = TimedRotatingFileHandler(
            filename=os.path.join(self.log_dir, 'performance.log'),
            when='H',  # Rotazione oraria
            interval=1,
            backupCount=24  # Mantiene log delle ultime 24 ore
        )

        formatter = logging.Formatter(
            '%(asctime)s,%(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger
    
    """ Metodi pubblici per il logging"""
    def log_state_change(self, old_state, new_state, reason=""):
        """Logga cambio stato FSM"""
        msg = f"STATE_CHANGE: {old_state} -> {new_state}"
        if reason:
            msg += f" | Reason: {reason}"
        self.system_logger.info(msg)
    
    def log_waypoint(self, wp_index, total_wp, lat, lon, status):
        """Logga raggiungimento waypoint"""
        msg = f"WAYPOINT: {wp_index}/{total_wp},LAT={lat:.6f},LON={lon:.6f},STATUS={status}"
        self.mission_logger.info(msg)
    
    def log_telemetry(self, lat, lon, sog, cog, state):
        """Logga telemetria (CSV format per parsing facile)"""
        msg = f"TELEMETRY,{lat:.6f},{lon:.6f},{sog:.2f},{cog:.1f},{state}"
        self.mission_logger.info(msg)
    
    def log_command(self, cmd_type, linear_vel, angular_vel):
        """Logga comandi inviati"""
        msg = f"CMD: type={cmd_type}, linear={linear_vel:.2f}, angular={angular_vel:.2f}"
        self.system_logger.info(msg)
    
    def log_error(self, error_type, error_msg, context=None):
        """Logga errori critici"""
        msg = f"ERROR: {error_type} | {error_msg}"
        if context:
            msg += f" | Context: {context}"
        self.error_logger.error(msg)
        self.system_logger.error(msg)  # Scrivi anche in system.log
    
    def log_warning(self, warning_msg):
        """Logga warning"""
        self.system_logger.warning(f"WARNING: {warning_msg}")
        self.error_logger.warning(warning_msg)
    
    def log_performance(self, cpu_percent, mem_mb, loop_time_ms):
        """Logga metriche performance"""
        msg = f"CPU={cpu_percent:.1f}%,MEM={mem_mb:.1f}MB,LOOP={loop_time_ms:.2f}ms"
        self.performance_logger.info(msg)
    
    def log_mission_start(self, mission_id, num_waypoints):
        """Logga inizio missione"""
        msg = f"MISSION_START: ID={mission_id}, waypoints={num_waypoints}"
        self.system_logger.info(msg)
        self.mission_logger.info(msg)
    
    def log_mission_end(self, mission_id, status, waypoints_reached, duration_sec):
        """Logga fine missione"""
        msg = f"MISSION_END: ID={mission_id}, status={status}, reached={waypoints_reached}, duration={duration_sec:.1f}s"
        self.system_logger.info(msg)
        self.mission_logger.info(msg)
    
    def log_sensor_status(self, sensor_name, status, details=""):
        """Logga stato sensori"""
        msg = f"SENSOR: {sensor_name} = {status}"
        if details:
            msg += f" | {details}"
        self.system_logger.info(msg)