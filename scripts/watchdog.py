#!/usr/bin/env python3

# Watchdog Node
# Viene lanciato all'avvio del sistema e si occupa del monitoraggio del mission_controller.
# Verifica la presenza di vita da parte del mission_controller e in caso di mancata
# ricezione del segnale di "heartbeat" per un certo intervallo di tempo, riavvia il nodo.
# Autore: Davide Domeneghetti
# Email: d.domenehetti@sitepitalia.it
# Versione: 1.2
# Data: 11/02/2026

import subprocess

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String

# --- Costanti (valori di default, sovrascritte da parametri ROS in __init__) ---
TIMEOUT = 15.0  # Secondi senza heartbeat prima di considerare il nodo non responsivo
WATCHDOG_RATE = 2.0  # Periodo di controllo in secondi (0.5 Hz)
MAX_MC_RESTARTS = 3  # Tentativi massimi di riavvio prima di errore critico
RECOVERY_COOLDOWN = 20.0  # Secondi di attesa dopo un tentativo di recovery

ALERT_MAP = {
    0: "Funzionante",
    1: "Allarme 1: Nessun heartbeat ricevuto",
    2: "Allarme 2: Mission Controller non responsivo, tentativo di riavvio in corso...",
    3: "Allarme 3: Errore critico, impossibile riavviare il Mission Controller"
}


class Watchdog:
    """Nodo watchdog per il monitoraggio del mission_controller.

    Controlla periodicamente la ricezione dell'heartbeat e, in caso di
    mancata risposta, tenta il riavvio del nodo o esegue una fermata
    di emergenza.
    """

    def __init__(self):
        rospy.init_node('watchdog', anonymous=False)

        # Lettura parametri ROS (dal YAML caricato nel namespace del nodo)
        global TIMEOUT, WATCHDOG_RATE, MAX_MC_RESTARTS, RECOVERY_COOLDOWN
        TIMEOUT = rospy.get_param('~timeout', TIMEOUT)
        WATCHDOG_RATE = rospy.get_param('~rate', WATCHDOG_RATE)
        MAX_MC_RESTARTS = rospy.get_param('~max_restarts', MAX_MC_RESTARTS)
        RECOVERY_COOLDOWN = rospy.get_param('~recovery_cooldown', RECOVERY_COOLDOWN)

        self.timeout = TIMEOUT
        self.last_heartbeat_time = None
        self.alert_level = 0
        self.restart_attempts = 0
        self.recovery_in_progress = False
        self.last_recovery_time = 0.0

        # Subscriber heartbeat del mission_controller
        self.heartbeat_sub = rospy.Subscriber(
            '/mission_controller/heartbeat', Header, self.heartbeat_callback
        )

        # Publisher comandi di emergenza
        self.pub_emergency_stop = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1
        )
        self.pub_watchdog_status = rospy.Publisher(
            '/watchdog/status', String, queue_size=1
        )
        self.pub_abort_current_goal = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=1
        )

        # Timer periodico per il controllo dell'heartbeat
        self.periodic_timer = rospy.Timer(
            rospy.Duration(WATCHDOG_RATE), self.check_heartbeat
        )

    def heartbeat_callback(self, msg):
        """Aggiorna il timestamp dell'ultimo heartbeat ricevuto."""
        self.last_heartbeat_time = rospy.Time.now()
        if self.recovery_in_progress:
            rospy.loginfo(
                "Heartbeat ricevuto dopo recovery. "
                "Mission Controller ripristinato."
            )
            self.recovery_in_progress = False
        self.alert_level = 0
        self.restart_attempts = 0

    def check_heartbeat(self, event=None):
        """Controlla periodicamente se il mission_controller e' vivo."""
        if self.last_heartbeat_time is None:
            return  # Primo avvio, nessun heartbeat ancora ricevuto

        # Se recovery in corso, aspetta il cooldown prima di ritentare
        if self.recovery_in_progress:
            elapsed_since_recovery = (
                rospy.get_time() - self.last_recovery_time
            )
            if elapsed_since_recovery < RECOVERY_COOLDOWN:
                rospy.loginfo(
                    f"Recovery in corso, attesa "
                    f"{RECOVERY_COOLDOWN - elapsed_since_recovery:.0f}s..."
                )
                return

            # Cooldown scaduto: il recovery non ha funzionato
            self.recovery_in_progress = False
            rospy.logwarn("Recovery cooldown scaduto senza heartbeat.")

        time_since_heartbeat = (
            rospy.Time.now() - self.last_heartbeat_time
        ).to_sec()

        if time_since_heartbeat > (self.timeout * 2):
            self.alert_level = 2
            self.raise_alert(time_since_heartbeat, self.alert_level)

            if self.restart_attempts < MAX_MC_RESTARTS:
                self.attempt_recovery()
                self.restart_attempts += 1
            else:
                rospy.logerr(
                    "Impossibile riavviare il Mission Controller: "
                    "numero massimo di tentativi raggiunto."
                )
                self.alert_level = 3
                self.raise_alert(time_since_heartbeat, self.alert_level)
                self.emergency_stop()

        elif time_since_heartbeat > self.timeout:
            self.alert_level = 1
            self.raise_alert(time_since_heartbeat, self.alert_level)

    def raise_alert(self, time_since_heartbeat, alert_level):
        """Logga un avviso con il livello di allerta corrente."""
        rospy.logwarn(
            f"Mission Controller non risponde da {time_since_heartbeat:.1f}s, "
            f"livello di allerta {alert_level}: {ALERT_MAP[alert_level]}"
        )

    def attempt_recovery(self):
        """Tenta il riavvio del mission_controller.

        Usa subprocess.Popen per tutte le operazioni per evitare di bloccare
        il thread del watchdog. Lancia solo il singolo nodo mission_controller
        tramite rosrun (non l'intero launch file).
        """
        self.recovery_in_progress = True
        self.last_recovery_time = rospy.get_time()

        rospy.logwarn(
            f"Tentativo recovery {self.restart_attempts + 1}/{MAX_MC_RESTARTS}..."
        )

        # Kill non bloccante del nodo esistente
        subprocess.Popen(
            ["rosnode", "kill", "/mission_controller"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Rilancio solo del mission_controller
        subprocess.Popen(
            ["rosrun", "usv_mission_control", "mission_controller.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def emergency_stop(self):
        """Esegue la fermata di emergenza: arresta i motori e annulla il goal."""
        rospy.logfatal("EMERGENCY STOP - Mission Controller non recuperabile")
        self.pub_abort_current_goal.publish(GoalID())
        self.pub_emergency_stop.publish(Twist())
        self.pub_watchdog_status.publish("CRITICAL")


if __name__ == '__main__':
    try:
        watchdog = Watchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
