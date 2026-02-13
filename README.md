# USV Mission Control

Questo pacchetto gestisce la logica di missione per un USV (Unmanned Surface Vehicle) tramite una FSM (Finite State Machine). Include il monitoraggio diagnostico e un sistema di watchdog.

## Struttura del Pacchetto
* `scripts/`: Nodi Python principali (`mission_controller.py`, `watchdog.py`).
* `msg/`: Messaggi custom (`Telemetry`, `MPB`, `FLOUR`).
* `launch/`: File per l'avvio del sistema.
* `config/`: Parametri di tuning YAML e configurazione log.

## Requisiti
* ROS Noetic / Ubuntu 20.04
* Python 3

## Installazione e Compilazione
1. Copia la cartella `usv_mission_control` nel tuo workspace (es. `~/catkin_ws/src/`).
2. Assicurati che gli script siano eseguibili:
   ```bash
   cd ~/catkin_ws/src/edge_mission_controller/scripts
   chmod +x *.py
