# Flight Control Firmware for STEVAL-FCU001V2

Questo repository contiene il firmware di controllo di volo sviluppato per la scheda di valutazione per quadricotteri STMicroelectronics STEVAL-FCU001V2. Il progetto è stato realizzato nell'ambito del corso di Embedded Control presso il Politecnico di Bari (A.A. 2025/2026).

## Panoramica
Il firmware gestisce un'architettura embedded ottimizzata per quadricotteri di piccole dimensioni. Scaricando i compiti critici di campionamento su timer hardware e DMA, il sistema garantisce un'esecuzione affidabile del loop di controllo a 416 Hz. Questa efficienza architetturale permette la coesistenza di task ad alta frequenza e operazioni asincrone come la telemetria Bluetooth Low Energy e i protocolli di sicurezza.

## Hardware Supportato
Il sistema è progettato per la board STEVAL-FCU001V2, equipaggiata con:
* **Microcontrollore:** STM32F401 Cortex-M4F (System clock a 84 MHz).
* **IMU (6-DoF):** LSM6DSR (Accelerometro e Giroscopio).
* **Barometro:** LPS22HH.
* **Connettività:** Processore di rete Bluetooth LE BlueNRG-M0A.
* **Motori:** 4 driver MOSFET low-side per motori DC a spazzole azionati da segnali PWM a 500 Hz.

## Architettura e Funzionalità
* **Acquisizione Dati Non-Bloccante:** I dati inerziali vengono campionati a 416 Hz tramite transazioni SPI DMA attivate da timer, richiedendo zero intervento della CPU durante il trasferimento su bus.
* **Stima di Stato (Sensor Fusion):**
    * Filtro complementare per il calcolo rapido degli angoli di rollio e beccheggio.
    * Stima dell'imbardata (Yaw) con dead-band di 1 rad/s per impedire l'integrazione del rumore e la deriva del giroscopio quando il drone è fermo.
    * Fusione sensoriale tra il barometro (100 Hz) e il doppio integrale dell'accelerazione sull'asse Z (416 Hz) per stimare accuratamente l'altitudine rispetto al punto di decollo.
* **Controllo PID:**
    * 3 controller PID indipendenti (Pitch, Roll, Yaw).
    * Sistema Anti-Windup con clamp dell'integrale e calcolo condizionale per evitare la saturazione prolungata dei comandi.
    * Azione derivativa calcolata direttamente sulla misurazione del giroscopio (e non sull'errore) per eliminare il "derivative kick" durante le variazioni di setpoint.
* **Comunicazione BLE e Sicurezza:**
    * Servizio GATT custom a 128-bit per disaccoppiare i comandi di riferimento (downlink) e la telemetria (uplink).
    * Trasmissione della telemetria scalata a circa 20.8 Hz per prevenire la saturazione del buffer BLE e mantenere il determinismo real-time del processore principale.
    * Failsafe automatico (stop immediato dei motori e reset del setpoint) in caso di disconnessione del client.

## Autori
* Angelo Pio Belmonte
* Davide D'Agostino
* Francesco Polieri
* Daniele Trisorio

**Supervisori:** Prof. Luca De Cicco, Ing. Nunzio Barone, Ing. Walter Brescia.
