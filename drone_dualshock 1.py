import asyncio
import csv
from datetime import datetime
import struct
import sys
import threading
import time
from collections import deque
from bleak import BleakClient, BleakScanner
import pygame
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# --- CONFIGURAZIONI BLE ---
DEVICE_NAME = "DRONE"
SETPOINT_CHAR_UUID = "669a0c21-0008-417c-a496-d4bd8253d865"
TELEMETRY_CHAR_UUID = "669a0c23-0008-417c-a496-d4bd8253d865"

# --- LIMITI COMANDI E CONTROLLER ---
MAX_THROTTLE = 55999.0
MAX_ANGLE = 30.0
DEADZONE = 0.08
THROTTLE_RATE = 30000.0

# Mappatura assi DualShock 4 su macOS
AXIS_LEFT_X = 0   # Yaw
AXIS_RIGHT_X = 2  # Roll
AXIS_RIGHT_Y = 3  # Pitch
AXIS_L2 = 4       # Throttle Down
AXIS_R2 = 5       # Throttle Up

# --- STATO GLOBALE PROTETTO DA MUTEX ---
state_lock = threading.Lock()
current_pitch = 0.0      
current_roll = 0.0       
current_yaw = 0.0        
current_throttle = 0.0   

# Evento asincrono per la chiusura cooperativa del thread BLE
ble_stop_event = None
is_running = True
last_poll_time = time.time()

# --- TELEMETRIA REAL-TIME (GRAFICI) ---
MAX_POINTS = 200 
x_data = list(range(MAX_POINTS))
telemetry_pitch = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
telemetry_roll = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
telemetry_yaw = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
telemetry_alt = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)

# Ultimi valori telemetria letti (per sincronizzazione nel log)
latest_rx_telemetry = (0.0, 0.0, 0.0, 0.0)

# Buffer in memoria per il log sincrono
log_records = []
session_start_time = None

def apply_deadzone(value, threshold=DEADZONE):
    if abs(value) < threshold:
        return 0.0
    return value

def get_trigger_value(raw_value):
    normalized = (raw_value + 1.0) / 2.0
    if normalized < DEADZONE:
        return 0.0
    return normalized

# --- CALLBACK TELEMETRIA BLE ---
def telemetry_notification_handler(sender, data):
    global latest_rx_telemetry
    if len(data) == 16:
        p, r, y, a = struct.unpack("<ffff", data)
        telemetry_pitch.append(p)
        telemetry_roll.append(r)
        telemetry_yaw.append(y)
        telemetry_alt.append(a)
        latest_rx_telemetry = (p, r, y, a)

# --- MOTORE BLE ---
async def send_telemetry_loop(client: BleakClient):
    global ble_stop_event, session_start_time
    print("\n--- CONTROLLO ATTIVO: COMANDI ABILITATI ---")
    await client.start_notify(TELEMETRY_CHAR_UUID, telemetry_notification_handler)

    target_interval = 0.02  # 50 Hz
    session_start_time = time.monotonic()

    try:
        while not ble_stop_event.is_set() and client.is_connected:
            loop_start = time.monotonic()
            timestamp_sec = loop_start - session_start_time

            # Lettura non bloccante dei comandi attuali
            with state_lock:
                sp_pitch, sp_roll, sp_yaw, sp_thr = current_pitch, current_roll, current_yaw, current_throttle

            # Trasmissione verso il drone
            payload = struct.pack("<ffff", sp_pitch, sp_roll, sp_yaw, sp_thr)
            await client.write_gatt_char(SETPOINT_CHAR_UUID, payload, response=False)

            # Acquisizione telemetria sincronizzata al frame temporale
            rx_p, rx_r, rx_y, rx_alt = latest_rx_telemetry

            # Registrazione nel buffer con base temporale comune
            log_records.append([
                f"{timestamp_sec:.4f}",
                round(sp_pitch, 2),
                round(sp_roll, 2),
                round(sp_yaw, 2),
                round(sp_thr, 1),
                round(rx_p, 2),
                round(rx_r, 2),
                round(rx_y, 2),
                round(rx_alt, 3)
            ])

            sys.stdout.write(
                f"\rTHR: {sp_thr:5.0f} | TX (P,R,Y): {sp_pitch:4.0f}, {sp_roll:4.0f}, {sp_yaw:4.0f} | "
                f"RX Alt: {rx_alt:4.1f}m"
            )
            sys.stdout.flush()

            elapsed = time.monotonic() - loop_start
            sleep_time = max(0.0, target_interval - elapsed)
            
            try:
                await asyncio.wait_for(ble_stop_event.wait(), timeout=sleep_time)
                break
            except asyncio.TimeoutError:
                pass

    finally:
        print("\nInvio segnale disarmo e spegnimento motori...")
        try:
            shutdown_payload = struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0)
            await client.write_gatt_char(SETPOINT_CHAR_UUID, shutdown_payload, response=False)
            await client.stop_notify(TELEMETRY_CHAR_UUID)
            await asyncio.sleep(0.05)
        except Exception:
            pass

async def ble_main():
    global is_running, ble_stop_event
    ble_stop_event = asyncio.Event()

    print(f"Scansione BLE alla ricerca del dispositivo '{DEVICE_NAME}'...")
    try:
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: d.name and DEVICE_NAME in d.name
        )
    except Exception as e:
        print(f"\nErrore durante la scansione BLE: {e}")
        is_running = False
        return

    if not device:
        print(f"\nErrore: drone '{DEVICE_NAME}' non trovato.")
        is_running = False
        return

    print(f"Drone trovato [{device.address}]. Connessione in corso...")
    try:
        async with BleakClient(device.address) as client:
            print("Connessione BLE stabilita!")
            await send_telemetry_loop(client)
    except Exception as e:
        print(f"\nDisconnessione o errore BLE: {e}")
    finally:
        is_running = False

def ble_thread_runner():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_main())
    except Exception as e:
        print(f"\nEccezione nel thread BLE: {e}")
    finally:
        pending = asyncio.all_tasks(loop)
        if pending:
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        loop.close()

# --- GESTIONE GRAFICI E JOYSTICK (Main Thread) ---
def setup_application(js):
    global is_running, last_poll_time
    global current_pitch, current_roll, current_yaw, current_throttle

    plt.style.use('fast')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    fig.canvas.manager.set_window_title('Telemetria Live Drone & Controller PS4')

    ax1.set_title("Pitch & Roll")
    ax1.set_xlim(0, MAX_POINTS)
    ax1.set_xticks([])
    line_pitch, = ax1.plot(x_data, list(telemetry_pitch), label='Pitch', color='red', animated=False)
    line_roll, = ax1.plot(x_data, list(telemetry_roll), label='Roll', color='blue', animated=False)
    ax1.legend(loc="upper right")

    ax2.set_title("Yaw")
    ax2.set_xlim(0, MAX_POINTS)
    ax2.set_xticks([])
    line_yaw, = ax2.plot(x_data, list(telemetry_yaw), label='Yaw', color='green', animated=False)
    ax2.legend(loc="upper right")

    ax3.set_title("Altitude (m)")
    ax3.set_xlim(0, MAX_POINTS)
    ax3.set_xticks([])
    line_alt, = ax3.plot(x_data, list(telemetry_alt), label='Alt', color='orange', animated=False)
    ax3.legend(loc="upper right")

    plt.tight_layout()

    def update(frame):
        global is_running, last_poll_time
        global current_pitch, current_roll, current_yaw, current_throttle

        if not is_running:
            plt.close(fig)
            return line_pitch, line_roll, line_yaw, line_alt

        now = time.monotonic()
        dt = now - last_poll_time
        last_poll_time = now

        pygame.event.pump()

        l2_val = get_trigger_value(js.get_axis(AXIS_L2))
        r2_val = get_trigger_value(js.get_axis(AXIS_R2))
        throttle_delta = (r2_val - l2_val) * THROTTLE_RATE * dt

        yaw_val = apply_deadzone(js.get_axis(AXIS_LEFT_X)) * MAX_ANGLE
        roll_val = apply_deadzone(js.get_axis(AXIS_RIGHT_X)) * MAX_ANGLE
        pitch_val = -apply_deadzone(js.get_axis(AXIS_RIGHT_Y)) * MAX_ANGLE

        kill_pressed = js.get_button(0)

        with state_lock:
            if kill_pressed:
                current_throttle = 0.0
                current_pitch = 0.0
                current_roll = 0.0
                current_yaw = 0.0
            else:
                if throttle_delta != 0.0:
                    current_throttle += throttle_delta
                    current_throttle = max(0.0, min(MAX_THROTTLE, current_throttle))
                current_pitch = pitch_val
                current_roll = roll_val
                current_yaw = yaw_val

        if js.get_button(8) or js.get_button(9):
            is_running = False
            plt.close(fig)
            return line_pitch, line_roll, line_yaw, line_alt

        line_pitch.set_ydata(list(telemetry_pitch))
        line_roll.set_ydata(list(telemetry_roll))
        line_yaw.set_ydata(list(telemetry_yaw))
        line_alt.set_ydata(list(telemetry_alt))

        ax1.relim()
        ax1.autoscale_view(scalex=False, scaley=True)
        ax2.relim()
        ax2.autoscale_view(scalex=False, scaley=True)
        ax3.relim()
        ax3.autoscale_view(scalex=False, scaley=True)

        return line_pitch, line_roll, line_yaw, line_alt

    ani = animation.FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)
    plt.show()

import os

def export_session_to_csv():
    if not log_records:
        print("Nessun dato registrato da esportare.")
        return

    # Percorso assoluto della cartella di destinazione
    target_dir = "/Users/francescopolieri/Desktop/ /EMBEDDED CONTROL/Progetto Drone"
    os.makedirs(target_dir, exist_ok=True)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join(target_dir, f"flight_telemetry_{timestamp_str}.csv")

    headers = [
        "time_sec",
        "setpoint_pitch",
        "setpoint_roll",
        "setpoint_yaw",
        "setpoint_throttle",
        "drone_pitch",
        "drone_roll",
        "drone_yaw",
        "drone_altitude"
    ]

    try:
        with open(file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(headers)
            writer.writerows(log_records)
        print(f"\nDati esportati con successo in: {file_path} ({len(log_records)} campioni)")
    except Exception as e:
        print(f"\nErrore durante l'esportazione del file CSV: {e}")

# --- ENTRY POINT ---
if __name__ == "__main__":
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Errore: Nessun controller rilevato. Collega il DualShock 4 e riprova.")
        pygame.quit()
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller pronto: {js.get_name()}")

    ble_thread = threading.Thread(target=ble_thread_runner, daemon=True)
    ble_thread.start()

    try:
        setup_application(js)
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        if ble_stop_event:
            ble_stop_event.set()

        print("\nChiusura controllata dei thread...")
        pygame.quit()
        ble_thread.join(timeout=2.0)
        
        # Scrittura del CSV al termine della sessione
        export_session_to_csv()
        print("Sessione terminata con successo.")