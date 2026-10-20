import glob
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Puoi inserire sia la cartella SIA direttamente il file CSV
TARGET_PATH = "/Users/francescopolieri/Desktop/ /EMBEDDED CONTROL/Progetto Drone/flight_telemetry_20261009_163712.csv"

def get_latest_csv(folder_path):
    pattern = os.path.join(folder_path, "flight_telemetry_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)

def analyze_flight_data(file_path):
    print(f"Caricamento dati da: {file_path}")
    df = pd.read_csv(file_path)

    # Calcolo dell'errore (Setpoint - Reale)
    df["err_pitch"] = df["setpoint_pitch"] - df["drone_pitch"]
    df["err_roll"] = df["setpoint_roll"] - df["drone_roll"]
    df["err_yaw"] = df["setpoint_yaw"] - df["drone_yaw"]

    # Calcolo metriche di controllo (RMSE e Max Error)
    metrics = {}
    for axis in ["pitch", "roll", "yaw"]:
        err = df[f"err_{axis}"]
        rmse = np.sqrt(np.mean(err**2))
        max_err = np.max(np.abs(err))
        metrics[axis] = (rmse, max_err)

    print("\n--- METRICHE DI ERRORE SUI CONTROLLI ---")
    for axis, (rmse, max_err) in metrics.items():
        print(f"{axis.upper():5s} | RMSE: {rmse:6.2f}° | Max Abs Error: {max_err:6.2f}°")

    # Creazione di 2 soli subplot condivisi sull'asse X
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.canvas.manager.set_window_title(f"Analisi Pitch - {os.path.basename(file_path)}")

    t = df["time_sec"]

    # 1. Pitch: Setpoint vs Valore Reale
    axes[0].plot(t, df["setpoint_pitch"], label="SP Pitch", color="red", linestyle="--", alpha=0.8)
    axes[0].plot(t, df["drone_pitch"], label="Drone Pitch", color="darkred")
    axes[0].set_ylabel("Pitch (°)")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right")

    # 2. Errore istantaneo Pitch
    axes[1].plot(t, df["err_pitch"], label=f"Err Pitch (RMSE: {metrics['pitch'][0]:.2f}°)", color="red", alpha=0.8)
    axes[1].axhline(0, color="black", linewidth=0.8, linestyle="-")
    axes[1].set_ylabel("Errore Pitch (°)")
    axes[1].set_xlabel("Tempo (s)")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="upper right")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    elif os.path.isfile(TARGET_PATH):
        target_file = TARGET_PATH
    elif os.path.isdir(TARGET_PATH):
        target_file = get_latest_csv(TARGET_PATH)
    else:
        target_file = None

    if not target_file or not os.path.exists(target_file):
        print(f"Errore: file non trovato al percorso: {TARGET_PATH}")
        sys.exit(1)

    analyze_flight_data(target_file)