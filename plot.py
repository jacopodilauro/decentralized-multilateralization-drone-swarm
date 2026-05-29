import sys
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
from matplotlib.widgets import Button

def disegna_geofence(ax):
    # Coordinate esatte del Geofence come in UwbSecurityApp.cpp
    x0, x1 = 70.0, 130.0
    y0, y1 = 70.0, 130.0
    z0, z1 = 20.0, 80.0

    # 1. Disegna i due quadrati (Base e Soffitto)
    for z in [z0, z1]:
        ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], [z, z, z, z, z], 
                color='red', linestyle='-', linewidth=1.5, 
                label='Geofence Area (Core)' if z == z0 else "")

    # 2. Disegna i 4 pilastri verticali del cubo
    for x in [x0, x1]:
        for y in [y0, y1]:
            ax.plot([x, x], [y, y], [z0, z1], color='red', linestyle='-', linewidth=1.5)

    # 3. Disegna il guscio di Isteresi 3D
    cx, cy, cz = (x0+x1)/2, (y0+y1)/2, (z0+z1)/2
    raggio_isteresi = 14.0 
    
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    sx = cx + raggio_isteresi * np.outer(np.cos(u), np.sin(v))
    sy = cy + raggio_isteresi * np.outer(np.sin(u), np.sin(v))
    sz = cz + raggio_isteresi * np.outer(np.ones(np.size(u)), np.cos(v))
    
    ax.plot_surface(sx, sy, sz, color='orange', alpha=0.1)

def main():
    print("--- Avvio Dashboard Distry MLAT-26 ---")
    numero = int(sys.argv[1]) if len(sys.argv) > 1 else 8

    possible_paths = [
        'tdma_security_log.csv',
        '../tdma_security_log.csv',
        '../../tdma_security_log.csv',
        '../../../tdma_security_log.csv'
    ]

    csv_file = None
    for path in possible_paths:
        if os.path.exists(path):
            csv_file = path
            break

    if csv_file is None:
        print("ERRORE: File 'tdma_security_log.csv' non trovato.")
        return

    print(f"Dati caricati con successo da: {csv_file}")
    df = pd.read_csv(csv_file)

    # --- INIZIO DEBUGGER ESTREMO ---
    print("\n================ DEBUGGER PYTHON ================")
    print(f"Percorso assoluto del file letto:\n{os.path.abspath(csv_file)}")
    
    df_debug = df[(df['time'] >= 238.0) & (df['time'] <= 239.0)]
    if not df_debug.empty:
        print(f"Media nodi attivi (t=238-239s): {df_debug['active_nodes'].mean():.1f}")
        print(f"Media soglia (t=238-239s): {df_debug['threshold'].mean():.1f}")
    else:
        print("ATTENZIONE: Nessun dato trovato tra t=238s e t=239s nel CSV!")
    print("=================================================\n")
    # --- FINE DEBUGGER ESTREMO ---

    target_id = 0
    df_target = df[df['sender_id'] == target_id]

    if df_target.empty:
        print(f"Nessun dato trovato per il Target {target_id}.")
        return

    observer_ids = sorted(df_target['observer_id'].unique())
    num_observers = len(observer_ids)

    # --- CALCOLO DRONI NEL GEOFENCE ---
    geofence = {'x': (70.0, 130.0), 'y': (70.0, 130.0), 'z': (20.0, 80.0)}
    gt_path = csv_file.replace('tdma_security_log.csv', 'ground_truth.csv')
    drones_inside = None

    if os.path.exists(gt_path):
        df_gt = pd.read_csv(gt_path)
        def count_inside_geofence(group):
            inside = (
                group['true_x'].between(*geofence['x']) &
                group['true_y'].between(*geofence['y']) &
                group['true_z'].between(*geofence['z'])
            )
            return inside.sum()
        # Calcoliamo quanti nodi sono dentro al cubo per ogni timestamp
        drones_inside = df_gt.groupby('time').apply(count_inside_geofence)
    else:
        print("Attenzione: ground_truth.csv non trovato. Conteggio droni fisici disabilitato.")

    fig = plt.figure(figsize=(16, 9))
    fig.suptitle('Dashboard Sicurezza SwarmRaft - Distry MLAT-26', fontsize=18, fontweight='bold')
    
    # --- PANNELLO 1: Vista 3D ---
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax_3d.set_title("Ricostruzione Traiettoria 3D vs Spoofing")

    label_base_added = False
    label_guest_added = False
    label_fault_added = False
    global_max_time = df['time'].max()

    try:
        gt_path = csv_file.replace('tdma_security_log.csv', 'ground_truth.csv')
        df_gt = pd.read_csv(gt_path)
        for node_id in df_gt['node_id'].unique():
            df_node_gt = df_gt[df_gt['node_id'] == node_id]
            ax_3d.plot(df_node_gt['true_x'], df_node_gt['true_y'], df_node_gt['true_z'],
                       color='green', linestyle=':', alpha=0.3, linewidth=1.0)
    except FileNotFoundError:
        print(f"Attenzione: ground_truth.csv non trovato. Salto le orbite fisiche.")

    for obs_id in observer_ids:
        df_anchor = df[df['sender_id'] == obs_id]
        if not df_anchor.empty:
            first_obs = df_anchor['observer_id'].iloc[0]
            df_anchor_unique = df_anchor[df_anchor['observer_id'] == first_obs]

            if obs_id < numero:
                lbl = 'Traiettoria Ancore Base' if not label_base_added else ""
                ax_3d.plot(df_anchor_unique['true_x'], df_anchor_unique['true_y'], df_anchor_unique['true_z'],
                           color='cadetblue', linewidth=0.6, alpha=0.4, label=lbl)
                label_base_added = True

                last_time = df_anchor_unique['time'].max()
                if last_time < global_max_time - 2.0:
                    t75 = df_anchor_unique['time'].quantile(0.75)
                    orbit_row = df_anchor_unique[df_anchor_unique['time'] <= t75].iloc[-1]
                    lbl_fault = 'Guasto/Abbandono' if not label_fault_added else ""
                    ax_3d.scatter(orbit_row['true_x'], orbit_row['true_y'], orbit_row['true_z'],
                                  color='darkred', marker='D', s=50, depthshade=False, label=lbl_fault)
                    label_fault_added = True
            else:
                lbl = 'Traiettoria Droni Ospiti (Join/Leave)' if not label_guest_added else ""
                ax_3d.plot(df_anchor_unique['true_x'], df_anchor_unique['true_y'], df_anchor_unique['true_z'],
                           color='orange', linewidth=1.5, alpha=0.8, label=lbl)
                ax_3d.scatter(df_anchor_unique['true_x'].iloc[0],
                              df_anchor_unique['true_y'].iloc[0],
                              df_anchor_unique['true_z'].iloc[0],
                              color='darkorange', marker='o', s=30)
                label_guest_added = True

    sample_obs = observer_ids[0]
    df_sample = df_target[df_target['observer_id'] == sample_obs]

    ax_3d.plot(df_sample['true_x'], df_sample['true_y'], df_sample['true_z'],
               color='black', linewidth=3, label='Posizione Reale Target')
    ax_3d.plot(df_sample['claim_x'], df_sample['claim_y'], df_sample['claim_z'],
               color='red', linestyle='--', linewidth=2, label='GPS Dichiarato (Attacco)')
    ax_3d.plot(df_sample['rec_x'], df_sample['rec_y'], df_sample['rec_z'],
               color='limegreen', linewidth=2, alpha=0.8, label='Posizione Recuperata (SwarmRaft)')

    disegna_geofence(ax_3d)

    ax_3d.set_xlabel('X [m]')
    ax_3d.set_ylabel('Y [m]')
    ax_3d.set_zlabel('Z [m]')
    ax_3d.legend(loc='upper right')

    # --- PANNELLO 2: Errore EKF ---
    ax_err = fig.add_subplot(2, 2, 2)
    ax_err.set_title("Errore di Stima EKF dei singoli Droni")

    colors = plt.cm.tab10(np.linspace(0, 1, num_observers))
    lines = []
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        line, = ax_err.plot(data['time'], data['estimation_error'],
                            label=f'Drone {obs_id}', color=colors[i], alpha=0.7)
        lines.append(line)

    ax_err.set_ylabel('Errore [m]')
    ax_err.grid(True, linestyle='--', alpha=0.5)

    leg = ax_err.legend(fontsize='x-small', ncol=4, loc='upper right',
                        title='Click per mostrare/nascondere')
    leg.set_visible(False) 

    ax_btn = plt.axes([0.82, 0.92, 0.15, 0.04])
    btn_legend = Button(ax_btn, 'Apri/Chiudi Legenda')

    def toggle_legend(event):
        is_visible = leg.get_visible()
        leg.set_visible(not is_visible)
        fig.canvas.draw_idle()

    btn_legend.on_clicked(toggle_legend)
    fig.btn_legend = btn_legend 

    lined = dict(zip(leg.get_lines(), lines))
    for legline in leg.get_lines():
        legline.set_picker(True)
        legline.set_pickradius(8)

    def on_pick(event):
        legline = event.artist
        if legline not in lined:
            return
        origline = lined[legline]
        origline.set_visible(not origline.get_visible())
        legline.set_alpha(1.0 if origline.get_visible() else 0.2)
        fig.canvas.draw_idle()

    cid = fig.canvas.mpl_connect('pick_event', on_pick)

    # --- PANNELLO 3: Consenso SwarmRaft ---
    ax_vote = fig.add_subplot(2, 2, 4, sharex=ax_err)
    ax_vote.set_title("Consenso SwarmRaft & Allarmi (Dati Live dal C++)")
    # Raggruppiamo i dati per timestamp estraendo i valori REALI calcolati dal C++
    df_grouped = df_target.groupby('time').agg({
        'total_votes': 'max',
        'threshold': 'max',
        'active_nodes': 'max'
    })
    
    unique_times = df_grouped.index.to_numpy()
    true_votes = df_grouped['total_votes'].to_numpy()
    true_threshold = df_grouped['threshold'].to_numpy()
    true_active_nodes = df_grouped['active_nodes'].to_numpy()

        # 1. Linea Blu: Droni attivi che comunicano (activeObservers in C++)
    # Sostituisci la linea blu con il conteggio fisico dal ground truth
    if drones_inside is not None:
        gt_times = drones_inside.index.to_numpy()
        gt_counts = drones_inside.to_numpy()
        ax_vote.plot(gt_times, gt_counts, color='blue', linewidth=2, 
                    label='Droni nel Geofence (Ground Truth)')    
    # 2. Linea Rossa Tratteggiata: La VERA soglia dinamica calcolata dal C++
    ax_vote.plot(unique_times, true_threshold, color='red', linestyle='--', label='Soglia Maggioranza (C++)')
    
    # 3. Linea Viola: I voti totali reali accumulati dalla rete
    ax_vote.plot(unique_times, true_votes, color='purple', linewidth=2, label='Voti Totali (Allarmi)')

    # 4. Area Rossa: Si attiva ESATTAMENTE quando i voti superano la soglia del C++
    max_y = true_active_nodes.max() if len(true_active_nodes) > 0 else 40
    ax_vote.fill_between(unique_times, 0, max_y + 1, 
                         where=(true_votes >= true_threshold),
                         color='red', alpha=0.15, label='Sistema in Protezione')
                         
    ax_vote.set_ylim(0, max(gt_counts.max(), true_threshold.max()) * 1.3)
    ax_vote.set_xlabel('Tempo di Simulazione [s]')
    ax_vote.set_ylabel('Conteggio (Nodi / Voti)')
    ax_vote.grid(True, linestyle='--', alpha=0.5)
    ax_vote.legend(fontsize='small', loc='upper right')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
