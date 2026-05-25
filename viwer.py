import sys
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os

# ---> FUNZIONE AGGIUNTA QUI <---
def disegna_geofence(ax):
    # --- CUBO INTERNO (Il Geofence reale 70-130) ---
    x0, x1 = 70, 130
    y0, y1 = 70, 130
    z0, z1 = 20, 80  # Altezza Z fissa per racchiudere lo sciame
    
    # Coordinate per disegnare un parallelepipedo
    x = [x0, x1, x1, x0, x0, x0, x1, x1, x0, x0, x1, x1, x1, x1, x0, x0]
    y = [y0, y0, y1, y1, y0, y0, y0, y1, y1, y0, y0, y0, y1, y1, y1, y1]
    z = [z0, z0, z0, z0, z0, z1, z1, z1, z1, z1, z1, z0, z0, z1, z1, z0]
    
    ax.plot(x, y, z, color='red', linestyle='-', linewidth=1.5, label='Geofence Area (Core)')

    # --- CUBO ESTERNO (La soglia di Trigger a 10m) ---
    # Si allarga di 10 metri per ogni lato: X e Y vanno da 60 a 140
    tx0, tx1 = 60, 140
    ty0, ty1 = 60, 140
    
    tx = [tx0, tx1, tx1, tx0, tx0, tx0, tx1, tx1, tx0, tx0, tx1, tx1, tx1, tx1, tx0, tx0]
    ty = [ty0, ty0, ty1, ty1, ty0, ty0, ty0, ty1, ty1, ty0, ty0, ty0, ty1, ty1, ty1, ty1]
    
    ax.plot(tx, ty, z, color='orange', linestyle='--', linewidth=1.0, alpha=0.7, label='Soglia Radio (10m)')
# ---------------------------------


def main():
    print("--- Avvio Dashboard Distry MLAT-26 ---")
    if len(sys.argv) > 1:
        numero  = int(sys.argv[1])
    else:
        numero = 8
 
    # 1. Ricerca automatica del file CSV
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
        print("Assicurati di aver fatto girare la simulazione ns-3 prima!")
        return

    print(f"Dati caricati con successo da: {csv_file}")
    df = pd.read_csv(csv_file)

    # 2. Analisi focalizzata sul Drone 0 (il target dell'attacco spoofing)
    target_id = 0 
    df_target = df[df['sender_id'] == target_id]
    
    if df_target.empty:
        print(f"Nessun dato trovato per il Target {target_id}.")
        return

    # Estraiamo gli ID degli osservatori (gli altri droni)
    observer_ids = sorted(df_target['observer_id'].unique())
    num_observers = len(observer_ids)
    
    # Prepariamo la figura principale (Layout a griglia)
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle('Dashboard Sicurezza SwarmRaft - Distry MLAT-26', fontsize=18, fontweight='bold')
    
    # --- PANNELLO 1: Vista 3D (a sinistra) ---
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax_3d.set_title("Ricostruzione Traiettoria 3D vs Spoofing")
    
    # Disegniamo le traiettorie degli ALTRI droni (Ancore e Ospiti)
    label_base_added = False
    label_guest_added = False
    label_fault_added = False

    global_max_time = df['time'].max()
    
    try:
        # Crea il percorso per ground_truth usando lo stesso percorso trovato per il csv principale
        gt_path = csv_file.replace('tdma_security_log.csv', 'ground_truth.csv')
        
        df_gt = pd.read_csv(gt_path)
        unique_nodes = df_gt['node_id'].unique()
        
        for node_id in unique_nodes:
            # Filtriamo i dati per ogni singolo drone
            df_node_gt = df_gt[df_gt['node_id'] == node_id]
            
            # Disegniamo la linea in grigio chiaro tratteggiato usando ax_3d
            ax_3d.plot(df_node_gt['true_x'], 
                       df_node_gt['true_y'], 
                       df_node_gt['true_z'], 
                       color='green', linestyle=':', alpha=0.3, linewidth=1.0)
            
    except FileNotFoundError:
        print(f"Attenzione: {gt_path} non trovato. Salto le orbite fisiche.")
    
    for obs_id in observer_ids:
        df_anchor = df[df['sender_id'] == obs_id]
        if not df_anchor.empty:
            first_obs = df_anchor['observer_id'].iloc[0]
            df_anchor_unique = df_anchor[df_anchor['observer_id'] == first_obs]
            
            if obs_id < numero:
                # È un DRONE BASE (presente dall'inizio)
                lbl = 'Traiettoria Ancore Base' if not label_base_added else ""
                ax_3d.plot(df_anchor_unique['true_x'], df_anchor_unique['true_y'], df_anchor_unique['true_z'], 
                           color='cadetblue', linewidth=0.6, alpha=0.4, label=lbl)
                label_base_added = True

                last_time = df_anchor_unique['time'].max()
                if last_time < global_max_time - 2.0:
                    t75 = df_anchor_unique['time'].quantile(0.75)
                    orbit_row = df_anchor_unique[df_anchor_unique['time'] <= t75].iloc[-1]
                    end_x = orbit_row['true_x']
                    end_y = orbit_row['true_y']
                    end_z = orbit_row['true_z']
    
                    lbl_fault = 'Guasto/Abbandono' if not label_fault_added else ""
                    ax_3d.scatter(end_x, end_y, end_z, color='darkred', marker='D',
                    s=50, depthshade=False, label=lbl_fault)
                    label_fault_added = True
                    
            else:
                # È un DRONE OSPITE (entrato a simulazione avviata)
                lbl = 'Traiettoria Droni Ospiti (Join/Leave)' if not label_guest_added else ""
                ax_3d.plot(df_anchor_unique['true_x'], df_anchor_unique['true_y'], df_anchor_unique['true_z'], 
                           color='orange', linewidth=1.5, alpha=0.8, label=lbl)
                
                start_x = df_anchor_unique['true_x'].iloc[0]
                start_y = df_anchor_unique['true_y'].iloc[0]
                start_z = df_anchor_unique['true_z'].iloc[0]
                ax_3d.scatter(start_x, start_y, start_z, color='darkorange', marker='o', s=30)
                
                label_guest_added = True

    # Prendiamo la traiettoria reale del target
    sample_obs = observer_ids[0]
    df_sample = df_target[df_target['observer_id'] == sample_obs]
    
    # Disegniamo Realtà, Spoofing e Recupero (SwarmRaft)
    ax_3d.plot(df_sample['true_x'], df_sample['true_y'], df_sample['true_z'], 
               color='black', linewidth=3, label='Posizione Reale Target')
               
    ax_3d.plot(df_sample['claim_x'], df_sample['claim_y'], df_sample['claim_z'], 
               color='red', linestyle='--', linewidth=2, label='GPS Dichiarato (Attacco)')
               
    ax_3d.plot(df_sample['rec_x'], df_sample['rec_y'], df_sample['rec_z'], 
               color='limegreen', linewidth=2, alpha=0.8, label='Posizione Recuperata (SwarmRaft)')

    # ---> RICHIAMO DELLA FUNZIONE GEOFENCE QUI! <---
    disegna_geofence(ax_3d)

    ax_3d.set_xlabel('X [m]')
    ax_3d.set_ylabel('Y [m]')
    ax_3d.set_zlabel('Z [m]')
    ax_3d.legend(loc='upper right')


    # --- PANNELLO 2: Errore di Localizzazione (in alto a destra) ---
    ax_err = fig.add_subplot(2, 2, 2)
    ax_err.set_title("Errore di Stima EKF dei singoli Droni")
    
    colors = plt.cm.tab10(np.linspace(0, 1, num_observers))
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        ax_err.plot(data['time'], data['estimation_error'], 
                    label=f'Drone {obs_id}', color=colors[i], alpha=0.7)
        
    ax_err.set_ylabel('Errore [m]')
    ax_err.grid(True, linestyle='--', alpha=0.5)
    ax_err.legend(fontsize='small', ncol=2)


    # --- PANNELLO 3: Analisi del Consenso SwarmRaft (in basso a destra) ---
    ax_vote = fig.add_subplot(2, 2, 4, sharex=ax_err)
    ax_vote.set_title("Consenso SwarmRaft & Allarmi")
    
    pivot_alarms = df_target.pivot_table(index='time', columns='observer_id', values='alarm')
    pivot_alarms = pivot_alarms.ffill().fillna(0)

    unique_times = pivot_alarms.index.to_numpy()
    votes_over_time = pivot_alarms.sum(axis=1).to_numpy()
        
    ax_vote.plot(unique_times, votes_over_time, color='purple', linewidth=2, label='N° di Allarmi Attivi')
    
    majority_threshold = num_observers / 2.0
    ax_vote.axhline(y=majority_threshold, color='red', linestyle='--', label='Soglia Maggioranza')
    
    ax_vote.fill_between(unique_times, 0, num_observers, 
                         where=(np.array(votes_over_time) >= majority_threshold), 
                         color='red', alpha=0.15, label='Sistema in Protezione')

    ax_vote.set_xlabel('Tempo di Simulazione [s]')
    ax_vote.set_ylabel('N° Droni Allarmati')
    ax_vote.set_ylim(-0.5, num_observers + 0.5)
    ax_vote.grid(True, linestyle='--', alpha=0.5)
    ax_vote.legend(fontsize='small')

    # Mostriamo a schermo il layout unificato
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()