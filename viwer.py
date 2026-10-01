"""
Dashboard della simulazione: traiettorie 3D, errore EKF degli osservatori e consenso sul bersaglio.

Uso:
    python3 viwer.py                              # cerca tdma_security_log.csv qui e nelle cartelle sopra
    python3 viwer.py 8                            # come prima: 8 = numero di droni base
    python3 viwer.py --csv runs/step1.4/seed1/tdma_security_log.csv
    python3 viwer.py --target 3                   # bersaglio diverso dal nodo 0 (come --targetsId)
    python3 viwer.py --save figura.png            # salva l'immagine invece di aprire la finestra
"""
import argparse
import os

import matplotlib
import numpy as np
import pandas as pd

# Geofence come in UwbSecurityApp.cpp: distanza 2D (solo x, y) dal quadrato 70-130 m.
# Un ospite entra in ascolto sotto i 10 m e avvia l'uscita oltre i 14 m, a qualunque quota.
GEO_MIN, GEO_MAX = 70.0, 130.0
GEO_ENTRY_M, GEO_EXIT_M = 10.0, 14.0


def contorno_geofence(offset, n_arco=16):
    """Contorno dei punti a distanza 2D `offset` dal quadrato: lati dritti + quarti di cerchio agli angoli."""
    if offset <= 0:
        xs = [GEO_MIN, GEO_MAX, GEO_MAX, GEO_MIN, GEO_MIN]
        ys = [GEO_MIN, GEO_MIN, GEO_MAX, GEO_MAX, GEO_MIN]
        return np.array(xs), np.array(ys)
    angoli = [(GEO_MAX, GEO_MIN, -np.pi / 2, 0.0), (GEO_MAX, GEO_MAX, 0.0, np.pi / 2),
              (GEO_MIN, GEO_MAX, np.pi / 2, np.pi), (GEO_MIN, GEO_MIN, np.pi, 1.5 * np.pi)]
    xs, ys = [], []
    for cx, cy, a0, a1 in angoli:
        t = np.linspace(a0, a1, n_arco)
        xs.extend(cx + offset * np.cos(t))
        ys.extend(cy + offset * np.sin(t))
    xs.append(xs[0]); ys.append(ys[0])
    return np.array(xs), np.array(ys)


def disegna_geofence(ax, z_min, z_max):
    # Il geofence non dipende dalla quota: lo disegno alla quota minima e massima dei droni
    for offset, stile, colore, etichetta in [
            (0.0, '-', 'red', 'Geofence (quadrato 70-130 m)'),
            (GEO_ENTRY_M, '--', 'darkorange', f'Ingresso ospiti (< {GEO_ENTRY_M:.0f} m, 2D)'),
            (GEO_EXIT_M, ':', 'darkorange', f'Uscita ospiti (> {GEO_EXIT_M:.0f} m, 2D)')]:
        xs, ys = contorno_geofence(offset)
        for i, z in enumerate((z_min, z_max)):
            ax.plot(xs, ys, np.full_like(xs, z), color=colore, linestyle=stile, linewidth=1.3,
                    label=etichetta if i == 0 else "")
    for x in (GEO_MIN, GEO_MAX):
        for y in (GEO_MIN, GEO_MAX):
            ax.plot([x, x], [y, y], [z_min, z_max], color='red', linewidth=0.8, alpha=0.5)


def trova_csv(percorso):
    if percorso:
        return percorso if os.path.exists(percorso) else None
    for p in ['tdma_security_log.csv', '../tdma_security_log.csv',
              '../../tdma_security_log.csv', '../../../tdma_security_log.csv']:
        if os.path.exists(p):
            return p
    return None


def main():
    ap = argparse.ArgumentParser(description="Dashboard Distry MLAT-26")
    ap.add_argument("n_base", nargs="?", type=int, default=8, help="numero di droni base (default 8)")
    ap.add_argument("--csv", help="percorso di tdma_security_log.csv")
    ap.add_argument("--target", type=int, default=0, help="ID del bersaglio da analizzare (default 0)")
    ap.add_argument("--save", help="salva l'immagine in questo file invece di aprire la finestra")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registra la proiezione 3D)

    print("--- Avvio Dashboard Distry MLAT-26 ---")
    csv_file = trova_csv(args.csv)
    if csv_file is None:
        print("ERRORE: File 'tdma_security_log.csv' non trovato.")
        print("Assicurati di aver fatto girare la simulazione ns-3 prima, oppure usa --csv.")
        return
    print(f"Dati caricati con successo da: {csv_file}")
    df = pd.read_csv(csv_file)

    numero = args.n_base
    target_id = args.target
    df_target = df[df['sender_id'] == target_id]
    if df_target.empty:
        print(f"Nessun dato trovato per il Target {target_id}.")
        return

    observer_ids = sorted(df_target['observer_id'].unique())
    num_observers = len(observer_ids)

    fig = plt.figure(figsize=(16, 9))
    fig.suptitle('Dashboard Sicurezza SwarmRaft - Distry MLAT-26', fontsize=18, fontweight='bold')

    # --- PANNELLO 1: Vista 3D ---
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax_3d.set_title(f"Ricostruzione Traiettoria 3D vs Spoofing (bersaglio: Drone {target_id})")

    label_base_added = False
    label_guest_added = False
    label_fault_added = False
    global_max_time = df['time'].max()
    z_min, z_max = df['true_z'].min(), df['true_z'].max()

    gt_path = os.path.join(os.path.dirname(csv_file), 'ground_truth.csv')
    if os.path.exists(gt_path):
        df_gt = pd.read_csv(gt_path)
        z_min, z_max = min(z_min, df_gt['true_z'].min()), max(z_max, df_gt['true_z'].max())
        for node_id in df_gt['node_id'].unique():
            df_node_gt = df_gt[df_gt['node_id'] == node_id]
            ax_3d.plot(df_node_gt['true_x'], df_node_gt['true_y'], df_node_gt['true_z'],
                       color='green', linestyle=':', alpha=0.3, linewidth=1.0)
    else:
        print("Attenzione: ground_truth.csv non trovato. Salto le orbite fisiche.")

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

    disegna_geofence(ax_3d, z_min, z_max)

    ax_3d.set_xlabel('X [m]')
    ax_3d.set_ylabel('Y [m]')
    ax_3d.set_zlabel('Z [m]')
    ax_3d.legend(loc='upper right', fontsize='small')

    # --- PANNELLO 2: Errore EKF ---
    ax_err = fig.add_subplot(2, 2, 2)
    ax_err.set_title(f"Errore di Stima EKF del Drone {target_id}, per osservatore")

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
        leg.set_visible(not leg.get_visible())
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

    fig.canvas.mpl_connect('pick_event', on_pick)

    # --- PANNELLO 3: Consenso ---
    # Quanti osservatori votano "sospetto" e qual e' la soglia di quorum che usa il codice
    # (max(2, ceil(2/3 degli osservatori attivi))). La zona rossa e' dove i voti raggiungono il quorum.
    ax_vote = fig.add_subplot(2, 2, 4, sharex=ax_err)
    ax_vote.set_title(f"Consenso sul Drone {target_id}: voti individuali e soglia di quorum")

    colonna_voto = 'my_vote' if 'my_vote' in df_target.columns else 'alarm'
    pivot_votes = df_target.pivot_table(index='time', columns='observer_id', values=colonna_voto)
    presenti = pivot_votes.notna().ffill()
    pivot_votes = pivot_votes.ffill().fillna(0)
    unique_times = pivot_votes.index.to_numpy()
    votes_over_time = pivot_votes.sum(axis=1).to_numpy()
    observers_over_time = presenti.sum(axis=1).to_numpy()

    soglia = (df_target.groupby('time')['threshold'].median()
              .reindex(pivot_votes.index).ffill().to_numpy())

    etichetta_voti = ('N° osservatori che votano "sospetto"' if colonna_voto == 'my_vote'
                      else 'N° osservatori con allarme collettivo (CSV senza my_vote)')
    ax_vote.plot(unique_times, votes_over_time, color='purple', linewidth=2, label=etichetta_voti)
    ax_vote.plot(unique_times, soglia, color='red', linestyle='--', linewidth=1.5,
                 label='Soglia di quorum usata dal codice')
    ax_vote.plot(unique_times, observers_over_time, color='gray', linewidth=1, alpha=0.6,
                 label='Osservatori che stimano il bersaglio (anche ospiti fuori geofence)')
    ax_vote.fill_between(unique_times, 0, num_observers,
                         where=(votes_over_time >= soglia),
                         color='red', alpha=0.15, label='Quorum raggiunto (allarme)')

    ax_vote.set_xlabel('Tempo di Simulazione [s]')
    ax_vote.set_ylabel('N° Droni')
    ax_vote.set_ylim(-0.5, num_observers + 0.5)
    ax_vote.grid(True, linestyle='--', alpha=0.5)
    ax_vote.legend(fontsize='small', loc='upper left')

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=100)
        print(f"Immagine salvata in {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
