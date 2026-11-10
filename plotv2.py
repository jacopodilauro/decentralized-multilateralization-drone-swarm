"""
Dashboard della simulazione, versione 2 (in inglese): traiettorie 3D, errore EKF degli osservatori e consenso.
Il pulsante "Per-drone votes" sostituisce il pannello del consenso sul bersaglio con una griglia di piccoli
grafici, uno per drone, con i voti "sospetto" contro quel drone e la soglia di quorum.

Uso:
    python3 plotv2.py                              # cerca tdma_security_log.csv qui e nelle cartelle sopra
    python3 plotv2.py 20                           # 20 = numero di droni base
    python3 plotv2.py --csv runs/demo/seed1/tdma_security_log.csv
    python3 plotv2.py --target 3                   # bersaglio diverso dal nodo 0 (come --targetsId)
    python3 plotv2.py --view grid                  # parte dalla griglia per drone
    python3 plotv2.py --save figura.png            # salva l'immagine invece di aprire la finestra
"""
import argparse
import math
import os

import matplotlib
import numpy as np
import pandas as pd

# Geofence come in UwbSecurityApp.cpp: distanza 2D (solo x, y) dal quadrato 70-130 m.
# Un ospite entra in ascolto sotto i 10 m e avvia l'uscita oltre i 14 m, a qualunque quota.
GEO_MIN, GEO_MAX = 70.0, 130.0
GEO_ENTRY_M, GEO_EXIT_M = 10.0, 14.0

# Riquadro della colonna destra in cui vivono, alternandosi, il pannello del consenso e la griglia per drone
VOTE_BOX = (0.555, 0.065, 0.425, 0.375)
ACCENT = "#0072B2"


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
            (0.0, '-', 'red', 'Geofence (70-130 m square)'),
            (GEO_ENTRY_M, '--', 'darkorange', f'Guest entry (< {GEO_ENTRY_M:.0f} m, 2D)'),
            (GEO_EXIT_M, ':', 'darkorange', f'Guest exit (> {GEO_EXIT_M:.0f} m, 2D)')]:
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


def serie_voti(df, sender_id):
    """Per un drone osservato: tempi, voti "sospetto" contro di lui, osservatori presenti e soglia di quorum.
    Stessa logica del vecchio pannello del consenso (ffill per gli osservatori che escono)."""
    d = df[df['sender_id'] == sender_id]
    if d.empty:
        return None
    colonna = 'my_vote' if 'my_vote' in d.columns else 'alarm'
    pivot = d.pivot_table(index='time', columns='observer_id', values=colonna)
    presenti = pivot.notna().ffill().sum(axis=1).to_numpy()
    pivot = pivot.ffill().fillna(0)
    soglia = d.groupby('time')['threshold'].median().reindex(pivot.index).ffill().to_numpy()
    return pivot.index.to_numpy(), pivot.sum(axis=1).to_numpy(), presenti, soglia, colonna


def main():
    ap = argparse.ArgumentParser(description="Distry MLAT-26 dashboard v2")
    ap.add_argument("n_base", nargs="?", type=int, default=8, help="numero di droni base (default 8)")
    ap.add_argument("--csv", help="percorso di tdma_security_log.csv")
    ap.add_argument("--target", type=int, default=0, help="ID del bersaglio da analizzare (default 0)")
    ap.add_argument("--view", choices=["target", "grid"], default="target",
                    help="vista iniziale del pannello in basso: consenso sul bersaglio o griglia per drone")
    ap.add_argument("--save", help="salva l'immagine in questo file invece di aprire la finestra")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registra la proiezione 3D)

    print("--- Distry MLAT-26 dashboard v2 ---")
    csv_file = trova_csv(args.csv)
    if csv_file is None:
        print("ERROR: tdma_security_log.csv not found.")
        print("Run the ns-3 simulation first, or point me to the file with --csv.")
        return
    print(f"Data loaded from: {csv_file}")
    df = pd.read_csv(csv_file)

    numero = args.n_base
    target_id = args.target
    df_target = df[df['sender_id'] == target_id]
    if df_target.empty:
        print(f"No data for target drone {target_id}.")
        return

    observer_ids = sorted(df_target['observer_id'].unique())
    num_observers = len(observer_ids)
    sender_ids = sorted(df['sender_id'].unique())

    fig = plt.figure(figsize=(16, 9))
    fig.suptitle('Distry MLAT-26 security dashboard', fontsize=18, fontweight='bold')

    # --- PANNELLO 1: Vista 3D (posizionato a mano: tight_layout non accetta gli assi 3D) ---
    ax_3d = fig.add_axes([0.01, 0.04, 0.50, 0.88], projection='3d')
    ax_3d.set_title(f"3D trajectories vs spoofing (target: drone {target_id})")

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
        print("Warning: ground_truth.csv not found, skipping the physical orbits.")

    for obs_id in observer_ids:
        df_anchor = df[df['sender_id'] == obs_id]
        if not df_anchor.empty:
            first_obs = df_anchor['observer_id'].iloc[0]
            df_anchor_unique = df_anchor[df_anchor['observer_id'] == first_obs]

            if obs_id < numero:
                lbl = 'Base anchors' if not label_base_added else ""
                ax_3d.plot(df_anchor_unique['true_x'], df_anchor_unique['true_y'], df_anchor_unique['true_z'],
                           color='cadetblue', linewidth=0.6, alpha=0.4, label=lbl)
                label_base_added = True

                last_time = df_anchor_unique['time'].max()
                if last_time < global_max_time - 2.0:
                    t75 = df_anchor_unique['time'].quantile(0.75)
                    orbit_row = df_anchor_unique[df_anchor_unique['time'] <= t75].iloc[-1]
                    lbl_fault = 'Failure / leave' if not label_fault_added else ""
                    ax_3d.scatter(orbit_row['true_x'], orbit_row['true_y'], orbit_row['true_z'],
                                  color='darkred', marker='D', s=50, depthshade=False, label=lbl_fault)
                    label_fault_added = True
            else:
                lbl = 'Guest drones (join/leave)' if not label_guest_added else ""
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
               color='black', linewidth=3, label='True target position')
    ax_3d.plot(df_sample['claim_x'], df_sample['claim_y'], df_sample['claim_z'],
               color='red', linestyle='--', linewidth=2, label='Declared GPS (attack)')
    ax_3d.plot(df_sample['rec_x'], df_sample['rec_y'], df_sample['rec_z'],
               color='limegreen', linewidth=2, alpha=0.8, label='Recovered position (swarm)')

    disegna_geofence(ax_3d, z_min, z_max)

    ax_3d.set_xlabel('X [m]')
    ax_3d.set_ylabel('Y [m]')
    ax_3d.set_zlabel('Z [m]')
    ax_3d.legend(loc='upper right', fontsize='small')

    # --- PANNELLO 2: Errore EKF, con la legenda minima (un rigo di numeri colorati, cliccabili) ---
    ax_err = fig.add_axes([0.555, 0.555, 0.425, 0.33])
    ax_err.set_title(f"EKF estimation error of drone {target_id}, one line per observer")

    colors = plt.cm.tab20(np.linspace(0, 1, max(num_observers, 2)))
    lines = []
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        line, = ax_err.plot(data['time'], data['estimation_error'],
                            label=str(obs_id), color=colors[i], alpha=0.75, linewidth=1.0)
        lines.append(line)

    ax_err.set_ylabel('Error [m]')
    ax_err.grid(True, linestyle='--', alpha=0.5)
    ax_err.margins(y=0.18)   # aria in alto per il rigo della legenda

    # Legenda minima: solo l'ID colorato di ogni osservatore, su un rigo, dentro il pannello.
    # "observer" fa da titolo in linea; un click su un ID nasconde o mostra la sua curva.
    leg = ax_err.legend(fontsize=6, ncol=min(num_observers, 20), loc='upper left',
                        frameon=False, handlelength=0.9, handletextpad=0.25,
                        columnspacing=0.55, borderaxespad=0.25, title='observer  (click an ID to hide)',
                        title_fontsize=6, alignment='left')
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

    # --- PANNELLO 3, vista A: consenso sul bersaglio ---
    # Quanti osservatori votano "sospetto" e qual e' la soglia di quorum che usa il codice
    # (max(2, ceil(2/3 degli osservatori attivi))). La zona rossa e' dove i voti raggiungono il quorum.
    ax_vote = fig.add_axes(VOTE_BOX, sharex=ax_err)
    tempi, voti, presenti, soglia, colonna_voto = serie_voti(df, target_id)
    etichetta_voti = ('observers voting "suspicious"' if colonna_voto == 'my_vote'
                      else 'observers with collective alarm (CSV without my_vote)')
    ax_vote.set_title(f'Consensus on drone {target_id}: individual votes vs quorum')
    ax_vote.plot(tempi, voti, color='purple', linewidth=2, label=etichetta_voti)
    ax_vote.plot(tempi, soglia, color='red', linestyle='--', linewidth=1.5, label='quorum threshold used by the code')
    ax_vote.plot(tempi, presenti, color='gray', linewidth=1, alpha=0.6,
                 label='observers estimating the target (guests too)')
    ax_vote.fill_between(tempi, 0, num_observers, where=(voti >= soglia),
                         color='red', alpha=0.15, label='quorum reached (alarm)')
    ax_vote.set_xlabel('Simulation time [s]')
    ax_vote.set_ylabel('Drones')
    ax_vote.set_ylim(-0.5, num_observers + 0.5)
    ax_vote.grid(True, linestyle='--', alpha=0.5)
    ax_vote.legend(fontsize=6, loc='upper left', frameon=False)

    # --- PANNELLO 3, vista B: griglia con i voti contro OGNI drone dello sciame ---
    x0, y0, w, h = VOTE_BOX
    # colonne scelte perche' ogni riquadro sia largo circa 1,5 volte l'altezza (figura 16:9)
    ncols = max(1, round(math.sqrt(len(sender_ids) * (w * 16) / (h * 9) / 1.5)))
    nrows = math.ceil(len(sender_ids) / ncols)
    gs = fig.add_gridspec(nrows, ncols, left=x0, bottom=y0, right=x0 + w, top=y0 + h - 0.028,
                          hspace=0.55, wspace=0.12)
    grid_axes, primo = [], None
    for idx, sid in enumerate(sender_ids):
        ax = fig.add_subplot(gs[idx // ncols, idx % ncols], sharex=primo, sharey=primo)
        primo = primo or ax
        t, v, _, s, _ = serie_voti(df, sid)
        ax.plot(t, v, color='purple', linewidth=0.9)
        ax.plot(t, s, color='red', linestyle='--', linewidth=0.7)
        quorum = v >= s
        ax.fill_between(t, 0, max(num_observers, 1), where=quorum, color='red', alpha=0.15)
        ax.set_title(f'drone {sid}', fontsize=6, pad=1.5,
                     color='darkred' if quorum.any() else 'black',
                     fontweight='bold' if quorum.any() else 'normal')
        ax.grid(True, linestyle='--', alpha=0.35)
        ax.tick_params(labelsize=5, length=2, pad=1)
        if idx % ncols:
            ax.tick_params(labelleft=False)
        if idx // ncols < nrows - 1 and idx + ncols < len(sender_ids):
            ax.tick_params(labelbottom=False)
        grid_axes.append(ax)
    if grid_axes:
        primo.set_ylim(-0.5, num_observers + 0.5)
        fig.text(x0 + w / 2, y0 + h - 0.012,
                 'Votes against each drone  -  purple: "suspicious" votes, dashed red: quorum, '
                 'red title: alarmed at least once', ha='center', fontsize=7)
        fig.grid_header = fig.texts[-1]

    # --- Il pulsante che alterna le due viste ---
    stato = {'grid': False}

    def mostra(grid):
        stato['grid'] = grid
        ax_vote.set_visible(not grid)
        for a in grid_axes:
            a.set_visible(grid)
        fig.grid_header.set_visible(grid)
        btn.label.set_text('Target view' if grid else 'Per-drone votes')
        fig.canvas.draw_idle()

    ax_btn = fig.add_axes([0.845, 0.935, 0.135, 0.040])
    btn = Button(ax_btn, 'Per-drone votes', color=ACCENT, hovercolor='#56B4E9')
    btn.label.set_color('white')
    btn.label.set_fontsize(9)
    btn.label.set_fontweight('bold')
    for lato in ax_btn.spines.values():
        lato.set_color(ACCENT)
    btn.on_clicked(lambda event: mostra(not stato['grid']))
    fig.btn = btn

    mostra(args.view == 'grid')

    if args.save:
        plt.savefig(args.save, dpi=100)
        print(f"Figure saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
