import sys
import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    print("--- Avvio Analisi Clock Bias - SwarmRaft ---")
    
    # Lista di percorsi possibili dove cercare il CSV
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

    # Verifica se la colonna clock_bias esiste nel file
    if 'clock_bias' not in df.columns:
        print("ERRORE: La colonna 'clock_bias' non è presente nel CSV.")
        print("Assicurati di aver eseguito la simulazione ns-3 aggiornata.")
        return

    # ========================================================
    # CONVERSIONE DA METRI A NANOSECONDI
    # Il C++ salva il dato equivalente in metri (Distanza = Tempo * c)
    # 1. Dividiamo per c (299792458.0) per ottenere i secondi
    # 2. Moltiplichiamo per 1e9 per ottenere i nanosecondi
    # ========================================================
    c = 299792458.0
    df['clock_bias'] = (df['clock_bias'] / c) * 1e9 

    # Fissiamo il target ID (ad esempio il drone 0, il master)
    target_id = 0
    df_target = df[df['sender_id'] == target_id]

    if df_target.empty:
        print(f"Nessun dato trovato per il Target {target_id}.")
        return

    # Creazione della finestra del grafico
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.suptitle(f'Stima del Clock Bias (Target Drone: {target_id})', fontsize=16, fontweight='bold')

    # Troviamo tutti i droni che hanno osservato il target
    observer_ids = sorted(df_target['observer_id'].unique())
    colors = plt.cm.tab10([i / max(1, len(observer_ids)-1) for i in range(len(observer_ids))])

    # Tracciamo una linea per ogni drone osservatore
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        # Ordiniamo i dati cronologicamente per evitare che le linee "tornino indietro"
        data = data.sort_values(by='time')
        
        ax.plot(data['time'], data['clock_bias'], 
                label=f'Osservatore: Drone {int(obs_id)}', 
                color=colors[i], 
                linewidth=1.5, alpha=0.8)

    ax.set_xlabel('Tempo di Simulazione [s]', fontsize=12)
    ax.set_ylabel('Clock Bias stimato [ns]', fontsize=12)  # <-- Aggiornato in nanosecondi
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # Aggiunta di una linea tratteggiata sullo zero per riferimento
    ax.axhline(0, color='black', linewidth=1, linestyle='--')

    # Legenda spostata all'esterno (in alto a destra rispetto al grafico)
    leg = ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), title="Nodi della Rete", fontsize=10)
    
    # Adatta il layout per fare spazio alla legenda
    plt.tight_layout()
    
    # Salva automaticamente il grafico in alta risoluzione
    output_filename = "clock_bias_plot_ns.png"
    plt.savefig(output_filename, dpi=150, bbox_inches='tight') 
    print(f"\nGrafico salvato come: {output_filename}")
    
    # Mostra la finestra interattiva
    plt.show()

if __name__ == "__main__":
    main()