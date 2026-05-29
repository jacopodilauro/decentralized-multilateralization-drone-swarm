import matplotlib.pyplot as plt
import re
import os

def main():
    # Sostituisci con il nome del file in cui hai salvato l'output del terminale
    log_filepath = "debug-eviction.txt"
    
    if not os.path.exists(log_filepath):
        print(f"Errore: Il file {log_filepath} non esiste.")
        print("Assicurati di aver salvato l'output della simulazione in un file txt.")
        return

    leaves = []      # Lista per le uscite volontarie
    evictions = []   # Lista per le espulsioni (spoofing)

    with open(log_filepath, 'r') as f:
        lines = f.readlines()

    # Analisi riga per riga
    for i, line in enumerate(lines):
        
        # 1. Cerca uscite volontarie
        # Esempio: ">>> [QUORUM LEAVE] t=150.5s | Drone 12 uscito con consenso globale."
        leave_match = re.search(r'>>> \[QUORUM LEAVE\] t=([\d\.]+)s\s*\|\s*Drone\s*(\d+)', line)
        if leave_match:
            t = float(leave_match.group(1))
            d_id = int(leave_match.group(2))
            leaves.append((t, d_id))

        # 2. Cerca espulsioni per anomalie
        # Esempio: "[WARNING] ESPULSIONE ESEGUITA!" seguito da ID e Tempo
        if "[WARNING] ESPULSIONE ESEGUITA!" in line:
            d_id = None
            t = None
            # Cerca le info nelle 5 righe successive al warning
            for j in range(i+1, min(i+6, len(lines))):
                id_match = re.search(r'Drone Sospetto ID\s*:\s*(\d+)', lines[j])
                if id_match:
                    d_id = int(id_match.group(1))
                
                t_match = re.search(r'Tempo Simulazione\s*:\s*([\d\.]+)s', lines[j])
                if t_match:
                    t = float(t_match.group(1))
            
            if d_id is not None and t is not None:
                evictions.append((t, d_id))

    # Creazione del grafico
    plt.figure(figsize=(10, 6))

    # Disegna le uscite volontarie (Verde, Cerchi)
    if leaves:
        times_l, ids_l = zip(*leaves)
        plt.scatter(times_l, ids_l, color='green', marker='o', s=150, alpha=0.8,
                    edgecolors='black', label='Uscita Volontaria (Geofence Leave)')

    # Disegna le espulsioni forzate (Rosso, Croci)
    if evictions:
        times_e, ids_e = zip(*evictions)
        plt.scatter(times_e, ids_e, color='red', marker='X', s=200, alpha=0.9,
                    edgecolors='black', label='Espulsione (Spoofing/Allarme EKF)')

    # Se non c'è nessun dato, avvisa
    if not leaves and not evictions:
        print("Nessun evento di uscita o espulsione trovato nel log.")

    # Formattazione
    plt.title('Timeline Uscite ed Espulsioni dallo Sciame (SwarmRaft)', fontsize=14, fontweight='bold')
    plt.xlabel('Tempo di Simulazione [s]', fontsize=12)
    plt.ylabel('ID del Drone', fontsize=12)
    
    # Assicurati che l'asse Y mostri solo numeri interi (gli ID dei droni)
    all_ids = [i for _, i in leaves] + [i for _, i in evictions]
    if all_ids:
        plt.yticks(range(0, max(all_ids) + 2))
    
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper left', fontsize=11, framealpha=0.9)
    plt.tight_layout()
    
    # Salva e mostra
    plt.savefig('esiti_droni.png', dpi=300)
    print("Grafico salvato con successo come 'esiti_droni.png'.")
    plt.show()

if __name__ == "__main__":
    main()