"""Riepilogo di una run: summary.json (simulazioni recenti) con ripiego su stdout.txt (simulazioni precedenti).

Il simulatore scrive in summary.json i valori che prima gli script cercavano nelle righe di stdout.txt, con la stessa
formattazione numerica: le due strade danno gli stessi numeri. Il testo a schermo puo' quindi cambiare liberamente.
"""
import json
import os


def load(path):
    """summary.json della cartella di `path` (un file della run, es. stdout.txt, o la cartella stessa); None se assente."""
    d = path if os.path.isdir(path) else os.path.dirname(path)
    f = os.path.join(d or ".", "summary.json")
    if not os.path.exists(f):
        return None
    with open(f) as fh:
        return json.load(fh)
