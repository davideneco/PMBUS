# PMBUS

## Utilisation

1. Dépose les fichiers à modifier dans le dossier `import/`.
2. Indique-moi ce que je dois changer dans ces fichiers.
3. Je modifie les fichiers, puis je commit et push sur la branche de travail.

## Structure

```
PMBUS/
├── README.md
└── import/     <- place tes fichiers ici
```

## Monitoring PMBus (BeagleBone Black)

`pmbus_monitor.py` : un seul fichier, sans dépendance.

```
./pmbus_monitor.py --scan                                  # cherche les périphériques
./pmbus_monitor.py --addr 0x58 0x59 --web                  # PSU en direct + page web
./pmbus_monitor.py --mux 0x70 --channels 0-3 --addr 0x58 --web   # via PDB
```

Page web : `http://<ip-de-la-BBB>:8080` (option `--port` pour changer).
Sans `--web`, affichage dans le terminal ; `--json` et `--csv` restent disponibles.
