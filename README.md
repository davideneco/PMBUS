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

`pmbus_monitor.py` : un seul fichier, sans dépendance (Python 3 + `openssl` pour le certificat).

```
./pmbus_monitor.py --scan                                        # cherche les périphériques
./pmbus_monitor.py --autodetect                            # détecte les PSU 0x58-0x5F
./pmbus_monitor.py --addr 0x58 0x59                        # PSU en direct
./pmbus_monitor.py --mux 0x70 --channels 0-3 --addr 0x58   # via un PDB
./pmbus_monitor.py --config psus.example.json --web --auth admin:secret   # multi-bus / multi-mux
```

Page web : `https://<ip-de-la-BBB>:8443` (certificat auto-signé créé dans `~/.pmbus_monitor`,
l'avertissement du navigateur est normal ; `--cert/--key` pour le tien, `--http` pour du HTTP).
Onglets : vue d'ensemble, détails et codes d'erreur (tous les registres STATUS_*),
graphiques (historique 1 h en mémoire), journal des événements.
Sans `--web` : affichage terminal, `--json`, `--csv`.

Documentation PMBus : dossier `docs/`.
