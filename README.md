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

`pmbus_monitor.py` : un seul fichier. Recommandé : `pip install smbus2` (sinon accès direct à `/dev/i2c-N`).

```
python3 pmbus_monitor.py                        # détecte tout (bus, mux, PSU) et ouvre la page web
python3 pmbus_monitor.py --scan                 # rapport de détection seulement
python3 pmbus_monitor.py -b 2 --addr 0x58 0x59  # forcer bus / adresses
python3 pmbus_monitor.py --mux 0x70 --channels 0-3 --addr 0x58   # via un PDB
python3 pmbus_monitor.py --config psus.example.json              # multi-bus / multi-mux
python3 pmbus_monitor.py --auth admin:secret --control           # + actions d'écriture
python3 pmbus_monitor.py --mock                 # simulation, sans matériel
```

Le terminal n'affiche que le titre et l'adresse de la page (`http://<ip>:8080`). `--verbose` ajoute le détail
(détection, pilote I2C, état des PSU) ; `--scan` donne le rapport de détection. `--https` active HTTPS (port 8443,
certificat auto-signé). Attention : sans HTTPS, le mot de passe `--auth` circule en clair sur le réseau.

Seuls les PSU qui ont répondu au moins une fois sont affichés. Deux emplacements qui renvoient le même numéro de série
sont considérés comme un seul PSU (doublon ignoré). Si aucun PSU n'est trouvé, la page reste ouverte, le script
retente toutes les 30 s et le bouton « Rescanner » force la détection.

Onglets : vue d'ensemble, détails et codes d'erreur (tous les registres STATUS_*), graphiques (+ export CSV),
registres (≈105 registres PMBus décodés), journal des événements.
`--control` (exige `--auth`) ajoute les boutons « Effacer les défauts », ON et OFF.

Documentation PMBus : dossier `docs/`.
