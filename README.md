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
python3 pmbus_monitor.py                        # page web http://<ip>:8080
python3 pmbus_monitor.py --auth admin:secret --control   # + modification des valeurs
python3 pmbus_monitor.py --mock                 # simulation, sans matériel
python3 pmbus_monitor.py --scan                 # rapport de détection dans le terminal
```

Le terminal n'affiche que l'adresse de la page (`--verbose` pour plus de détail).

### Navigation
**Menu principal** : PSU / PMBus · Scanner I2C · Journal · Système.

- **PSU / PMBus** → choix du bus (seuls les bus où une alimentation répond sont listés)
  → choix de l'adresse (seules les adresses PSU qui existent) → fiche du PSU en onglets :
  *Mesures*, *Graphiques* (+ export CSV), *Erreurs* (alarmes, STATUS_* bit par bit, journal),
  *Réglages* (ON/OFF, marges, effacer les défauts, limites modifiables avec `--control`), *Registres* (~105 décodés).
- **Scanner I2C** → bus ayant au moins un appareil → toutes les adresses présentes et leur nature
  (alimentation, mux, EEPROM, composants de la BBB, type probable selon l'adresse).
- **Journal** : tous les événements (alarmes, pertes de communication, actions).
- **Système** : adresses de la page, pilote I2C, options, état de chaque bus, relance de la détection.

Tous les bus sont balayés au démarrage (le bus 0 interne à la BBB en lecture seule).
Les boutons « ← Retour », le fil d'Ariane et le bouton « précédent » du navigateur fonctionnent partout.
Les bus autres que 0 sont balayés au démarrage ; le bus 0 (interne à la BBB) seulement quand on l'ouvre.
Deux emplacements renvoyant le même numéro de série sont signalés comme doublon.

Les écritures (`--control`, qui exige `--auth`) sont vérifiées : si le PSU les refuse (STATUS_CML), l'erreur s'affiche.
Sans `--https`, le mot de passe circule en clair : à réserver à un réseau local.

Documentation PMBus : dossier `docs/`.
