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
1. **Accueil** : la liste des bus I2C (`/dev/i2c-*`) et des PSU déjà détectés.
2. **Page d'un bus** : toutes les adresses qui répondent, regroupées par segment (direct, canaux des mux),
   avec leur nature : alimentation PMBus (fabricant, modèle, série), mux, EEPROM, composants de la BBB,
   ou type probable déduit de l'adresse. Bouton « Rescanner ».
3. **Page d'un PSU** (clic sur une alimentation), en onglets :
   - *Mesures* : tensions, courants, puissances, températures, ventilateurs, rendement, identité, limites ;
   - *Graphiques* : historique d'1 h, export CSV ;
   - *Erreurs* : alarmes actives, registres STATUS_* bit par bit, journal du PSU ;
   - *Réglages* : ON / OFF / marges / effacer les défauts et limites modifiables (avec `--control`) ;
   - *Registres* : ~105 registres PMBus décodés.

Les boutons « ← Retour », le fil d'Ariane et le bouton « précédent » du navigateur fonctionnent partout.
Les bus autres que 0 sont balayés au démarrage ; le bus 0 (interne à la BBB) seulement quand on l'ouvre.
Deux emplacements renvoyant le même numéro de série sont signalés comme doublon.

Les écritures (`--control`, qui exige `--auth`) sont vérifiées : si le PSU les refuse (STATUS_CML), l'erreur s'affiche.
Sans `--https`, le mot de passe circule en clair : à réserver à un réseau local.

Documentation PMBus : dossier `docs/`.
