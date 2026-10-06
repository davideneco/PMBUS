#!/usr/bin/env python3
"""
pmbus_monitor.py - Moniteur PMBus mono-fichier pour BeagleBone Black.

Sans dépendance externe (Python 3 standard + ioctl /dev/i2c-N ; `openssl` pour
générer le certificat HTTPS auto-signé, présent par défaut sur Debian/BBB).

Topologies gérées (combinables via --config) :
  - PSU directement sur un bus I2C de la BBB
  - PSU derrière un PDB avec un ou plusieurs mux I2C (PCA9546/9548...)
  - plusieurs bus I2C en même temps (un thread de lecture par bus)
  - PSU multi-rails (commande PAGE)

Exemples :
  ./pmbus_monitor.py --scan                                   # cherche les périphériques
  ./pmbus_monitor.py --autodetect                             # détecte les PSU sur tous les bus I2C
  ./pmbus_monitor.py --addr 0x58 0x59                         # 2 PSU en direct (web par défaut)
  ./pmbus_monitor.py --mux 0x70 --channels 0-3 --addr 0x58 --web   # 4 PSU derrière un PDB
  ./pmbus_monitor.py --config psus.json --web --auth admin:secret # topologie complète
  ./pmbus_monitor.py --addr 0x58 --once --json                # une mesure JSON ; --terminal pour le mode texte

Interface web : https://<ip-bbb>:8443 (certificat auto-signé, l'avertissement du
navigateur est normal ; --http pour du HTTP clair sur le port 8080).

Exemple de psus.json :
  {"interval": 2,
   "psus": [
     {"name": "PSU1", "bus": 2, "addr": "0x58", "mux": "0x70", "channel": 0},
     {"name": "PSU2", "bus": 2, "addr": "0x58", "mux": "0x70", "channel": 1},
     {"name": "Direct", "bus": 1, "addr": "0x5A"},
     {"name": "Multi-rail", "bus": 2, "addr": "0x40", "page": 1}]}

BBB : I2C2 = /dev/i2c-2 (P9_19 SCL / P9_20 SDA) est le bus par défaut.
      I2C1 = /dev/i2c-1 (P9_17 SCL / P9_18 SDA) nécessite un overlay.
"""

import argparse
import base64
import collections
import csv
import ctypes
import fcntl
import hmac
import json
import os
import signal
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --------------------------------------------------------------------------
# Accès I2C / SMBus via ioctl (équivalent minimal de smbus2)
# --------------------------------------------------------------------------
I2C_SLAVE_FORCE = 0x0706
I2C_SMBUS = 0x0720
I2C_SMBUS_READ = 1
I2C_SMBUS_WRITE = 0
I2C_SMBUS_BYTE = 1
I2C_SMBUS_BYTE_DATA = 2
I2C_SMBUS_WORD_DATA = 3
I2C_SMBUS_BLOCK_DATA = 5


class _SmbusData(ctypes.Union):
    _fields_ = [("byte", ctypes.c_uint8),
                ("word", ctypes.c_uint16),
                ("block", ctypes.c_uint8 * 34)]


class _SmbusIoctl(ctypes.Structure):
    _fields_ = [("read_write", ctypes.c_uint8),
                ("command", ctypes.c_uint8),
                ("size", ctypes.c_uint32),
                ("data", ctypes.POINTER(_SmbusData))]


class I2CBus:
    def __init__(self, bus):
        self.fd = os.open("/dev/i2c-%d" % bus, os.O_RDWR)
        self.addr = None

    def close(self):
        os.close(self.fd)

    def _xfer(self, addr, rw, cmd, size, data=None):
        if addr != self.addr:
            fcntl.ioctl(self.fd, I2C_SLAVE_FORCE, addr)
            self.addr = addr
        d = data or _SmbusData()
        fcntl.ioctl(self.fd, I2C_SMBUS, _SmbusIoctl(rw, cmd, size, ctypes.pointer(d)))
        return d

    def write_byte(self, addr, value):
        self._xfer(addr, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE)

    def read_byte(self, addr):
        return self._xfer(addr, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE).byte

    def read_byte_data(self, addr, cmd):
        return self._xfer(addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_BYTE_DATA).byte

    def write_byte_data(self, addr, cmd, value):
        d = _SmbusData()
        d.byte = value
        self._xfer(addr, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE_DATA, d)

    def read_word_data(self, addr, cmd):
        return self._xfer(addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_WORD_DATA).word

    def read_block_data(self, addr, cmd):
        d = self._xfer(addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_BLOCK_DATA)
        return bytes(d.block[1:1 + min(d.block[0], 32)])


def safe(fn, *a):
    """Appelle fn ; renvoie None si le périphérique NACK (commande non supportée)."""
    try:
        return fn(*a)
    except OSError:
        return None


# --------------------------------------------------------------------------
# Registres PMBus (PMBus Spec Part II, Rev 1.3.1)
# --------------------------------------------------------------------------
CMD_PAGE = 0x00
CMD_OPERATION = 0x01
CMD_CAPABILITY = 0x19
CMD_VOUT_MODE = 0x20
CMD_STATUS_WORD = 0x79
CMD_STATUS_CML = 0x7E
CMD_PMBUS_REVISION = 0x98

# (clé, commande, unité, décimales, libellé) - LINEAR11, sauf VOUT (LINEAR16)
TELEMETRY = [
    ("vin",   0x88, "V",   1, "Tension d'entrée (VIN)"),
    ("iin",   0x89, "A",   2, "Courant d'entrée (IIN)"),
    ("pin",   0x97, "W",   1, "Puissance d'entrée (PIN)"),
    ("vout",  0x8B, "V",   3, "Tension de sortie (VOUT)"),
    ("iout",  0x8C, "A",   2, "Courant de sortie (IOUT)"),
    ("pout",  0x96, "W",   1, "Puissance de sortie (POUT)"),
    ("temp1", 0x8D, "°C",  1, "Température 1"),
    ("temp2", 0x8E, "°C",  1, "Température 2"),
    ("temp3", 0x8F, "°C",  1, "Température 3"),
    ("fan1",  0x90, "rpm", 0, "Ventilateur 1"),
    ("fan2",  0x91, "rpm", 0, "Ventilateur 2"),
    ("fan3",  0x92, "rpm", 0, "Ventilateur 3"),
    ("fan4",  0x93, "rpm", 0, "Ventilateur 4"),
    ("duty",  0x94, "%",   1, "Rapport cyclique"),
    ("freq",  0x95, "kHz", 1, "Fréquence de commutation"),
]
TELEMETRY_KEYS = [t[0] for t in TELEMETRY]

# Registres de statut : (nom, commande, [(bit, nom, gravité, description)])
# gravité : fault = défaut, warn = avertissement, info = information
_MFR = [(b, "MFR_BIT%d" % b, "warn", "Bit défini par le fabricant (voir sa documentation)")
        for b in range(7, -1, -1)]
STATUS_REGS = [
    ("STATUS_VOUT", 0x7A, [
        (7, "VOUT_OV_FAULT", "fault", "Défaut de surtension de sortie"),
        (6, "VOUT_OV_WARNING", "warn", "Avertissement de surtension de sortie"),
        (5, "VOUT_UV_WARNING", "warn", "Avertissement de sous-tension de sortie"),
        (4, "VOUT_UV_FAULT", "fault", "Défaut de sous-tension de sortie"),
        (3, "VOUT_MAX_MIN_WARNING", "warn", "Consigne de tension hors limites VOUT_MAX / VOUT_MIN"),
        (2, "TON_MAX_FAULT", "fault", "Tension de sortie trop lente à atteindre sa valeur (TON_MAX)"),
        (1, "TOFF_MAX_WARNING", "warn", "Tension de sortie trop lente à retomber (TOFF_MAX)"),
        (0, "VOUT_TRACKING_ERROR", "warn", "Erreur de suivi de tension (défini par le fabricant)")]),
    ("STATUS_IOUT", 0x7B, [
        (7, "IOUT_OC_FAULT", "fault", "Défaut de surintensité de sortie"),
        (6, "IOUT_OC_LV_FAULT", "fault", "Surintensité de sortie avec tension basse"),
        (5, "IOUT_OC_WARNING", "warn", "Avertissement de surintensité de sortie"),
        (4, "IOUT_UC_FAULT", "fault", "Défaut de sous-intensité de sortie"),
        (3, "CURRENT_SHARE_FAULT", "fault", "Défaut de partage de courant entre PSU"),
        (2, "POWER_LIMITING_MODE", "warn", "Fonctionnement en limitation de puissance (POUT_MAX)"),
        (1, "POUT_OP_FAULT", "fault", "Défaut de surpuissance de sortie"),
        (0, "POUT_OP_WARNING", "warn", "Avertissement de surpuissance de sortie")]),
    ("STATUS_INPUT", 0x7C, [
        (7, "VIN_OV_FAULT", "fault", "Défaut de surtension d'entrée"),
        (6, "VIN_OV_WARNING", "warn", "Avertissement de surtension d'entrée"),
        (5, "VIN_UV_WARNING", "warn", "Avertissement de sous-tension d'entrée"),
        (4, "VIN_UV_FAULT", "fault", "Défaut de sous-tension d'entrée"),
        (3, "UNIT_OFF_LOW_VIN", "fault", "PSU éteint : tension d'entrée insuffisante"),
        (2, "IIN_OC_FAULT", "fault", "Défaut de surintensité d'entrée"),
        (1, "IIN_OC_WARNING", "warn", "Avertissement de surintensité d'entrée"),
        (0, "PIN_OP_WARNING", "warn", "Avertissement de surpuissance d'entrée")]),
    ("STATUS_TEMPERATURE", 0x7D, [
        (7, "OT_FAULT", "fault", "Défaut de surtempérature"),
        (6, "OT_WARNING", "warn", "Avertissement de surtempérature"),
        (5, "UT_WARNING", "warn", "Avertissement de sous-température"),
        (4, "UT_FAULT", "fault", "Défaut de sous-température")]),
    ("STATUS_CML", 0x7E, [
        (7, "INVALID_COMMAND", "fault", "Commande invalide ou non supportée reçue"),
        (6, "INVALID_DATA", "fault", "Donnée invalide ou non supportée reçue"),
        (5, "PEC_FAILED", "fault", "Échec du contrôle d'erreur de paquet (PEC)"),
        (4, "MEMORY_FAULT", "fault", "Défaut mémoire détecté"),
        (3, "PROCESSOR_FAULT", "fault", "Défaut du processeur interne"),
        (1, "COMM_FAULT", "fault", "Autre défaut de communication"),
        (0, "OTHER_MEM_LOGIC_FAULT", "fault", "Autre défaut mémoire ou logique")]),
    ("STATUS_OTHER", 0x7F, [
        (5, "INPUT_A_FUSE_FAULT", "fault", "Fusible / disjoncteur de l'entrée A en défaut"),
        (4, "INPUT_B_FUSE_FAULT", "fault", "Fusible / disjoncteur de l'entrée B en défaut"),
        (3, "INPUT_A_ORING_FAULT", "fault", "Dispositif OR-ing de l'entrée A en défaut"),
        (2, "INPUT_B_ORING_FAULT", "fault", "Dispositif OR-ing de l'entrée B en défaut"),
        (1, "OUTPUT_ORING_FAULT", "fault", "Dispositif OR-ing de sortie en défaut"),
        (0, "FIRST_SMBALERT", "info", "Premier à avoir déclenché SMBALERT#")]),
    ("STATUS_MFR_SPECIFIC", 0x80, _MFR),
    ("STATUS_FANS_1_2", 0x81, [
        (7, "FAN1_FAULT", "fault", "Défaut du ventilateur 1"),
        (6, "FAN2_FAULT", "fault", "Défaut du ventilateur 2"),
        (5, "FAN1_WARNING", "warn", "Avertissement du ventilateur 1"),
        (4, "FAN2_WARNING", "warn", "Avertissement du ventilateur 2"),
        (3, "FAN1_OVERRIDDEN", "info", "Vitesse du ventilateur 1 forcée par un contrôleur externe"),
        (2, "FAN2_OVERRIDDEN", "info", "Vitesse du ventilateur 2 forcée par un contrôleur externe"),
        (1, "AIRFLOW_FAULT", "fault", "Défaut de flux d'air"),
        (0, "AIRFLOW_WARNING", "warn", "Avertissement de flux d'air")]),
    ("STATUS_FANS_3_4", 0x82, [
        (7, "FAN3_FAULT", "fault", "Défaut du ventilateur 3"),
        (6, "FAN4_FAULT", "fault", "Défaut du ventilateur 4"),
        (5, "FAN3_WARNING", "warn", "Avertissement du ventilateur 3"),
        (4, "FAN4_WARNING", "warn", "Avertissement du ventilateur 4"),
        (3, "FAN3_OVERRIDDEN", "info", "Vitesse du ventilateur 3 forcée par un contrôleur externe"),
        (2, "FAN4_OVERRIDDEN", "info", "Vitesse du ventilateur 4 forcée par un contrôleur externe")]),
]

# STATUS_WORD : résumé des défauts (octet bas = STATUS_BYTE)
STATUS_WORD_BITS = [
    (15, "VOUT", "warn", "Défaut ou avertissement de tension de sortie"),
    (14, "IOUT/POUT", "warn", "Défaut ou avertissement de courant / puissance de sortie"),
    (13, "INPUT", "warn", "Défaut ou avertissement d'entrée (tension, courant, puissance)"),
    (12, "MFR_SPECIFIC", "warn", "Défaut ou avertissement spécifique au fabricant"),
    (11, "PG_STATUS#", "fault", "Signal POWER_GOOD à l'état négatif (sortie non valide)"),
    (10, "FANS", "warn", "Défaut ou avertissement de ventilateur / flux d'air"),
    (9, "OTHER", "warn", "Un bit de STATUS_OTHER est actif"),
    (8, "UNKNOWN", "warn", "Défaut de type inconnu"),
    (7, "BUSY", "warn", "Le PSU était occupé et n'a pas pu répondre"),
    (6, "OFF", "info", "Le PSU ne fournit pas de puissance en sortie"),
    (5, "VOUT_OV_FAULT", "fault", "Défaut de surtension de sortie"),
    (4, "IOUT_OC_FAULT", "fault", "Défaut de surintensité de sortie"),
    (3, "VIN_UV_FAULT", "fault", "Défaut de sous-tension d'entrée"),
    (2, "TEMPERATURE", "fault", "Défaut ou avertissement de température"),
    (1, "CML", "fault", "Défaut de communication, mémoire ou logique"),
    (0, "NONE_OF_THE_ABOVE", "warn", "Défaut ou avertissement non listé dans les autres bits"),
]

# Limites programmées : (groupe, libellé, commande, unité, format 'l11'|'l16')
LIMITS = [
    ("Sortie", "Consigne VOUT (VOUT_COMMAND)", 0x21, "V", "l16"),
    ("Tension d'entrée", "VIN_OV_FAULT_LIMIT", 0x55, "V", "l11"),
    ("Tension d'entrée", "VIN_OV_WARN_LIMIT", 0x57, "V", "l11"),
    ("Tension d'entrée", "VIN_UV_WARN_LIMIT", 0x58, "V", "l11"),
    ("Tension d'entrée", "VIN_UV_FAULT_LIMIT", 0x59, "V", "l11"),
    ("Courant d'entrée", "IIN_OC_FAULT_LIMIT", 0x5B, "A", "l11"),
    ("Courant d'entrée", "IIN_OC_WARN_LIMIT", 0x5D, "A", "l11"),
    ("Puissance", "PIN_OP_WARN_LIMIT", 0x6B, "W", "l11"),
    ("Puissance", "POUT_OP_FAULT_LIMIT", 0x68, "W", "l11"),
    ("Puissance", "POUT_OP_WARN_LIMIT", 0x6A, "W", "l11"),
    ("Tension de sortie", "VOUT_OV_FAULT_LIMIT", 0x40, "V", "l16"),
    ("Tension de sortie", "VOUT_OV_WARN_LIMIT", 0x42, "V", "l16"),
    ("Tension de sortie", "VOUT_UV_WARN_LIMIT", 0x43, "V", "l16"),
    ("Tension de sortie", "VOUT_UV_FAULT_LIMIT", 0x44, "V", "l16"),
    ("Courant de sortie", "IOUT_OC_FAULT_LIMIT", 0x46, "A", "l11"),
    ("Courant de sortie", "IOUT_OC_WARN_LIMIT", 0x4A, "A", "l11"),
    ("Température", "OT_FAULT_LIMIT", 0x4F, "°C", "l11"),
    ("Température", "OT_WARN_LIMIT", 0x51, "°C", "l11"),
    ("Température", "UT_WARN_LIMIT", 0x52, "°C", "l11"),
    ("Température", "UT_FAULT_LIMIT", 0x53, "°C", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_VIN_MIN", 0xA0, "V", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_VIN_MAX", 0xA1, "V", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_IIN_MAX", 0xA2, "A", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_PIN_MAX", 0xA3, "W", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_VOUT_MIN", 0xA4, "V", "l16"),
    ("Valeurs nominales (fabricant)", "MFR_VOUT_MAX", 0xA5, "V", "l16"),
    ("Valeurs nominales (fabricant)", "MFR_IOUT_MAX", 0xA6, "A", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_POUT_MAX", 0xA7, "W", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_TAMBIENT_MAX", 0xA8, "°C", "l11"),
    ("Valeurs nominales (fabricant)", "MFR_TAMBIENT_MIN", 0xA9, "°C", "l11"),
]

INFO_TEXT = [("Fabricant (MFR_ID)", 0x99), ("Modèle (MFR_MODEL)", 0x9A),
             ("Révision (MFR_REVISION)", 0x9B), ("Lieu de fabrication", 0x9C),
             ("Date de fabrication", 0x9D), ("N° de série", 0x9E)]


def decode_linear11(raw):
    exp = raw >> 11
    if exp > 15:
        exp -= 32
    man = raw & 0x7FF
    if man > 1023:
        man -= 2048
    return man * (2.0 ** exp)


def decode_linear16(raw, vout_mode):
    exp = vout_mode & 0x1F
    if exp > 15:
        exp -= 32
    return raw * (2.0 ** exp)


def word_flags(sw):
    return [{"bit": b, "name": n, "sev": s, "desc": d}
            for b, n, s, d in STATUS_WORD_BITS if sw & (1 << b)]


def status_defs():
    defs = [{"reg": "STATUS_WORD", "bits": [list(x) for x in STATUS_WORD_BITS]}]
    defs += [{"reg": n, "bits": [list(x) for x in bits]} for n, _c, bits in STATUS_REGS]
    return defs


# --------------------------------------------------------------------------
# PSU : description, lecture, décodage
# --------------------------------------------------------------------------
class Psu:
    def __init__(self, name, bus, addr, mux=None, channel=None, page=None):
        self.name, self.bus, self.addr = name, bus, addr
        self.mux, self.channel, self.page = mux, channel, page
        loc = "bus %d" % bus
        if mux is not None:
            loc += " · mux 0x%02X/ch%d" % (mux, channel)
        loc += " · 0x%02X" % addr
        if page is not None:
            loc += " · page %d" % page
        self.loc = loc
        self.reset()

    def reset(self):
        """Force une relecture complète (identité, limites, commandes supportées)."""
        self.supported = None
        self.info = {}
        self.limits = []
        self.vout_mode = None
        self.ignore_cml = 0


def read_text(bus, addr, cmd):
    raw = safe(bus.read_block_data, addr, cmd)
    return raw.decode("ascii", "replace").strip("\x00 ") if raw else ""


def first_contact(bus, psu):
    """Identifie le PSU et détermine les commandes qu'il supporte (une seule fois)."""
    a = psu.addr
    cml_before = safe(bus.read_byte_data, a, CMD_STATUS_CML)
    psu.vout_mode = safe(bus.read_byte_data, a, CMD_VOUT_MODE)
    lin16 = psu.vout_mode is not None and (psu.vout_mode >> 5) == 0

    supported = set()
    for key, cmd, *_ in TELEMETRY:
        if safe(bus.read_word_data, a, cmd) is not None:
            supported.add(key)
    for name, cmd, _bits in STATUS_REGS:
        if safe(bus.read_byte_data, a, cmd) is not None:
            supported.add(name)
    if safe(bus.read_byte_data, a, CMD_OPERATION) is not None:
        supported.add("OPERATION")

    info = {label: read_text(bus, a, cmd) for label, cmd in INFO_TEXT}
    rev = safe(bus.read_byte_data, a, CMD_PMBUS_REVISION)
    if rev is not None:
        info["Révision PMBus"] = "Partie I 1.%d / Partie II 1.%d" % (rev >> 4, rev & 0xF)
    cap = safe(bus.read_byte_data, a, CMD_CAPABILITY)
    if cap is not None:
        info["Capacités"] = "PEC %s · bus max %s · SMBALERT# %s" % (
            "oui" if cap & 0x80 else "non",
            {0: "100 kHz", 1: "400 kHz"}.get((cap >> 5) & 3, "?"),
            "oui" if cap & 0x10 else "non")
    psu.info = {k: v for k, v in info.items() if v}

    limits = []
    for group, label, cmd, unit, fmt in LIMITS:
        raw = safe(bus.read_word_data, a, cmd)
        if raw is None:
            continue
        if fmt == "l16":
            if not lin16:
                continue
            val = decode_linear16(raw, psu.vout_mode)
        else:
            val = decode_linear11(raw)
        limits.append({"group": group, "label": label, "code": cmd, "value": val, "unit": unit})
    psu.limits = limits

    # Les commandes non supportées lèvent le bit 7/6 de STATUS_CML : on masque
    # ce que *nous* avons provoqué pour ne pas afficher de fausse alarme.
    cml_after = safe(bus.read_byte_data, a, CMD_STATUS_CML)
    if cml_before is not None and cml_after is not None:
        psu.ignore_cml = (cml_after & ~cml_before) & 0xC0
    psu.supported = supported


def read_psu(bus, psu):
    """Une mesure complète. Renvoie toujours un dict avec 'name', 'loc', 'summary'."""
    a = psu.addr
    res = {"name": psu.name, "loc": psu.loc, "t": time.time()}
    try:
        if psu.page is not None:
            bus.write_byte_data(a, CMD_PAGE, psu.page)
        sw = bus.read_word_data(a, CMD_STATUS_WORD)
    except OSError as e:
        psu.reset()
        res.update(summary="offline", online=False,
                   error="pas de réponse (%s)" % (e.strerror or e))
        return res

    if psu.supported is None:
        first_contact(bus, psu)
        if psu.page is not None:   # first_contact ne change pas de page, mais par sécurité
            safe(bus.write_byte_data, a, CMD_PAGE, psu.page)
    lin16 = psu.vout_mode is not None and (psu.vout_mode >> 5) == 0

    values = {}
    for key, cmd, *_ in TELEMETRY:
        if key not in psu.supported:
            continue
        raw = safe(bus.read_word_data, a, cmd)
        if raw is None:
            continue
        values[key] = decode_linear16(raw, psu.vout_mode) if key == "vout" and lin16 \
            else decode_linear11(raw)

    regs = {}
    for name, cmd, _bits in STATUS_REGS:
        if name in psu.supported:
            v = safe(bus.read_byte_data, a, cmd)
            if v is not None:
                regs[name] = v
    if "STATUS_CML" in regs and psu.ignore_cml:
        regs["STATUS_CML"] &= ~psu.ignore_cml & 0xFF
        if regs["STATUS_CML"] == 0:
            sw &= ~0x0002

    alarms = []
    for name, _cmd, bits in STATUS_REGS:
        v = regs.get(name)
        if not v:
            continue
        for bit, bname, sev, desc in bits:
            if v & (1 << bit):
                alarms.append({"reg": name, "bit": bit, "name": bname, "sev": sev, "desc": desc})

    flags = word_flags(sw)
    op = safe(bus.read_byte_data, a, CMD_OPERATION) if "OPERATION" in psu.supported else None

    sevs = {x["sev"] for x in alarms}
    if not alarms:   # pas de registre détaillé : on se rabat sur le STATUS_WORD
        sevs = {f["sev"] for f in flags if f["name"] != "OFF"}
    if "fault" in sevs:
        summary = "fault"
    elif "warn" in sevs:
        summary = "warn"
    elif sw & 0x40 or (op is not None and not op & 0x80):
        summary = "off"
    else:
        summary = "ok"

    res.update(online=True, summary=summary, values=values, status_word=sw, word_flags=flags,
               regs=regs, alarms=alarms, operation=op, info=psu.info, limits=psu.limits)
    return res


# --------------------------------------------------------------------------
# Bus I2C + mux de PDB
# --------------------------------------------------------------------------
class BusWorker:
    """Lit séquentiellement tous les PSU d'un bus, en pilotant les mux du PDB."""

    def __init__(self, bus_num, psus):
        self.num = bus_num
        self.psus = psus
        self.bus = I2CBus(bus_num)
        self.cur = None   # (mux, canal) actuellement sélectionné

    def _select(self, psu):
        want = (psu.mux, psu.channel) if psu.mux is not None else None
        if want == self.cur:
            return
        try:
            if self.cur and (want is None or want[0] != self.cur[0]):
                self.bus.write_byte(self.cur[0], 0)
            if want:
                self.bus.write_byte(want[0], 1 << want[1])
        except OSError:
            self.cur = None
            raise
        self.cur = want

    def deselect(self):
        if self.cur:
            safe(self.bus.write_byte, self.cur[0], 0)
            self.cur = None

    def poll(self):
        out = []
        for psu in self.psus:
            try:
                self._select(psu)
            except OSError as e:
                psu.reset()
                out.append({"name": psu.name, "loc": psu.loc, "t": time.time(), "online": False,
                            "summary": "offline", "error": "mux 0x%02X : %s" % (psu.mux, e.strerror or e)})
                continue
            out.append(read_psu(self.bus, psu))
        self.deselect()
        return out

    def close(self):
        self.deselect()
        self.bus.close()


def scan_bus(bus):
    """Adresses qui répondent (lecture d'octet, comme i2cdetect ; repli sur STATUS_WORD)."""
    found = []
    for a in range(0x03, 0x78):
        if safe(bus.read_byte, a) is not None or safe(bus.read_word_data, a, CMD_STATUS_WORD) is not None:
            found.append(a)
    return found


def hexl(addrs):
    return " ".join("0x%02X" % a for a in addrs) or "(rien)"


def looks_like_pmbus(bus, addr):
    """Un périphérique est pris pour un PSU PMBus s'il lit STATUS_WORD et confirme par d'autres commandes."""
    if safe(bus.read_word_data, addr, CMD_STATUS_WORD) is None:
        return False
    score = 0
    rev = safe(bus.read_byte_data, addr, CMD_PMBUS_REVISION)
    if rev is not None and (rev >> 4) <= 4 and (rev & 0xF) <= 4:
        score += 1
    for cmd in (CMD_VOUT_MODE, CMD_CAPABILITY):
        if safe(bus.read_byte_data, addr, cmd) is not None:
            score += 1
    for cmd in (0x88, 0x8B, 0x8C, 0x8D):   # READ_VIN / VOUT / IOUT / TEMPERATURE_1
        if safe(bus.read_word_data, addr, cmd) is not None:
            score += 1
    return score >= 2


def is_mux(bus, addr):
    """PCA954x : l'octet de contrôle écrit se relit à l'identique."""
    try:
        bus.write_byte(addr, 0x01)
        ok = bus.read_byte(addr) & 0x0F == 0x01
        bus.write_byte(addr, 0x00)
        return ok
    except OSError:
        return False


def discover(args, report=True):
    """Balaye tous les bus I2C (ou celui demandé avec -b), les mux du PDB et toutes les adresses.
    Renvoie la liste des PSU PMBus trouvés."""
    say = print if report else (lambda *a, **k: None)
    nums = [args.bus] if args.bus_explicit else sorted(
        int(f.split("-")[1]) for f in os.listdir("/dev") if f.startswith("i2c-") and f.split("-")[1].isdigit())
    if not nums:
        say("Aucun /dev/i2c-* : active le bus I2C (overlay / config-pin) et vérifie les droits.")
        return []
    say("Bus I2C à balayer : %s" % ", ".join(map(str, nums)))
    psus = []
    for num in nums:
        try:
            bus = I2CBus(num)
        except OSError as e:
            say("Bus %d : ouverture impossible (%s)" % (num, e))
            continue
        try:
            # 1) mux : on les repère et on les remet à zéro avant tout balayage direct
            cands = [args.mux] if args.mux is not None else [a for a in range(0x70, 0x78)
                                                              if safe(bus.read_byte, a) is not None]
            muxes = [m for m in cands if is_mux(bus, m)]
            direct = scan_bus(bus)
            say("Bus %d : %s" % (num, hexl(direct)))
            if muxes:
                say("  mux I2C détecté(s) : %s" % hexl(muxes))
            skip = set(muxes) | set(range(0x50, 0x58))   # mux et EEPROM FRU : pas des PSU
            for a in direct:
                if (args.addr is None or a in args.addr) and a not in skip:
                    if looks_like_pmbus(bus, a):
                        psus.append(Psu("PSU%d" % (len(psus) + 1), num, a))
                        say("  -> PSU PMBus en 0x%02X (direct)" % a)
                    else:
                        say("  0x%02X répond mais n'est pas reconnu comme PMBus" % a)
            # 2) derrière chaque mux, canal par canal
            for m in muxes:
                chans = args.channels if args.mux is not None else range(8)
                for ch in chans:
                    try:
                        bus.write_byte(m, 1 << ch)
                    except OSError:
                        continue
                    found = [a for a in scan_bus(bus) if a != m]
                    if found:
                        say("  mux 0x%02X canal %d : %s" % (m, ch, hexl(found)))
                    for a in found:
                        if (args.addr is None or a in args.addr) and a not in skip and a not in muxes:
                            if looks_like_pmbus(bus, a):
                                psus.append(Psu("PSU%d" % (len(psus) + 1), num, a, m, ch))
                                say("  -> PSU PMBus en 0x%02X (mux 0x%02X canal %d)" % (a, m, ch))
                            else:
                                say("  0x%02X (canal %d) répond mais n'est pas reconnu comme PMBus" % (a, ch))
                safe(bus.write_byte, m, 0)
        finally:
            bus.close()
    say("Détection terminée : %d PSU PMBus trouvé(s)." % len(psus))
    return psus


# --------------------------------------------------------------------------
# Construction de la liste des PSU
# --------------------------------------------------------------------------
def int_auto(s):
    return int(s, 0) if isinstance(s, str) else int(s)


def parse_channels(values):
    """Accepte '0 1 2', '0,1,2' ou '0-3'."""
    chans = []
    for v in values:
        for part in v.split(","):
            if "-" in part:
                a, b = part.split("-")
                chans.extend(range(int(a), int(b) + 1))
            elif part:
                chans.append(int(part))
    return chans


def psus_from_config(path, default_bus):
    with open(path) as f:
        cfg = json.load(f)
    psus = []
    for i, e in enumerate(cfg["psus"], 1):
        mux = e.get("mux")
        psus.append(Psu(e.get("name", "PSU%d" % i), int(e.get("bus", default_bus)), int_auto(e["addr"]),
                        int_auto(mux) if mux is not None else None,
                        int(e["channel"]) if mux is not None else None,
                        int(e["page"]) if e.get("page") is not None else None))
    return psus, cfg.get("interval")


def psus_from_args(args):
    addrs = args.addr or [0x58]
    pages = args.pages or [None]
    psus = []
    chans = [None] if args.mux is None else args.channels
    for ch in chans:
        for a in addrs:
            for pg in pages:
                psus.append(Psu("PSU%d" % (len(psus) + 1), args.bus, a, args.mux, ch, pg))
    return psus


def unique_names(psus):
    seen = {}
    for p in psus:
        n = seen.get(p.name, 0) + 1
        seen[p.name] = n
        if n > 1:
            p.name = "%s (%d)" % (p.name, n)


# --------------------------------------------------------------------------
# État partagé : dernières mesures, historique, journal d'événements
# --------------------------------------------------------------------------
class State:
    def __init__(self, psus, interval, history_s):
        self.lock = threading.Lock()
        self.order = [p.name for p in psus]
        self.interval = interval
        self.latest = {}
        self.hist = {p.name: {} for p in psus}
        self.maxlen = max(120, int(history_s / max(interval, 0.2)))
        self.events = collections.deque(maxlen=1000)
        self.seq = 0

    def _event(self, t, psu, sev, msg):
        self.seq += 1
        self.events.append({"id": self.seq, "t": t, "psu": psu, "sev": sev, "msg": msg})

    def update(self, results):
        with self.lock:
            for r in results:
                name, t = r["name"], r["t"]
                prev = self.latest.get(name)
                was_online = prev.get("online") if prev else None
                if r["online"] is False and was_online is not False:
                    self._event(t, name, "fault", "Hors ligne : " + r.get("error", ""))
                elif r["online"] and was_online is False:
                    self._event(t, name, "info", "De nouveau en ligne")
                if r["online"]:
                    old = {(a["reg"], a["bit"]) for a in prev.get("alarms", [])} if prev and was_online else set()
                    for a in r["alarms"]:
                        if (a["reg"], a["bit"]) not in old:
                            self._event(t, name, a["sev"], "%s : %s (%s)" % (a["name"], a["desc"], a["reg"]))
                    new = {(a["reg"], a["bit"]) for a in r["alarms"]}
                    if prev and was_online:
                        for a in prev["alarms"]:
                            if (a["reg"], a["bit"]) not in new:
                                self._event(t, name, "info", "Effacé : %s (%s)" % (a["name"], a["reg"]))
                    for key, v in r["values"].items():
                        dq = self.hist[name].setdefault(key, collections.deque(maxlen=self.maxlen))
                        dq.append((t, v))
                self.latest[name] = r

    def snapshot(self):
        with self.lock:
            psus = [self.latest.get(n) or {"name": n, "loc": "", "summary": "pending", "online": None}
                    for n in self.order]
        return {"time": time.time(), "interval": self.interval, "psus": psus}

    def history(self, since):
        with self.lock:
            return {n: {k: [[round(t, 2), v] for t, v in dq if t > since] for k, dq in series.items()}
                    for n, series in self.hist.items()}

    def events_since(self, since):
        with self.lock:
            return [e for e in self.events if e["id"] > since]


METRICS_META = {k: {"label": lab, "unit": u, "dec": d} for k, _c, u, d, lab in TELEMETRY}

# --------------------------------------------------------------------------
# Page web (tout en un : HTML + CSS + JS, aucune ressource externe)
# --------------------------------------------------------------------------
HTML_PAGE = r"""<!doctype html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PMBus Monitor</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1c1f24;--mut:#6b7280;--bd:#e5e7eb;--ok:#16a34a;--warn:#d97706;--fault:#dc2626;--off:#6b7280;--acc:#2563eb}
@media(prefers-color-scheme:dark){:root{--bg:#111418;--card:#1a1e24;--fg:#e8eaed;--mut:#9aa3af;--bd:#2a3039;--acc:#60a5fa}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif}
header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;padding:12px 16px}
h1{font-size:18px;margin:0}#meta{color:var(--mut);font-size:13px;margin-left:auto}
.pill{font-size:12px;padding:3px 10px;border-radius:99px;color:#fff;background:var(--ok);white-space:nowrap}
.pill.warn{background:var(--warn)}.pill.fault{background:var(--fault)}.pill.off,.pill.offline,.pill.pending{background:var(--off)}
nav{display:flex;gap:4px;padding:0 16px;border-bottom:1px solid var(--bd);overflow-x:auto}
nav button{background:none;border:0;border-bottom:3px solid transparent;color:var(--mut);padding:10px 14px;font:inherit;cursor:pointer;white-space:nowrap}
nav button.on{color:var(--fg);border-color:var(--acc);font-weight:600}
main{padding:16px}section[hidden]{display:none}
#grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(270px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--bd);border-left:4px solid var(--ok);border-radius:10px;padding:14px}
.card.warn{border-left-color:var(--warn)}.card.fault{border-left-color:var(--fault)}
.card.off,.card.offline,.card.pending{border-left-color:var(--off)}
.card.click{cursor:pointer}.card.click:hover{border-color:var(--acc)}
.top{display:flex;justify-content:space-between;align-items:center;gap:8px}
.name{font-weight:600}.loc{color:var(--mut);font-size:12px;margin-bottom:8px}
dl{display:grid;grid-template-columns:1fr auto;gap:3px 12px;margin:8px 0 0}
dt{color:var(--mut)}dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;font-weight:500}
.chips{display:flex;flex-wrap:wrap;gap:4px;margin-top:8px}
.chip{font-size:11px;padding:2px 7px;border-radius:6px;background:var(--warn);color:#fff}
.chip.fault{background:var(--fault)}.chip.info{background:var(--off)}
.err{color:var(--fault);font-size:13px}
h2{font-size:16px;margin:20px 0 8px}h3{font-size:13px;margin:14px 0 6px;color:var(--mut);text-transform:uppercase;letter-spacing:.04em}
.panel{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:14px;margin-bottom:14px}
table{border-collapse:collapse;width:100%;font-size:14px}
td,th{padding:5px 8px;border-bottom:1px solid var(--bd);text-align:left;vertical-align:top}
th{color:var(--mut);font-weight:500;font-size:12px}td.n{text-align:right;font-variant-numeric:tabular-nums}
.reg{margin:10px 0}.reg b{font-size:13px}.reg code{color:var(--mut);margin-left:6px}
.bits{display:flex;flex-wrap:wrap;gap:4px;margin-top:5px}
.bit{font-size:11px;padding:2px 7px;border-radius:6px;border:1px solid var(--bd);color:var(--mut)}
.bit.set{color:#fff;border-color:transparent;background:var(--warn)}.bit.set.fault{background:var(--fault)}.bit.set.info{background:var(--off)}
.ok-line{color:var(--ok)}
.bar{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:12px}
select,label{font:inherit;color:var(--fg)}select{background:var(--card);border:1px solid var(--bd);border-radius:6px;padding:5px 8px}
.leg{display:inline-flex;align-items:center;gap:5px;cursor:pointer;font-size:13px}
.leg i{width:12px;height:12px;border-radius:3px;display:inline-block}
#charts{display:grid;grid-template-columns:repeat(auto-fill,minmax(420px,1fr));gap:12px}
@media(max-width:480px){#charts{grid-template-columns:1fr}}
.ch{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:10px}
.ch h3{margin:0 0 4px}.ch canvas{width:100%;height:190px;display:block}
.empty{color:var(--mut);padding:20px;text-align:center}
</style></head><body>
<header><h1>PMBus Monitor</h1><span id="summary"></span><span id="meta">chargement…</span></header>
<nav id="tabs">
 <button data-tab="overview" class="on">Vue d'ensemble</button>
 <button data-tab="details">Détails &amp; codes d'erreur</button>
 <button data-tab="graphs">Graphiques</button>
 <button data-tab="events">Journal</button>
</nav>
<main>
 <section id="tab-overview"><div id="grid"></div></section>
 <section id="tab-details" hidden>
  <div class="bar"><label>PSU : <select id="sel"></select></label></div><div id="detail"></div></section>
 <section id="tab-graphs" hidden>
  <div class="bar"><label>Période : <select id="range">
    <option value="300">5 min</option><option value="900">15 min</option>
    <option value="1800">30 min</option><option value="3600" selected>1 h</option></select></label>
   <span id="legend"></span></div>
  <div id="charts"></div></section>
 <section id="tab-events" hidden><div class="panel"><table id="events"></table></div></section>
</main>
<script>
const SEV={ok:"OK",warn:"Avertissement",fault:"Défaut",off:"Éteint",offline:"Hors ligne",pending:"…",info:"Info"};
const COLORS=["#2563eb","#dc2626","#16a34a","#d97706","#7c3aed","#0891b2","#db2777","#65a30d","#475569","#ea580c"];
const $=id=>document.getElementById(id);
let data=null,hist={},histSince=0,histLoaded=false,events=[],evId=0,tab="overview",sel="all",hidden=new Set(),openSec=new Set(),chartKey="";
const esc=s=>String(s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const fmt=(v,d)=>v==null?"–":v.toFixed(d);
const hex=(v,n)=>"0x"+v.toString(16).toUpperCase().padStart(n,"0");
const names=()=>data?data.psus.map(p=>p.name):[];
const color=n=>COLORS[names().indexOf(n)%COLORS.length];

document.querySelectorAll("#tabs button").forEach(b=>b.onclick=()=>{
  tab=b.dataset.tab;
  document.querySelectorAll("#tabs button").forEach(x=>x.classList.toggle("on",x===b));
  document.querySelectorAll("main>section").forEach(s=>s.hidden=s.id!=="tab-"+tab);
  if(tab==="graphs")loadHist().then(drawCharts);
  render();
});
$("sel").onchange=e=>{sel=e.target.value;renderDetails()};
$("range").onchange=()=>drawCharts();

function worst(list){for(const s of ["fault","warn","off","offline"])if(list.some(p=>p.summary===s))return s;return "ok"}

function renderSummary(){
  const p=data.psus,bad=p.filter(x=>x.summary==="fault").length,w=p.filter(x=>x.summary==="warn").length,
  off=p.filter(x=>x.summary==="offline").length;
  const parts=[`${p.length} PSU`];if(bad)parts.push(`${bad} en défaut`);if(w)parts.push(`${w} avert.`);if(off)parts.push(`${off} hors ligne`);
  $("summary").innerHTML=`<span class="pill ${worst(p)}">${parts.join(" · ")}</span>`;
  $("meta").textContent="maj "+new Date(data.time*1000).toLocaleTimeString()+" · toutes les "+data.interval+" s";
}

function eff(v){return v.pin>5&&v.pout!=null?v.pout/v.pin*100:null}
function renderOverview(){
  if(!data.psus.length){$("grid").innerHTML='<div class="empty">Aucun PSU détecté. Regarde le terminal SSH (rapport de détection), vérifie le câblage et relance, ou utilise <code>--scan</code>, <code>-b</code>, <code>--addr</code>, <code>--mux</code>.</div>';return}
  $("grid").innerHTML=data.psus.map(p=>{
    if(p.summary==="pending")return `<div class="card pending"><div class="name">${esc(p.name)}</div><div class="loc">lecture en cours…</div></div>`;
    if(!p.online)return `<div class="card offline click" data-p="${esc(p.name)}"><div class="top"><span class="name">${esc(p.name)}</span><span class="pill offline">Hors ligne</span></div><div class="loc">${esc(p.loc)}</div><div class="err">${esc(p.error||"")}</div></div>`;
    const v=p.values,M=data.metrics;let rows="";
    for(const k of ["vin","iin","pin","vout","iout","pout"])if(k in v)rows+=`<dt>${k.toUpperCase()}</dt><dd>${fmt(v[k],M[k].dec)} ${M[k].unit}</dd>`;
    const temps=["temp1","temp2","temp3"].filter(k=>k in v).map(k=>v[k]);
    if(temps.length)rows+=`<dt>Température max</dt><dd>${Math.max(...temps).toFixed(1)} °C</dd>`;
    const fans=["fan1","fan2","fan3","fan4"].filter(k=>k in v);
    if(fans.length)rows+=`<dt>Ventilateurs</dt><dd>${fans.map(k=>v[k].toFixed(0)).join(" / ")} rpm</dd>`;
    const e=eff(v);if(e!=null)rows+=`<dt>Rendement</dt><dd>${e.toFixed(1)} %</dd>`;
    const chips=p.alarms.slice(0,6).map(a=>`<span class="chip ${a.sev}" title="${esc(a.desc)}">${esc(a.name)}</span>`).join("")+
      (p.alarms.length>6?`<span class="chip info">+${p.alarms.length-6}</span>`:"");
    return `<div class="card ${p.summary} click" data-p="${esc(p.name)}"><div class="top"><span class="name">${esc(p.name)}</span><span class="pill ${p.summary}">${SEV[p.summary]}</span></div><div class="loc">${esc(p.loc)}</div><dl>${rows}</dl><div class="chips">${chips}</div></div>`;
  }).join("");
  document.querySelectorAll("#grid .click").forEach(c=>c.onclick=()=>{sel=c.dataset.p;document.querySelector('[data-tab=details]').click()});
}

function bitsHtml(reg,val){
  const d=data.defs.find(x=>x.reg===reg);if(!d)return "";
  return d.bits.map(([b,n,s,desc])=>`<span class="bit ${val&(1<<b)?"set "+s:""}" title="${esc(desc)}">${b} · ${esc(n)}</span>`).join("");
}
function detail(p){
  let h=`<div class="panel"><div class="top"><h2 style="margin:0">${esc(p.name)}</h2><span class="pill ${p.summary}">${SEV[p.summary]}</span></div><div class="loc">${esc(p.loc)}</div>`;
  if(!p.online)return h+`<div class="err">${esc(p.error||"")}</div></div>`;
  const M=data.metrics;
  h+=`<h3>Alarmes actives</h3>`;
  if(!p.alarms.length&&!p.word_flags.filter(f=>f.name!=="OFF").length)h+=`<div class="ok-line">Aucune alarme</div>`;
  else if(!p.alarms.length)h+=`<div class="ok-line">Aucun détail dans les registres STATUS_* — voir STATUS_WORD ci-dessous</div>`;
  else h+=`<table><tr><th>Gravité</th><th>Code</th><th>Registre / bit</th><th>Description</th></tr>`+
    p.alarms.map(a=>`<tr><td><span class="chip ${a.sev}">${SEV[a.sev==="warn"?"warn":a.sev]}</span></td><td><b>${esc(a.name)}</b></td><td>${a.reg} bit ${a.bit}</td><td>${esc(a.desc)}</td></tr>`).join("")+`</table>`;
  h+=`<h3>Mesures</h3><table>`+Object.keys(p.values).map(k=>`<tr><td>${M[k].label}</td><td class="n">${fmt(p.values[k],M[k].dec)} ${M[k].unit}</td></tr>`).join("");
  const e=eff(p.values);if(e!=null)h+=`<tr><td>Rendement (POUT/PIN)</td><td class="n">${e.toFixed(1)} %</td></tr>`;
  if(p.operation!=null)h+=`<tr><td>OPERATION</td><td class="n">${hex(p.operation,2)} (${p.operation&0x80?"ON":"OFF"})</td></tr>`;
  h+=`</table><h3>Registres de statut</h3>`;
  h+=`<div class="reg"><b>STATUS_WORD</b><code>${hex(p.status_word,4)}</code><div class="bits">${bitsHtml("STATUS_WORD",p.status_word)}</div></div>`;
  for(const [r,v] of Object.entries(p.regs))
    h+=`<div class="reg"><b>${r}</b><code>${hex(v,2)}</code><div class="bits">${bitsHtml(r,v)}</div></div>`;
  const inf=Object.entries(p.info||{});
  if(inf.length)h+=`<h3>Identité</h3><table>`+inf.map(([k,v])=>`<tr><td>${esc(k)}</td><td>${esc(v)}</td></tr>`).join("")+`</table>`;
  if(p.limits&&p.limits.length){
    const groups={};p.limits.forEach(l=>(groups[l.group]=groups[l.group]||[]).push(l));
    h+=`<h3>Limites programmées</h3>`+Object.entries(groups).map(([g,ls])=>{
      const key=p.name+"|"+g;
      return `<details data-k="${esc(key)}" ${openSec.has(key)?"open":""}><summary>${esc(g)}</summary><table>`+
        ls.map(l=>`<tr><td>${l.label}</td><td><code>${hex(l.code,2)}</code></td><td class="n">${l.value.toFixed(3)} ${l.unit}</td></tr>`).join("")+`</table></details>`}).join("");
  }
  return h+"</div>";
}
function renderDetails(){
  const opts=`<option value="all">Tous les PSU</option>`+data.psus.map(p=>`<option ${p.name===sel?"selected":""} value="${esc(p.name)}">${esc(p.name)}</option>`).join("");
  if($("sel").innerHTML!==opts)$("sel").innerHTML=opts;
  $("sel").value=sel;
  $("detail").innerHTML=data.psus.filter(p=>sel==="all"||p.name===sel).filter(p=>p.summary!=="pending").map(detail).join("");
  document.querySelectorAll("#detail details").forEach(d=>d.ontoggle=()=>d.open?openSec.add(d.dataset.k):openSec.delete(d.dataset.k));
}
function renderEvents(){
  $("events").innerHTML=`<tr><th>Heure</th><th>PSU</th><th>Gravité</th><th>Événement</th></tr>`+
   (events.length?events.slice().reverse().map(e=>`<tr><td>${new Date(e.t*1000).toLocaleString()}</td><td>${esc(e.psu)}</td><td><span class="chip ${e.sev}">${SEV[e.sev]}</span></td><td>${esc(e.msg)}</td></tr>`).join(""):`<tr><td colspan="4" class="empty">Aucun événement depuis le démarrage</td></tr>`);
}

/* ---------- Graphiques ---------- */
async function loadHist(){
  const r=await fetch("api/history?since="+histSince,{cache:"no-store"}),j=await r.json();
  for(const [n,series] of Object.entries(j)){
    hist[n]=hist[n]||{};
    for(const [k,pts] of Object.entries(series)){
      const a=hist[n][k]=hist[n][k]||[];
      for(const p of pts){a.push(p);if(p[0]>histSince)histSince=p[0]}
      if(a.length>4000)a.splice(0,a.length-4000);
    }
  }
  histLoaded=true;
}
function seriesFor(key){
  const out=[];
  for(const n of names()){
    if(hidden.has(n)||!hist[n])continue;
    let pts;
    if(key==="eff"){const a=hist[n].pin,b=hist[n].pout;if(!a||!b)continue;
      const m=new Map(b);pts=a.filter(p=>p[1]>5&&m.has(p[0])).map(p=>[p[0],m.get(p[0])/p[1]*100]);}
    else pts=hist[n][key];
    if(pts&&pts.length)out.push({name:n,color:color(n),pts});
  }
  return out;
}
function metricKeys(){
  const keys=Object.keys(data.metrics).filter(k=>names().some(n=>hist[n]&&hist[n][k]&&hist[n][k].length));
  if(keys.includes("pin")&&keys.includes("pout"))keys.push("eff");
  return keys;
}
function drawCharts(){
  if(!data||!histLoaded)return;
  $("legend").innerHTML=names().map(n=>`<label class="leg"><input type="checkbox" data-n="${esc(n)}" ${hidden.has(n)?"":"checked"}><i style="background:${color(n)}"></i>${esc(n)}</label>`).join(" ");
  document.querySelectorAll("#legend input").forEach(i=>i.onchange=()=>{i.checked?hidden.delete(i.dataset.n):hidden.add(i.dataset.n);drawCharts()});
  const keys=metricKeys(),key=keys.join(",");
  if(key!==chartKey){
    chartKey=key;
    $("charts").innerHTML=keys.length?keys.map(k=>{
      const m=k==="eff"?{label:"Rendement (POUT/PIN)",unit:"%"}:data.metrics[k];
      return `<div class="ch"><h3>${m.label} <span style="text-transform:none">(${m.unit})</span></h3><canvas data-k="${k}"></canvas></div>`}).join(""):`<div class="empty">Pas encore de données…</div>`;
    document.querySelectorAll("#charts canvas").forEach(cv=>{
      cv.onmousemove=e=>{cv._hx=e.offsetX;paint(cv)};cv.onmouseleave=()=>{cv._hx=null;paint(cv)};});
  }
  document.querySelectorAll("#charts canvas").forEach(paint);
}
function paint(cv){
  const k=cv.dataset.k,dec=k==="eff"?1:data.metrics[k].dec,unit=k==="eff"?"%":data.metrics[k].unit;
  const ser=seriesFor(k),t1=data.time;let t0=t1-parseInt($("range").value);
  const first=Math.min(...ser.map(s=>s.pts[0][0]));   // pas de vide à gauche tant que l'historique est court
  if(first>t0)t0=Math.min(first,t1-30);
  drawChart(cv,ser,unit,Math.min(dec+1,3),t0,t1,data.interval*3);
}
function drawChart(cv,series,unit,dec,t0,t1,gap){
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==Math.round(w*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr)}
  const c=cv.getContext("2d");c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
  const cs=getComputedStyle(document.documentElement),mut=cs.getPropertyValue("--mut"),bd=cs.getPropertyValue("--bd"),fg=cs.getPropertyValue("--fg"),card=cs.getPropertyValue("--card");
  let lo=Infinity,hi=-Infinity;
  for(const s of series)for(const p of s.pts){if(p[0]<t0)continue;if(p[1]<lo)lo=p[1];if(p[1]>hi)hi=p[1]}
  c.font="11px system-ui";c.fillStyle=mut;
  if(lo===Infinity){c.textAlign="center";c.fillText("pas de données sur cette période",w/2,h/2);return}
  if(hi-lo<1e-9){lo-=1;hi+=1}else{const pad=(hi-lo)*.1;lo-=pad;hi+=pad}
  const L=50,R=8,T=8,B=20,pw=w-L-R,ph=h-T-B,X=t=>L+(t-t0)/(t1-t0)*pw,Y=v=>T+(1-(v-lo)/(hi-lo))*ph;
  c.strokeStyle=bd;c.lineWidth=1;
  for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4,y=Y(v);c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.stroke();c.textAlign="right";c.fillStyle=mut;c.fillText(v.toFixed(dec),L-5,y+4)}
  for(let i=0;i<=4;i++){const t=t0+(t1-t0)*i/4;c.textAlign=i===0?"left":i===4?"right":"center";c.fillStyle=mut;
    c.fillText(new Date(t*1000).toLocaleTimeString([],{hour:"2-digit",minute:"2-digit",second:"2-digit"}),X(t),h-5)}
  c.lineWidth=1.8;c.lineJoin="round";
  for(const s of series){c.strokeStyle=s.color;c.beginPath();let prev=null;
    for(const p of s.pts){if(p[0]<t0)continue;const x=X(p[0]),y=Y(p[1]);
      if(prev===null||p[0]-prev>gap)c.moveTo(x,y);else c.lineTo(x,y);prev=p[0]}c.stroke()}
  if(cv._hx!=null&&cv._hx>=L&&cv._hx<=w-R){
    const t=t0+(cv._hx-L)/pw*(t1-t0);c.strokeStyle=mut;c.beginPath();c.moveTo(cv._hx,T);c.lineTo(cv._hx,T+ph);c.stroke();
    const lines=[];let tt=t;
    for(const s of series){let best=null;for(const p of s.pts)if(best===null||Math.abs(p[0]-t)<Math.abs(best[0]-t))best=p;
      if(best&&Math.abs(best[0]-t)<gap*2){lines.push([s.color,s.name+" : "+best[1].toFixed(dec)+" "+unit]);tt=best[0]}}
    if(lines.length){
      const head=new Date(tt*1000).toLocaleTimeString(),bw=Math.max(c.measureText(head).width,...lines.map(l=>c.measureText(l[1]).width))+26,bh=16*(lines.length+1)+6;
      let bx=cv._hx+10;if(bx+bw>w-R)bx=cv._hx-bw-10;
      c.fillStyle=card;c.strokeStyle=bd;c.fillRect(bx,T+4,bw,bh);c.strokeRect(bx,T+4,bw,bh);
      c.textAlign="left";c.fillStyle=mut;c.fillText(head,bx+8,T+18);
      lines.forEach((l,i)=>{c.fillStyle=l[0];c.fillRect(bx+8,T+25+i*16,8,8);c.fillStyle=fg;c.fillText(l[1],bx+22,T+33+i*16)});
    }
  }
}

/* ---------- Boucle principale ---------- */
function render(){
  if(!data)return;
  renderSummary();
  if(tab==="overview")renderOverview();
  if(tab==="details")renderDetails();
  if(tab==="events")renderEvents();
}
async function tick(){
  try{
    const r=await fetch("api/data",{cache:"no-store"});
    if(r.status===401){location.reload();return}
    data=await r.json();
    if(!window._defs)window._defs=await fetch("api/defs").then(x=>x.json());
    data.defs=window._defs;
    const er=await fetch("api/events?since="+evId,{cache:"no-store"}),ej=await er.json();
    for(const e of ej){events.push(e);evId=Math.max(evId,e.id)}
    if(events.length>1000)events.splice(0,events.length-1000);
    render();
    if(histLoaded){await loadHist();if(tab==="graphs")drawCharts()}
  }catch(e){$("meta").textContent="connexion perdue…"}
  setTimeout(tick,Math.max(1000,(data?data.interval:2)*1000));
}
tick();
</script></body></html>
"""


# --------------------------------------------------------------------------
# Serveur HTTP(S)
# --------------------------------------------------------------------------
def make_handler(state, auth):
    defs_json = json.dumps(status_defs()).encode()
    page = HTML_PAGE.encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        server_version = "pmbus-monitor"

        def _authorized(self):
            if not auth:
                return True
            hdr = self.headers.get("Authorization", "")
            if hdr.startswith("Basic "):
                try:
                    got = base64.b64decode(hdr[6:]).decode("utf-8", "replace")
                    return hmac.compare_digest(got.encode(), auth.encode())
                except ValueError:
                    pass
            return False

        def _send(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self._authorized():
                self.send_response(401)
                self.send_header("WWW-Authenticate", 'Basic realm="PMBus Monitor"')
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)

            def num(name, default=0.0):
                try:
                    return float(q.get(name, [default])[0])
                except ValueError:
                    return default

            if u.path in ("/", "/index.html"):
                self._send(page, "text/html; charset=utf-8")
            elif u.path == "/api/data":
                snap = state.snapshot()
                snap["metrics"] = METRICS_META
                self._send(json.dumps(snap).encode(), "application/json")
            elif u.path == "/api/defs":
                self._send(defs_json, "application/json")
            elif u.path == "/api/history":
                self._send(json.dumps(state.history(num("since"))).encode(), "application/json")
            elif u.path == "/api/events":
                self._send(json.dumps(state.events_since(int(num("since")))).encode(), "application/json")
            elif u.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

        def log_message(self, *_):
            pass

    return Handler


class Server(ThreadingHTTPServer):
    ctx = None
    request_queue_size = 16

    def get_request(self):
        sock, addr = self.socket.accept()
        sock.settimeout(15)
        if self.ctx:   # la négociation TLS se fait dans le thread du client, pas dans l'accept
            sock = self.ctx.wrap_socket(sock, server_side=True, do_handshake_on_connect=False)
        return sock, addr

    def handle_error(self, request, client_address):
        pass   # navigateur qui coupe / handshake raté : sans importance


def local_ips():
    ips = set()
    try:
        ips.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))   # aucun paquet envoyé : sert à trouver l'IP de la route par défaut
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:   # BBB : l'IP USB (192.168.7.2) n'a pas toujours de route par défaut
        out = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True).stdout
        for line in out.splitlines():
            ips.add(line.split()[3].split("/")[0])
    except (OSError, IndexError):
        pass
    return sorted(i for i in ips if not i.startswith("127."))


def ensure_cert(certdir, regen=False):
    crt, key = os.path.join(certdir, "server.crt"), os.path.join(certdir, "server.key")
    if not regen and os.path.exists(crt) and os.path.exists(key):
        return crt, key
    os.makedirs(certdir, mode=0o700, exist_ok=True)
    host = socket.gethostname()
    san = ",".join(["DNS:localhost", "DNS:" + host, "IP:127.0.0.1"] + ["IP:" + i for i in local_ips()])
    base = ["openssl", "req", "-x509", "-newkey", "ec", "-pkeyopt", "ec_paramgen_curve:prime256v1",
            "-nodes", "-keyout", key, "-out", crt, "-days", "3650", "-subj", "/CN=pmbus-monitor"]
    for extra in (["-addext", "subjectAltName=" + san], []):   # vieux OpenSSL : sans SAN
        try:
            subprocess.run(base + extra, check=True, capture_output=True)
            break
        except FileNotFoundError:
            sys.exit("openssl introuvable : installe-le (apt install openssl) ou fournis --cert/--key.")
        except subprocess.CalledProcessError:
            continue
    else:
        sys.exit("Impossible de générer le certificat HTTPS (openssl a échoué).")
    os.chmod(key, 0o600)
    print("Certificat auto-signé créé dans %s" % certdir)
    return crt, key


def wait_first_poll(state, psus, timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        with state.lock:
            if all(p.name in state.latest for p in psus):
                return
        time.sleep(0.2)


def print_banner(state, psus, workers, scheme, port, args):
    ips = [args.host] if args.host != "0.0.0.0" else local_ips()
    host = socket.gethostname()
    line = "=" * 64
    print("\n" + line)
    print(" PMBus Monitor - interface web disponible")
    print(line)
    print(" Ouvre l'une de ces adresses dans ton navigateur :")
    for ip in ips:
        print("   %s://%s:%d" % (scheme, ip, port))
    if args.host == "0.0.0.0":
        print("   %s://%s.local:%d   (si ton réseau gère le mDNS)" % (scheme, host, port))
    if not ips:
        print("   %s://<ip-de-la-BBB>:%d   (aucune IP détectée)" % (scheme, port))
    if scheme == "https":
        print(" Certificat auto-signé : accepte l'avertissement du navigateur")
        print("   (Avancé > Continuer). Certificat : %s" % args.certdir)
    if args.auth:
        print(" Connexion : utilisateur '%s' (mot de passe défini avec --auth)" % args.auth.split(":")[0])
    else:
        print(" Accès sans mot de passe (ajoute --auth utilisateur:motdepasse)")
    print(" Intervalle de lecture : %g s - historique : %d min" % (args.interval, args.history // 60))
    print("-" * 64)
    print(" %d PSU, %d bus I2C :" % (len(psus), len(workers)))
    with state.lock:
        for p in psus:
            r = state.latest.get(p.name, {})
            if r.get("online"):
                info = r.get("info", {})
                what = " ".join(x for x in (info.get("Fabricant (MFR_ID)"), info.get("Modèle (MFR_MODEL)")) if x)
                st = {"ok": "OK", "warn": "AVERTISSEMENT", "fault": "DÉFAUT", "off": "ÉTEINT"}.get(r["summary"], r["summary"])
                print("   [%-13s] %-12s %s  %s" % (st, p.name, p.loc, what))
            else:
                print("   [HORS LIGNE   ] %-12s %s  %s" % (p.name, p.loc, r.get("error", "")))
    if not any(state.latest.get(p.name, {}).get("online") for p in psus):
        print("\n Aucun PSU ne répond : vérifie le câblage et l'alimentation du PSU, lance --scan")
        print(" pour voir les adresses, puis relance avec -b <bus> --addr 0x5X (et --mux 0x7X avec un PDB).")
    print(line)
    print(" Ctrl+C pour arrêter\n", flush=True)


def run_web(workers, psus, args):
    state = State(psus, args.interval, args.history)
    stop = threading.Event()
    csv_lock = threading.Lock()

    def loop(w):
        while not stop.is_set():
            t0 = time.time()
            try:
                results = w.poll()
                state.update(results)
                if args.csv:
                    with csv_lock:
                        write_csv(args.csv, results)
            except Exception as e:   # un incident sur un bus ne doit pas tuer le thread
                print("bus %d : %s" % (w.num, e), file=sys.stderr)
            stop.wait(max(0.05, args.interval - (time.time() - t0)))

    threads = [threading.Thread(target=loop, args=(w,), daemon=True) for w in workers]
    for t in threads:
        t.start()

    port = args.port or (8080 if args.http else 8443)
    server = Server((args.host, port), make_handler(state, args.auth))
    scheme = "http"
    if not args.http:
        if args.cert and args.key:
            crt, key = args.cert, args.key
        else:
            crt, key = ensure_cert(args.certdir, args.regen_cert)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        ctx.load_cert_chain(crt, key)
        server.ctx = ctx
        scheme = "https"
    wait_first_poll(state, psus)
    print_banner(state, psus, workers, scheme, port, args)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        for t in threads:
            t.join(timeout=args.interval + 2)


# --------------------------------------------------------------------------
# Terminal / JSON / CSV
# --------------------------------------------------------------------------
def fv(r, key):
    for k, _c, _u, dec, _l in TELEMETRY:
        if k == key:
            return ("%.*f" % (dec if key != "vout" else 2, r["values"][key])) if key in r.get("values", {}) else "-"
    return "-"


def render_table(results):
    cols = [("PSU", 12), ("VIN", 7), ("IIN", 7), ("PIN", 7), ("VOUT", 7), ("IOUT", 7),
            ("POUT", 7), ("T1", 6), ("FAN1", 6), ("ÉTAT", 0)]
    keys = [None, "vin", "iin", "pin", "vout", "iout", "pout", "temp1", "fan1", None]
    lines = ["".join(n.ljust(w) if w else n for n, w in cols), "-" * 90]
    for r in results:
        if not r.get("online"):
            lines.append("%-12s HORS LIGNE : %s  -> essaie --scan ou --autodetect" % (r["name"][:11], r.get("error", "")))
            continue
        cells = [r["name"][:11].ljust(12)] + [fv(r, k).ljust(w) for (n, w), k in zip(cols[1:-1], keys[1:-1])]
        state = "OK" if r["summary"] == "ok" else (
            ", ".join(a["name"] for a in r["alarms"]) or ", ".join(f["name"] for f in r["word_flags"])
            or r["summary"].upper())
        lines.append("".join(cells) + state)
    lines += ["", "Unités : V, A, W, T = °C, FAN = rpm"]
    return "\n".join(lines)


def write_csv(path, results):
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["timestamp", "psu", "summary", "status_word"] + TELEMETRY_KEYS + ["alarms", "error"])
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        for r in results:
            v = r.get("values", {})
            w.writerow([ts, r["name"], r["summary"],
                        "0x%04X" % r["status_word"] if "status_word" in r else ""]
                       + [("%.3f" % v[k]) if k in v else "" for k in TELEMETRY_KEYS]
                       + [";".join(a["name"] for a in r.get("alarms", [])), r.get("error", "")])


def run_terminal(workers, args):
    first_loop = True
    while True:
        results = []
        for w in workers:
            results += w.poll()
        if args.csv:
            write_csv(args.csv, results)
        if args.json:
            print(json.dumps({"time": time.time(), "psu": results}), flush=True)
        else:
            if not args.once and sys.stdout.isatty():
                sys.stdout.write("\x1b[2J\x1b[H")
            print("PMBus monitor - %s" % time.strftime("%H:%M:%S"))
            print(render_table(results), flush=True)
            if args.info and first_loop:
                for r in results:
                    if r.get("online"):
                        print("\n%s (%s)" % (r["name"], r["loc"]))
                        for k, v in r["info"].items():
                            print("  %-26s %s" % (k, v))
        first_loop = False
        if args.once:
            return
        time.sleep(args.interval)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Moniteur PMBus pour BeagleBone Black",
                                formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("-b", "--bus", type=int, help="bus I2C (défaut : tous les bus détectés ; 2 pour une config manuelle)")
    p.add_argument("-a", "--addr", type=int_auto, nargs="+", help="adresse(s) PMBus des PSU (défaut 0x58)")
    p.add_argument("--mux", type=int_auto, help="adresse du mux I2C du PDB (ex: 0x70)")
    p.add_argument("--channels", nargs="+", default=["0-7"], help="canaux du mux, ex: 0-3 (défaut 0-7)")
    p.add_argument("--pages", type=int, nargs="+", help="pages PMBus pour les PSU multi-rails, ex: 0 1")
    p.add_argument("--config", metavar="FICHIER", help="fichier JSON décrivant tous les PSU (multi-bus, multi-mux)")
    p.add_argument("--autodetect", action="store_true", help="détecter les PSU sur tous les bus (déjà le cas sans --addr/--config)")
    p.add_argument("-i", "--interval", type=float, help="période de lecture en s (défaut 2)")
    p.add_argument("--scan", action="store_true", help="rapport complet : bus, adresses, mux, PSU PMBus détectés")
    p.add_argument("--once", action="store_true", help="une seule mesure puis quitter")
    p.add_argument("--info", action="store_true", help="terminal : afficher l'identité des PSU")
    p.add_argument("--json", action="store_true", help="sortie JSON (terminal)")
    p.add_argument("--csv", metavar="FICHIER", help="ajouter les mesures à un fichier CSV")
    w = p.add_argument_group("interface web")
    p.add_argument("--terminal", action="store_true", help="affichage terminal au lieu de l'interface web")
    w.add_argument("--web", action="store_true", help="(défaut) servir l'interface web, HTTPS par défaut")
    w.add_argument("--port", type=int, help="port (défaut 8443 en HTTPS, 8080 en HTTP)")
    w.add_argument("--host", default="0.0.0.0", help="adresse d'écoute (défaut toutes)")
    w.add_argument("--http", action="store_true", help="HTTP clair, sans TLS")
    w.add_argument("--certdir", default=os.path.expanduser("~/.pmbus_monitor"),
                   help="dossier du certificat auto-signé (défaut ~/.pmbus_monitor)")
    w.add_argument("--cert", help="certificat PEM à utiliser (avec --key)")
    w.add_argument("--key", help="clé privée PEM (avec --cert)")
    w.add_argument("--regen-cert", action="store_true", help="régénérer le certificat auto-signé")
    w.add_argument("--auth", metavar="USER:MOT_DE_PASSE", default=os.environ.get("PMBUS_AUTH"),
                   help="protéger la page par mot de passe (ou variable PMBUS_AUTH)")
    w.add_argument("--history", type=int, default=3600, help="durée d'historique gardée en mémoire, en s (défaut 3600)")
    args = p.parse_args()
    args.channels = parse_channels(args.channels)
    args.bus_explicit = args.bus is not None
    if args.bus is None:
        args.bus = 2
    # L'interface web est le mode par défaut ; --once / --json / --terminal donnent le mode terminal.
    args.web = not (args.terminal or args.once or args.json)

    if args.scan:
        discover(args)
        return

    cfg_interval = None
    try:
        if args.config:
            psus, cfg_interval = psus_from_config(args.config, args.bus)
        elif args.autodetect or (args.addr is None and not args.pages):
            psus = discover(args)
        else:
            psus = psus_from_args(args)
    except (OSError, ValueError, KeyError) as e:
        sys.exit("Configuration impossible : %s" % e)
    args.interval = args.interval or cfg_interval or 2.0
    if not psus and not args.web:
        sys.exit("Aucun PSU trouvé. Lance --scan pour voir ce qui répond sur les bus.")
    if psus and not args.web:
        print("%d PSU : %s" % (len(psus), ", ".join("%s (%s)" % (x.name, x.loc) for x in psus)))
    unique_names(psus)

    workers = []
    try:
        for b in sorted({x.bus for x in psus}):
            workers.append(BusWorker(b, [x for x in psus if x.bus == b]))
    except OSError as e:
        sys.exit("Impossible d'ouvrir /dev/i2c-%d : %s\n"
                 "Vérifie que le bus est activé (overlay) et que tu as les droits." % (b, e))

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        if args.web:
            run_web(workers, psus, args)
        else:
            run_terminal(workers, args)
    except KeyboardInterrupt:
        pass
    finally:
        for w in workers:
            w.close()


if __name__ == "__main__":
    main()
