#!/usr/bin/env python3
"""
pmbus_monitor.py - Moniteur PMBus mono-fichier pour BeagleBone Black.

Python 3 standard. `pip install smbus2` est recommandé (pilote utilisé en priorité) ; sans lui,
le script parle directement à /dev/i2c-N. `openssl` génère le certificat HTTPS auto-signé.
Chaque transaction I2C est temporisée (3 ms) et retentée : nécessaire avec beaucoup de PSU.

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
  ./pmbus_monitor.py --mock                                   # 3 PSU simulés, pour tester la page sans matériel
  ./pmbus_monitor.py --auth admin:secret --control            # + boutons effacer défauts / ON / OFF

Interface web : http://<ip-bbb>:8080 (--https pour HTTPS avec certificat auto-signé, port 8443).

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
import io
import json
import math
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


try:
    from smbus2 import SMBus as _SMBus
except ImportError:
    _SMBus = None

I2C_SMBUS_I2C_BLOCK_DATA = 8


class I2CBus:
    """Bus I2C : smbus2 s'il est installé, sinon ioctl direct.
    Chaque transaction est temporisée (pacing) et retentée : beaucoup de PSU ratent la 1re."""
    pace = 0.003          # pause après chaque transaction réussie (s)

    def __init__(self, bus):
        self.num = bus
        self.retries = 2   # nouvelles tentatives par transaction (0 pendant les balayages)
        self.sm = _SMBus(bus) if _SMBus else None
        self.backend = "smbus2" if self.sm else "ioctl"
        if not self.sm:
            self.fd = os.open("/dev/i2c-%d" % bus, os.O_RDWR)
            self.addr = None

    def close(self):
        if self.sm:
            self.sm.close()
        else:
            os.close(self.fd)

    def _xfer(self, addr, rw, cmd, size, data=None):
        if addr != self.addr:
            fcntl.ioctl(self.fd, I2C_SLAVE_FORCE, addr)
            self.addr = addr
        d = data or _SmbusData()
        fcntl.ioctl(self.fd, I2C_SMBUS, _SmbusIoctl(rw, cmd, size, ctypes.pointer(d)))
        return d

    def _try(self, fn, *a):
        n = self.retries + 1
        for i in range(n):
            try:
                r = fn(*a)
                if self.pace:
                    time.sleep(self.pace)
                return r
            except OSError:
                if i == n - 1:
                    raise
                time.sleep(0.005)

    def write_byte(self, addr, value):
        if self.sm:
            return self._try(self.sm.write_byte, addr, value)
        self._try(self._xfer, addr, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE)

    def read_byte(self, addr):
        if self.sm:
            return self._try(self.sm.read_byte, addr)
        return self._try(self._xfer, addr, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE).byte

    def read_byte_data(self, addr, cmd):
        if self.sm:
            return self._try(self.sm.read_byte_data, addr, cmd)
        return self._try(self._xfer, addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_BYTE_DATA).byte

    def write_byte_data(self, addr, cmd, value):
        if self.sm:
            return self._try(self.sm.write_byte_data, addr, cmd, value)
        d = _SmbusData()
        d.byte = value
        self._try(self._xfer, addr, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE_DATA, d)

    def read_word_data(self, addr, cmd):
        if self.sm:
            return self._try(self.sm.read_word_data, addr, cmd)
        return self._try(self._xfer, addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_WORD_DATA).word

    def write_word_data(self, addr, cmd, value):
        if self.sm:
            return self._try(self.sm.write_word_data, addr, cmd, value)
        d = _SmbusData()
        d.word = value
        self._try(self._xfer, addr, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_WORD_DATA, d)

    def read_block_data(self, addr, cmd):
        """Chaîne PMBus (octet de longueur + caractères). On essaie d'abord le « I2C block read »
        car le contrôleur I2C de la BBB ne sait pas faire le SMBus block read."""
        try:
            if self.sm:
                raw = bytes(self._try(self.sm.read_i2c_block_data, addr, cmd, 17))
            else:
                d = _SmbusData()
                d.block[0] = 17
                d = self._try(self._xfer, addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_I2C_BLOCK_DATA, d)
                raw = bytes(d.block[1:18])
            if 0 < raw[0] <= 16:
                return raw[1:1 + raw[0]]
        except OSError:
            pass
        if self.sm:
            return bytes(self.sm.read_block_data(addr, cmd))
        d = self._try(self._xfer, addr, I2C_SMBUS_READ, cmd, I2C_SMBUS_BLOCK_DATA)
        return bytes(d.block[1:1 + min(d.block[0], 32)])


# --------------------------------------------------------------------------
# Simulation (--mock) : 3 PSU fictifs, pour essayer l'interface sans matériel
# --------------------------------------------------------------------------
def encode_linear11(v):
    exp = -16
    while abs(v / (2.0 ** exp)) > 1000 and exp < 15:
        exp += 1
    return ((exp & 0x1F) << 11) | (int(round(v / (2.0 ** exp))) & 0x7FF)


class MockBus:
    """Même interface que I2CBus. 0x58 = OK, 0x59 = en défaut, 0x5A = éteint."""
    backend = "simulation"
    PSUS = {0x58: "ok", 0x59: "fault", 0x5A: "off"}

    def __init__(self, bus):
        self.num, self.retries = bus, 0
        self.op = {a: (0x00 if k == "off" else 0x80) for a, k in self.PSUS.items()}

    def close(self):
        pass

    def _kind(self, a):
        if a not in self.PSUS:
            raise OSError(121, "Remote I/O error")
        return self.PSUS[a]

    def read_byte(self, a):
        self._kind(a)
        return 0

    def write_byte(self, a, v):
        if a in self.PSUS and v == 0x03:   # CLEAR_FAULTS
            return
        raise OSError(121, "Remote I/O error")

    def write_byte_data(self, a, c, v):
        self._kind(a)
        if c == CMD_OPERATION:
            self.op[a] = v

    def write_word_data(self, a, c, v):
        self._kind(a)

    def read_byte_data(self, a, c):
        k = self._kind(a)
        off = self.op[a] == 0
        if c == 0x78:
            return (0x40 if off else 0x00) | (0x04 if k == "fault" else 0)
        if c == CMD_VOUT_MODE:
            return 0x17
        if c == CMD_OPERATION:
            return self.op[a]
        if c == CMD_PMBUS_REVISION:
            return 0x33
        if c == CMD_CAPABILITY:
            return 0xB0
        if c == 0x7C and k == "fault":
            return 0x30        # VIN_UV_WARNING + VIN_UV_FAULT
        if c == 0x7D and k == "fault":
            return 0x40        # OT_WARNING
        if c in (0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81):
            return 0
        raise OSError(121, "nack")

    def read_word_data(self, a, c):
        k = self._kind(a)
        off = self.op[a] == 0
        t = time.time() + a
        w = math.sin(t / 6.0)
        if c == CMD_STATUS_WORD:
            return (0x0040 if off else 0) | (0x2004 if k == "fault" else 0)
        pout = 0.0 if off else 600 + 150 * w
        iout = pout / 12.0
        table = {0x88: 230.0 + w, 0x89: 0 if off else (pout / 0.93) / 230.0, 0x97: 0 if off else pout / 0.93,
                 0x96: pout, 0x8C: iout, 0x8D: 28 if off else 41 + 3 * w, 0x8E: 27 if off else 36 + 2 * w,
                 0x90: 1200 if off else 4200 + 300 * w, 0x91: 1200 if off else 4100 + 250 * w,
                 0x55: 264.0, 0x59: 170.0, 0x57: 260.0, 0x58: 180.0, 0x4F: 100.0, 0x51: 85.0, 0xA7: 1000.0}
        if c == 0x8B:
            return 0 if off else int(round(12.0 * 2 ** 9))        # LINEAR16, exposant -9
        if c in (0x21, 0x40, 0x42, 0x43, 0x44, 0xA4, 0xA5):
            return int(round({0x40: 14.0, 0x42: 13.2, 0x43: 10.8, 0x44: 10.0}.get(c, 12.0) * 2 ** 9))
        if c in table:
            return encode_linear11(table[c])
        raise OSError(121, "nack")

    def read_block_data(self, a, c):
        self._kind(a)
        txt = {0x99: b"ACME", 0x9A: b"SIM-1000W", 0x9B: b"R1.0", 0x9E: b"SIM%04X" % a}.get(c)
        if txt is None:
            raise OSError(121, "nack")
        return txt


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


REGISTERS = [   # (code, nom, type, catégorie, description) - table inspirée de pmbus_multi_psu_gui.py
    (0x00, 'PAGE', 'byte_page', 'Configuration', 'Active channel/phase selection (0 to 31, 0xFF=All)'),
    (0x01, 'OPERATION', 'byte_op', 'Configuration', 'Active power stage state (0x80=On, 0x00=Off, Margins)'),
    (0x02, 'ON_OFF_CONFIG', 'byte_onoff', 'Configuration', 'Hardware CONTROL pin and software enable logic'),
    (0x10, 'WRITE_PROTECT', 'byte_wp', 'Configuration', 'Register write protection status'),
    (0x19, 'CAPABILITY', 'byte_cap', 'Configuration', 'Supported features (PEC, Max speed, SMBALERT#)'),
    (0x35, 'VIN_ON', 'word_volt', 'Startup & Input (VIN)', 'Input voltage turn-on / startup threshold (V)'),
    (0x36, 'VIN_OFF', 'word_volt', 'Startup & Input (VIN)', 'Input voltage turn-off / undervoltage shutdown threshold (V)'),
    (0x55, 'VIN_OV_FAULT_LIMIT', 'word_volt', 'Startup & Input (VIN)', 'Input Overvoltage Fault shutdown threshold (V)'),
    (0x56, 'VIN_OV_FAULT_RESPONSE', 'byte_resp', 'Startup & Input (VIN)', 'Response profile for Input Overvoltage Fault'),
    (0x57, 'VIN_OV_WARN_LIMIT', 'word_volt', 'Startup & Input (VIN)', 'Input Overvoltage Warning advisory threshold (V)'),
    (0x58, 'VIN_UV_WARN_LIMIT', 'word_volt', 'Startup & Input (VIN)', 'Input Undervoltage Warning advisory threshold (V)'),
    (0x59, 'VIN_UV_FAULT_LIMIT', 'word_volt', 'Startup & Input (VIN)', 'Input Undervoltage Fault shutdown threshold (V)'),
    (0x5A, 'VIN_UV_FAULT_RESPONSE', 'byte_resp', 'Startup & Input (VIN)', 'Response profile for Input Undervoltage Fault'),
    (0x5B, 'IIN_OC_FAULT_LIMIT', 'word_curr', 'Startup & Input (VIN)', 'Input Overcurrent Fault shutdown threshold (A)'),
    (0x5C, 'IIN_OC_FAULT_RESPONSE', 'byte_resp', 'Startup & Input (VIN)', 'Response profile for Input Overcurrent Fault'),
    (0x5D, 'IIN_OC_WARN_LIMIT', 'word_curr', 'Startup & Input (VIN)', 'Input Overcurrent Warning advisory threshold (A)'),
    (0x6B, 'PIN_OP_WARN_LIMIT', 'word_power', 'Startup & Input (VIN)', 'Input Overpower Warning advisory threshold (W)'),
    (0x88, 'READ_VIN', 'word_volt', 'Startup & Input (VIN)', 'Real-time measured input voltage (V)'),
    (0x89, 'READ_IIN', 'word_curr', 'Startup & Input (VIN)', 'Real-time measured input current (A)'),
    (0x97, 'READ_PIN', 'word_power', 'Startup & Input (VIN)', 'Real-time measured input power (W)'),
    (0x20, 'VOUT_MODE', 'byte_vmode', 'Output Voltage (VOUT)', 'VOUT data format (Linear/VID/Direct) and exponent'),
    (0x21, 'VOUT_COMMAND', 'word_vout', 'Output Voltage (VOUT)', 'Target output voltage command setpoint (V)'),
    (0x22, 'VOUT_TRIM', 'word_vout', 'Output Voltage (VOUT)', 'Fine trimming offset for output voltage (V)'),
    (0x23, 'VOUT_CAL_OFFSET', 'word_vout', 'Output Voltage (VOUT)', 'Output voltage calibration offset (V)'),
    (0x24, 'VOUT_MAX', 'word_vout', 'Output Voltage (VOUT)', 'Upper safety clamp limit for VOUT (V)'),
    (0x25, 'VOUT_MARGIN_HIGH', 'word_vout', 'Output Voltage (VOUT)', 'Target voltage for high margin testing (V)'),
    (0x26, 'VOUT_MARGIN_LOW', 'word_vout', 'Output Voltage (VOUT)', 'Target voltage for low margin testing (V)'),
    (0x27, 'VOUT_TRANSITION_RATE', 'word_v_ms', 'Output Voltage (VOUT)', 'Output voltage slew rate during transitions (V/ms)'),
    (0x28, 'VOUT_DROOP', 'word_droop', 'Output Voltage (VOUT)', 'Active voltage positioning loadline slope (mV/A)'),
    (0x29, 'VOUT_SCALE_LOOP', 'word_linear_ratio', 'Output Voltage (VOUT)', 'Feedback divider scaling ratio'),
    (0x40, 'VOUT_OV_FAULT_LIMIT', 'word_vout', 'Output Voltage (VOUT)', 'Output Overvoltage Fault shutdown threshold (V)'),
    (0x41, 'VOUT_OV_FAULT_RESPONSE', 'byte_resp', 'Output Voltage (VOUT)', 'Response profile for VOUT Overvoltage Fault'),
    (0x42, 'VOUT_OV_WARN_LIMIT', 'word_vout', 'Output Voltage (VOUT)', 'Output Overvoltage Warning advisory threshold (V)'),
    (0x43, 'VOUT_UV_WARN_LIMIT', 'word_vout', 'Output Voltage (VOUT)', 'Output Undervoltage Warning advisory threshold (V)'),
    (0x44, 'VOUT_UV_FAULT_LIMIT', 'word_vout', 'Output Voltage (VOUT)', 'Output Undervoltage Fault shutdown threshold (V)'),
    (0x45, 'VOUT_UV_FAULT_RESPONSE', 'byte_resp', 'Output Voltage (VOUT)', 'Response profile for VOUT Undervoltage Fault'),
    (0x5E, 'POWER_GOOD_ON', 'word_vout', 'Output Voltage (VOUT)', 'Output voltage threshold for Power Good assertion (V)'),
    (0x5F, 'POWER_GOOD_OFF', 'word_vout', 'Output Voltage (VOUT)', 'Output voltage threshold for Power Good deassertion (V)'),
    (0x8B, 'READ_VOUT', 'word_vout', 'Output Voltage (VOUT)', 'Real-time measured output voltage (V)'),
    (0x60, 'TON_DELAY', 'word_ms', 'Sequencing & Timings', 'Turn-on delay time from enable to ramp (ms)'),
    (0x61, 'TON_RISE', 'word_ms', 'Sequencing & Timings', 'Turn-on rise / soft-start ramp time (ms)'),
    (0x62, 'TON_MAX_FAULT_LIMIT', 'word_ms', 'Sequencing & Timings', 'Maximum allowed turn-on transition time (ms)'),
    (0x63, 'TON_MAX_FAULT_RESPONSE', 'byte_resp', 'Sequencing & Timings', 'Response profile for TON Max Fault'),
    (0x64, 'TOFF_DELAY', 'word_ms', 'Sequencing & Timings', 'Turn-off delay time (ms)'),
    (0x65, 'TOFF_FALL', 'word_ms', 'Sequencing & Timings', 'Turn-off ramp-down fall time (ms)'),
    (0x66, 'TOFF_MAX_WARN_LIMIT', 'word_ms', 'Sequencing & Timings', 'Turn-off maximum warning limit (ms)'),
    (0x31, 'POUT_MAX', 'word_power', 'Current & Power (IOUT)', 'Maximum continuous output power rating (W)'),
    (0x38, 'IOUT_CAL_GAIN', 'word_mohm', 'Current & Power (IOUT)', 'Current sense shunt resistance (mOhm)'),
    (0x39, 'IOUT_CAL_OFFSET', 'word_curr', 'Current & Power (IOUT)', 'Output current calibration offset (A)'),
    (0x46, 'IOUT_OC_FAULT_LIMIT', 'word_curr', 'Current & Power (IOUT)', 'Output Overcurrent Fault shutdown threshold (A)'),
    (0x47, 'IOUT_OC_FAULT_RESPONSE', 'byte_resp', 'Current & Power (IOUT)', 'Response profile for Output Overcurrent Fault'),
    (0x48, 'IOUT_OC_LV_FAULT_LIMIT', 'word_vout', 'Current & Power (IOUT)', 'Output Overcurrent Low-Voltage shutdown threshold (V)'),
    (0x49, 'IOUT_OC_LV_FAULT_RESPONSE', 'byte_resp', 'Current & Power (IOUT)', 'Response profile for OC Low-Voltage Fault'),
    (0x4A, 'IOUT_OC_WARN_LIMIT', 'word_curr', 'Current & Power (IOUT)', 'Output Overcurrent Warning advisory threshold (A)'),
    (0x4B, 'IOUT_UC_FAULT_LIMIT', 'word_curr', 'Current & Power (IOUT)', 'Output Undercurrent Fault shutdown threshold (A)'),
    (0x4C, 'IOUT_UC_FAULT_RESPONSE', 'byte_resp', 'Current & Power (IOUT)', 'Response profile for Output Undercurrent Fault'),
    (0x68, 'POUT_OP_FAULT_LIMIT', 'word_power', 'Current & Power (IOUT)', 'Output Overpower Fault shutdown threshold (W)'),
    (0x69, 'POUT_OP_FAULT_RESPONSE', 'byte_resp', 'Current & Power (IOUT)', 'Response profile for Output Overpower Fault'),
    (0x6A, 'POUT_OP_WARN_LIMIT', 'word_power', 'Current & Power (IOUT)', 'Output Overpower Warning advisory threshold (W)'),
    (0x8C, 'READ_IOUT', 'word_curr', 'Current & Power (IOUT)', 'Real-time measured output current (A)'),
    (0x96, 'READ_POUT', 'word_power', 'Current & Power (IOUT)', 'Real-time measured output power (W)'),
    (0x4F, 'OT_FAULT_LIMIT', 'word_temp', 'Thermal & Cooling', 'Overtemperature Fault shutdown threshold (deg C)'),
    (0x50, 'OT_FAULT_RESPONSE', 'byte_resp', 'Thermal & Cooling', 'Response profile for Overtemperature Fault'),
    (0x51, 'OT_WARN_LIMIT', 'word_temp', 'Thermal & Cooling', 'Overtemperature Warning advisory threshold (deg C)'),
    (0x52, 'UT_WARN_LIMIT', 'word_temp', 'Thermal & Cooling', 'Undertemperature Warning advisory threshold (deg C)'),
    (0x53, 'UT_FAULT_LIMIT', 'word_temp', 'Thermal & Cooling', 'Undertemperature Fault shutdown threshold (deg C)'),
    (0x54, 'UT_FAULT_RESPONSE', 'byte_resp', 'Thermal & Cooling', 'Response profile for Undertemperature Fault'),
    (0x3A, 'FAN_CONFIG_1_2', 'byte_hex', 'Thermal & Cooling', 'Fan 1 and 2 configuration parameters'),
    (0x3B, 'FAN_COMMAND_1', 'word_rpm', 'Thermal & Cooling', 'Fan 1 speed command setpoint (RPM or %)'),
    (0x3C, 'FAN_COMMAND_2', 'word_rpm', 'Thermal & Cooling', 'Fan 2 speed command setpoint (RPM or %)'),
    (0x8D, 'READ_TEMPERATURE_1', 'word_temp', 'Thermal & Cooling', 'Primary power stage temperature reading (deg C)'),
    (0x8E, 'READ_TEMPERATURE_2', 'word_temp', 'Thermal & Cooling', 'Secondary / Synchronous rectifier temperature (deg C)'),
    (0x8F, 'READ_TEMPERATURE_3', 'word_temp', 'Thermal & Cooling', 'Ambient / Air intake temperature reading (deg C)'),
    (0x90, 'READ_FAN_SPEED_1', 'word_rpm', 'Thermal & Cooling', 'Measured speed of cooling fan 1 (RPM)'),
    (0x91, 'READ_FAN_SPEED_2', 'word_rpm', 'Thermal & Cooling', 'Measured speed of cooling fan 2 (RPM)'),
    (0x32, 'MAX_DUTY', 'word_percent', 'Regulation & Timing', 'Maximum permitted PWM duty cycle (%)'),
    (0x33, 'FREQUENCY_SWITCH', 'word_khz', 'Regulation & Timing', 'Target switching frequency (kHz)'),
    (0x94, 'READ_DUTY_CYCLE', 'word_percent', 'Regulation & Timing', 'Measured PWM duty cycle (%)'),
    (0x95, 'READ_FREQUENCY', 'word_khz', 'Regulation & Timing', 'Measured switching frequency (kHz)'),
    (0x78, 'STATUS_BYTE', 'byte_hex', 'Status & Diagnostics', 'Summary status byte (critical faults)'),
    (0x79, 'STATUS_WORD', 'word_hex', 'Status & Diagnostics', 'Full 16-bit status word'),
    (0x7A, 'STATUS_VOUT', 'byte_hex', 'Status & Diagnostics', 'Output voltage faults detail'),
    (0x7B, 'STATUS_IOUT', 'byte_hex', 'Status & Diagnostics', 'Output current and power faults detail'),
    (0x7C, 'STATUS_INPUT', 'byte_hex', 'Status & Diagnostics', 'Input voltage and current faults detail'),
    (0x7D, 'STATUS_TEMPERATURE', 'byte_hex', 'Status & Diagnostics', 'Thermal faults detail'),
    (0x7E, 'STATUS_CML', 'byte_hex', 'Status & Diagnostics', 'I2C, CRC/PEC, memory and command errors'),
    (0x80, 'STATUS_MFR_SPECIFIC', 'byte_hex', 'Status & Diagnostics', 'Manufacturer proprietary diagnostic flags'),
    (0x81, 'STATUS_FANS_1_2', 'byte_hex', 'Status & Diagnostics', 'Cooling fans 1 and 2 fault status'),
    (0x98, 'PMBUS_REVISION', 'byte_hex', 'Manufacturer Info', 'Supported PMBus specification revision code'),
    (0x99, 'MFR_ID', 'block_ascii', 'Manufacturer Info', 'Power supply manufacturer identity string'),
    (0x9A, 'MFR_MODEL', 'block_ascii', 'Manufacturer Info', 'Equipment commercial model number string'),
    (0x9B, 'MFR_REVISION', 'block_ascii', 'Manufacturer Info', 'Hardware revision / firmware build string'),
    (0x9C, 'MFR_LOCATION', 'block_ascii', 'Manufacturer Info', 'Manufacturing facility location'),
    (0x9D, 'MFR_DATE', 'block_ascii', 'Manufacturer Info', 'Manufacturing date code (YYMMDD)'),
    (0x9E, 'MFR_SERIAL', 'block_ascii', 'Manufacturer Info', 'Unique device serial number'),
    (0xA0, 'MFR_VIN_MIN', 'word_volt', 'Manufacturer Info', 'Minimum guaranteed input operating voltage (V)'),
    (0xA1, 'MFR_VIN_MAX', 'word_volt', 'Manufacturer Info', 'Maximum guaranteed input operating voltage (V)'),
    (0xA2, 'MFR_IIN_MAX', 'word_curr', 'Manufacturer Info', 'Maximum rated input current (A)'),
    (0xA3, 'MFR_PIN_MAX', 'word_power', 'Manufacturer Info', 'Maximum rated input power (W)'),
    (0xA4, 'MFR_VOUT_MIN', 'word_vout', 'Manufacturer Info', 'Minimum adjustable output voltage (V)'),
    (0xA5, 'MFR_VOUT_MAX', 'word_vout', 'Manufacturer Info', 'Maximum adjustable output voltage (V)'),
    (0xA6, 'MFR_IOUT_MAX', 'word_curr', 'Manufacturer Info', 'Maximum continuous rated output current (A)'),
    (0xA7, 'MFR_POUT_MAX', 'word_power', 'Manufacturer Info', 'Maximum continuous rated output power (W)'),
    (0xA8, 'MFR_TAMBIENT_MAX', 'word_temp', 'Manufacturer Info', 'Maximum rated ambient temperature (deg C)'),
    (0xA9, 'MFR_TAMBIENT_MIN', 'word_temp', 'Manufacturer Info', 'Minimum rated ambient temperature (deg C)'),
]

REG_UNITS = {"word_volt": ("V", 3), "word_curr": ("A", 2), "word_power": ("W", 2), "word_temp": ("°C", 2),
             "word_rpm": ("rpm", 0), "word_mohm": ("mΩ", 3), "word_droop": ("mV/A", 4), "word_v_ms": ("V/ms", 4),
             "word_ms": ("ms", 2), "word_percent": ("%", 1), "word_khz": ("kHz", 1), "word_linear_ratio": ("", 4)}


def decode_register(rtype, raw, vmode):
    """Valeur brute -> texte lisible selon le type du registre."""
    if raw is None:
        return "n/a"
    if rtype == "byte_resp":
        act = ["Ignorer", "Continuer avec délai", "Arrêt jusqu'à effacement", "Nouvelle tentative"][(raw >> 6) & 3]
        return "%s (0x%02X)" % (act, raw)
    if rtype == "byte_vmode":
        exp = vmode & 0x1F
        return "Linear16, exposant %d (0x%02X)" % (exp - 32 if exp > 15 else exp, raw) if raw >> 5 == 0 else "0x%02X" % raw
    if rtype == "byte_op":
        names = {0x80: "ON", 0x00: "OFF immédiat", 0x40: "OFF progressif", 0x98: "ON (marge haute)", 0x94: "ON (marge basse)"}
        return "%s (0x%02X)" % (names.get(raw, "?"), raw)
    if rtype == "byte_cap":
        return "%s0x%02X : PEC %s, %s, SMBALERT# %s" % ("", raw, "oui" if raw & 0x80 else "non",
                                                         "400 kHz" if (raw >> 5) & 3 == 1 else "100 kHz",
                                                         "oui" if raw & 0x10 else "non")
    if rtype == "byte_page":
        return "0x%02X (toutes)" % raw if raw == 0xFF else "0x%02X (page %d)" % (raw, raw)
    if rtype in ("byte_hex", "byte_onoff", "byte_wp"):
        return "0x%02X" % raw
    if rtype == "word_hex":
        return "0x%04X" % raw
    if rtype == "word_vout":
        if vmode is not None and vmode >> 5 == 0:
            return "%.3f V" % decode_linear16(raw, vmode)
        return "0x%04X" % raw
    if rtype in REG_UNITS:
        unit, dec = REG_UNITS[rtype]
        return ("%.*f %s" % (dec, decode_linear11(raw), unit)).strip()
    return "0x%04X" % raw


def read_registers(bus, psu):
    """Lit tous les registres de la table REGISTERS (à la demande, pas dans la boucle de mesure)."""
    old, bus.retries = bus.retries, 1
    try:
        vmode = safe(bus.read_byte_data, psu.addr, CMD_VOUT_MODE)
        out = []
        for code, name, rtype, cat, desc in REGISTERS:
            if rtype == "block_ascii":
                val = read_text(bus, psu.addr, code) or "n/a"
            elif rtype.startswith("byte"):
                val = decode_register(rtype, safe(bus.read_byte_data, psu.addr, code), vmode)
            else:
                val = decode_register(rtype, safe(bus.read_word_data, psu.addr, code), vmode)
            out.append({"code": "0x%02X" % code, "name": name, "cat": cat, "desc": desc, "val": val})
        return out
    finally:
        bus.retries = old


def do_control(bus, psu, action):
    """Actions d'écriture (uniquement avec --control)."""
    a = psu.addr
    if action == "clear_faults":
        try:
            bus.write_byte(a, 0x03)                    # CLEAR_FAULTS (send byte)
        except OSError:
            bus.write_byte_data(a, 0x03, 0x00)
    elif action in OPERATION_MODES:
        bus.write_byte_data(a, CMD_OPERATION, OPERATION_MODES[action])
    else:
        raise ValueError("action inconnue : %s" % action)


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
def psu_id(bus, addr, mux=None, channel=None, page=None):
    """Identifiant stable, utilisable dans une URL : 2-58, 2-m70c1-58, 2-58-p1."""
    pid = "%d-" % bus + ("m%02xc%d-" % (mux, channel) if mux is not None else "") + "%02x" % addr
    return pid + ("-p%d" % page if page is not None else "")


class Psu:
    def __init__(self, name, bus, addr, mux=None, channel=None, page=None):
        self.id = psu_id(bus, addr, mux, channel, page)
        self.name = name or ("B%d" % bus + ("/C%d" % channel if mux is not None else "") + "/0x%02X" % addr)
        self.bus, self.addr = bus, addr
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
    old, bus.retries = bus.retries, 1   # commandes non supportées : inutile d'insister
    try:
        _first_contact(bus, psu)
    finally:
        bus.retries = old


def _first_contact(bus, psu):
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
    res = {"id": psu.id, "name": psu.name, "loc": psu.loc, "t": time.time()}
    try:
        if psu.page is not None:
            bus.write_byte_data(a, CMD_PAGE, psu.page)
        try:
            sw = bus.read_word_data(a, CMD_STATUS_WORD)
        except OSError:   # certains PSU n'ont que STATUS_BYTE ; sinon READ_VOUT prouve qu'ils sont là
            try:
                sw = bus.read_byte_data(a, 0x78)
            except OSError:
                bus.read_word_data(a, 0x8B)
                sw = 0
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
        self.lock = threading.RLock()   # un seul accès à la fois : mesures, registres, actions

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

    def run(self, psu, fn):
        """Exécute fn(bus) sur ce PSU (mux sélectionné, page positionnée), sans gêner les mesures."""
        with self.lock:
            try:
                self._select(psu)
                if psu.page is not None:
                    self.bus.write_byte_data(psu.addr, CMD_PAGE, psu.page)
                return fn(self.bus)
            finally:
                self.deselect()

    def poll(self):
        with self.lock:
            return self._poll()

    def _poll(self):
        out = []
        for psu in self.psus:
            try:
                self._select(psu)
            except OSError as e:
                psu.reset()
                out.append({"id": psu.id, "name": psu.name, "loc": psu.loc, "t": time.time(), "online": False,
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
    old, bus.retries = bus.retries, 0
    try:
        for a in range(0x03, 0x78):
            if safe(bus.read_byte, a) is not None or safe(bus.read_word_data, a, CMD_STATUS_WORD) is not None:
                found.append(a)
    finally:
        bus.retries = old
    return found


def hexl(addrs):
    return " ".join("0x%02X" % a for a in addrs) or "(rien)"


def looks_like_pmbus(bus, addr):
    """Critères repris d'un outil qui marche sur le terrain : STATUS_BYTE lisible (≠ 0xFF),
    puis au moins une autre lecture PMBus cohérente (READ_VOUT / READ_VIN / PMBUS_REVISION / VOUT_MODE)."""
    st = safe(bus.read_byte_data, addr, 0x78)            # STATUS_BYTE
    if st is None:
        st = safe(bus.read_word_data, addr, CMD_STATUS_WORD)
        st = None if st is None else st & 0xFF
    if st is None or st == 0xFF:
        return False
    vout = safe(bus.read_word_data, addr, 0x8B)
    vin = safe(bus.read_word_data, addr, 0x88)
    rev = safe(bus.read_byte_data, addr, CMD_PMBUS_REVISION)
    vmode = safe(bus.read_byte_data, addr, CMD_VOUT_MODE)
    good = [vout is not None and vout not in (0xFFFF, 0x7FFF),
            vin is not None and vin not in (0xFFFF, 0x7FFF),
            rev is not None and (rev >> 4) <= 4 and (rev & 0xF) <= 4,
            vmode is not None and vmode != 0xFF]
    if not any(good):
        return False
    return not (st == 0 and not vout and not vin)        # tout à zéro = périphérique fantôme


def is_mux(bus, addr):
    """PCA954x : l'octet de contrôle écrit se relit à l'identique."""
    try:
        bus.write_byte(addr, 0x01)
        ok = bus.read_byte(addr) & 0x0F == 0x01
        bus.write_byte(addr, 0x00)
        return ok
    except OSError:
        return False


PSU_RANGE = range(0x58, 0x68)   # adresses habituelles des PSU PMBus (EEPROM FRU en 0x50-0x57)


def guess_device(num, addr):
    """Ce que l'adresse a toutes les chances d'être (périphériques I2C courants)."""
    if num == 0:   # bus interne de la BeagleBone Black
        known = {0x24: "PMIC TPS65217 (alimentation de la BeagleBone)", 0x34: "HDMI TDA19988 (CEC)",
                 0x50: "EEPROM d'identification de la BeagleBone", 0x70: "HDMI TDA19988"}
        if addr in known:
            return "systeme", known[addr]
    if 0x50 <= addr <= 0x57:
        return "eeprom", "EEPROM (FRU / identification, souvent celle d'un PSU en 0x%02X)" % (addr + 8)
    ranges = [((0x20, 0x27), "Expandeur d'E/S probable (PCF8574 / MCP23017)"),
              ((0x38, 0x3B), "Expandeur d'E/S probable (PCF8574A)"),
              ((0x3C, 0x3D), "Écran OLED probable (SSD1306)"),
              ((0x40, 0x47), "Capteur courant/tension probable (INA219 / INA226) ou PCA9685"),
              ((0x48, 0x4F), "Capteur de température ou ADC probable (LM75, TMP10x, ADS1115)"),
              ((0x68, 0x69), "Horloge RTC (DS1307 / DS3231) ou centrale inertielle probable"),
              ((0x76, 0x77), "Capteur de pression probable (BMP280 / BME280)")]
    for (lo, hi), label in ranges:
        if lo <= addr <= hi:
            return "autre", label
    return "autre", "Périphérique I2C non identifié"


def scan_one_bus(bus, num, args, ids=None, say=lambda *a: None):
    """Balaye un bus (adresses directes + canaux de chaque mux).
    Renvoie (appareils, psus) ; `ids` (identité -> id PSU) sert à repérer les doublons entre bus."""
    ids = {} if ids is None else ids
    devices, psus = [], []
    allow_mux = args.mux is not None or num != 0      # pas d'écriture sur le bus interne de la BBB
    cands = [args.mux] if args.mux is not None else (
        [a for a in range(0x70, 0x78) if safe(bus.read_byte, a) is not None] if allow_mux else [])
    muxes = [m for m in cands if is_mux(bus, m)]       # remis à 0 par is_mux
    if muxes:
        say("  mux I2C détecté(s) : %s" % hexl(muxes))

    def segment(found, mux=None, ch=None):
        where = "direct" if mux is None else "mux 0x%02X canal %d" % (mux, ch)
        if found:
            say("  %s : %s" % (where, hexl(found)))
        # les adresses PSU habituelles sont sondées même si le balayage ne les a pas vues
        for a in sorted((set(found) | set(PSU_RANGE)) - set(muxes)):
            dev = {"addr": "0x%02X" % a, "a": a, "mux": None if mux is None else "0x%02X" % mux,
                   "ch": ch, "where": where, "psu": None}
            if not (0x50 <= a <= 0x57) and (args.addr is None or a in args.addr) and looks_like_pmbus(bus, a):
                ident = tuple(read_text(bus, a, c) for c in (0x99, 0x9A, 0x9E))
                pid = psu_id(num, a, mux, ch)
                if ident[2] and ids.get(ident, pid) != pid:
                    dev.update(kind="doublon", label="Doublon du PSU %s (même N° de série %s)" % (ids[ident], ident[2]))
                    say("  0x%02X (%s) ignoré : doublon de %s" % (a, where, ids[ident]))
                else:
                    if ident[2]:
                        ids[ident] = pid
                    psu = Psu(None, num, a, mux, ch)
                    psus.append(psu)
                    dev.update(kind="psu", psu=psu.id, label="Alimentation PMBus",
                               detail=" ".join(x for x in ident[:2] if x) + (" · SN " + ident[2] if ident[2] else ""))
                    say("  -> PSU PMBus en 0x%02X (%s) %s" % (a, where, dev["detail"]))
            elif a in found:
                kind, label = guess_device(num, a)
                dev.update(kind=kind, label=label)
            else:
                continue
            devices.append(dev)

    segment(scan_bus(bus))
    for m in muxes:
        devices.append({"addr": "0x%02X" % m, "a": m, "mux": None, "ch": None, "where": "direct",
                        "psu": None, "kind": "mux", "label": "Mux / switch I2C (PCA954x / TCA954x), 8 canaux"})
        for ch in (args.channels if args.mux is not None else range(8)):
            try:
                bus.write_byte(m, 1 << ch)
            except OSError:
                continue
            segment([a for a in scan_bus(bus) if a != m], m, ch)
        safe(bus.write_byte, m, 0)
    devices.sort(key=lambda d: (d["mux"] or "", d["ch"] if d["ch"] is not None else -1, d["a"]))
    return devices, psus


def list_buses():
    return sorted(int(f.split("-")[1]) for f in os.listdir("/dev")
                  if f.startswith("i2c-") and f.split("-")[1].isdigit())


BUS_HINT = {0: "interne (PMIC, EEPROM, HDMI)", 1: "I2C1 : P9_17 SCL / P9_18 SDA", 2: "I2C2 : P9_19 SCL / P9_20 SDA"}


def discover(args, report=True):
    """Balaye les bus I2C (tous sauf 0, ou celui demandé avec -b) et renvoie les PSU trouvés."""
    say = print if report else (lambda *a, **k: None)
    nums = [args.bus] if args.bus_explicit else [n for n in list_buses() if n > 0]
    if not nums:
        say("Aucun /dev/i2c-N (N>0) : active le bus I2C (config-pin / overlay) et vérifie les droits.")
        return []
    psus, ids = [], {}
    for num in nums:
        try:
            bus = I2CBus(num)
        except OSError as e:
            say("Bus %d : ouverture impossible (%s)" % (num, e))
            continue
        try:
            say("Bus %d (pilote %s) :" % (num, bus.backend))
            devices, found = scan_one_bus(bus, num, args, ids, say)
            for d in devices:
                if d["kind"] not in ("psu", "doublon"):
                    say("  0x%02X (%s) : %s" % (d["a"], d["where"], d["label"]))
            psus += found
        finally:
            bus.close()
    say("Détection terminée : %d PSU PMBus trouvé(s)." % len(psus))
    return psus


# Valeurs modifiables (onglet Réglages) : (code, nom, groupe, format, unité)
WRITABLE = [
    (0x21, "VOUT_COMMAND", "Sortie", "vout", "V"), (0x24, "VOUT_MAX", "Sortie", "vout", "V"),
    (0x25, "VOUT_MARGIN_HIGH", "Sortie", "vout", "V"), (0x26, "VOUT_MARGIN_LOW", "Sortie", "vout", "V"),
    (0x35, "VIN_ON", "Entrée", "l11", "V"), (0x36, "VIN_OFF", "Entrée", "l11", "V"),
    (0x55, "VIN_OV_FAULT_LIMIT", "Entrée", "l11", "V"), (0x57, "VIN_OV_WARN_LIMIT", "Entrée", "l11", "V"),
    (0x58, "VIN_UV_WARN_LIMIT", "Entrée", "l11", "V"), (0x59, "VIN_UV_FAULT_LIMIT", "Entrée", "l11", "V"),
    (0x5B, "IIN_OC_FAULT_LIMIT", "Entrée", "l11", "A"), (0x5D, "IIN_OC_WARN_LIMIT", "Entrée", "l11", "A"),
    (0x6B, "PIN_OP_WARN_LIMIT", "Entrée", "l11", "W"),
    (0x40, "VOUT_OV_FAULT_LIMIT", "Protections sortie", "vout", "V"), (0x42, "VOUT_OV_WARN_LIMIT", "Protections sortie", "vout", "V"),
    (0x43, "VOUT_UV_WARN_LIMIT", "Protections sortie", "vout", "V"), (0x44, "VOUT_UV_FAULT_LIMIT", "Protections sortie", "vout", "V"),
    (0x46, "IOUT_OC_FAULT_LIMIT", "Protections sortie", "l11", "A"), (0x4A, "IOUT_OC_WARN_LIMIT", "Protections sortie", "l11", "A"),
    (0x68, "POUT_OP_FAULT_LIMIT", "Protections sortie", "l11", "W"), (0x6A, "POUT_OP_WARN_LIMIT", "Protections sortie", "l11", "W"),
    (0x4F, "OT_FAULT_LIMIT", "Température", "l11", "°C"), (0x51, "OT_WARN_LIMIT", "Température", "l11", "°C"),
    (0x52, "UT_WARN_LIMIT", "Température", "l11", "°C"), (0x53, "UT_FAULT_LIMIT", "Température", "l11", "°C"),
    (0x3B, "FAN_COMMAND_1", "Ventilation & séquencement", "l11", "rpm/%"),
    (0x3C, "FAN_COMMAND_2", "Ventilation & séquencement", "l11", "rpm/%"),
    (0x60, "TON_DELAY", "Ventilation & séquencement", "l11", "ms"), (0x61, "TON_RISE", "Ventilation & séquencement", "l11", "ms"),
    (0x64, "TOFF_DELAY", "Ventilation & séquencement", "l11", "ms"), (0x65, "TOFF_FALL", "Ventilation & séquencement", "l11", "ms"),
]
WRITABLE_BY_CODE = {w[0]: w for w in WRITABLE}
OPERATION_MODES = {"on": 0x80, "off": 0x00, "soft_off": 0x40, "margin_low": 0x94, "margin_high": 0x98}


def read_settings(bus, psu):
    old, bus.retries = bus.retries, 1
    try:
        vmode = safe(bus.read_byte_data, psu.addr, CMD_VOUT_MODE)
        lin16 = vmode is not None and vmode >> 5 == 0
        out = []
        for code, name, group, fmt, unit in WRITABLE:
            raw = safe(bus.read_word_data, psu.addr, code)
            val = None
            if raw is not None:
                val = decode_linear16(raw, vmode) if fmt == "vout" and lin16 else (
                    None if fmt == "vout" else decode_linear11(raw))
            out.append({"code": "0x%02X" % code, "name": name, "group": group, "unit": unit, "value": val})
        op = safe(bus.read_byte_data, psu.addr, CMD_OPERATION)
        wp = safe(bus.read_byte_data, psu.addr, 0x10)
        return {"settings": out, "operation": op, "write_protect": wp, "vout_linear": lin16}
    finally:
        bus.retries = old


def write_setting(bus, psu, code, value):
    """Écrit une valeur et vérifie que le PSU l'a acceptée (STATUS_CML). Renvoie la valeur relue."""
    w = WRITABLE_BY_CODE.get(code)
    if not w:
        raise ValueError("registre 0x%02X non modifiable ici" % code)
    a, fmt = psu.addr, w[3]
    if fmt == "vout":
        vmode = bus.read_byte_data(a, CMD_VOUT_MODE)
        if vmode >> 5 != 0:
            raise ValueError("VOUT_MODE non linéaire : écriture non gérée")
        exp = vmode & 0x1F
        exp = exp - 32 if exp > 15 else exp
        raw = int(round(value / (2.0 ** exp)))
        if not 0 <= raw <= 0xFFFF:
            raise ValueError("valeur hors plage")
    else:
        raw = encode_linear11(value)
    cml_before = safe(bus.read_byte_data, a, CMD_STATUS_CML) or 0
    bus.write_word_data(a, code, raw)
    cml_after = safe(bus.read_byte_data, a, CMD_STATUS_CML) or 0
    if (cml_after & ~cml_before) & 0xC0:
        raise ValueError("refusé par le PSU (STATUS_CML=0x%02X : commande ou donnée invalide, "
                         "ou écriture protégée par WRITE_PROTECT)" % cml_after)
    back = safe(bus.read_word_data, a, code)
    if back is None:
        return None
    return decode_linear16(back, vmode) if fmt == "vout" else decode_linear11(back)


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
                psus.append(Psu(None, args.bus, a, args.mux, ch, pg))
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
    """Dernières mesures, historique et journal de tous les PSU connus (clé : id du PSU)."""

    def __init__(self, interval, history_s):
        self.lock = threading.Lock()
        self.interval = interval
        self.maxlen = max(120, int(history_s / max(interval, 0.2)))
        self.events = collections.deque(maxlen=2000)
        self.seq = 0
        self.meta = {}       # id -> Psu
        self.latest = {}     # id -> dernière mesure
        self.hist = {}       # id -> {clé: deque[(t, v)]}
        self.seen = set()    # PSU déjà vus en ligne : les autres restent invisibles

    def add(self, psus):
        with self.lock:
            for p in psus:
                if p.id not in self.meta:
                    self.meta[p.id] = p
                    self.hist[p.id] = {}

    def _event(self, t, pid, sev, msg):
        self.seq += 1
        p = self.meta.get(pid)
        self.events.append({"id": self.seq, "t": t, "psu": pid, "name": p.name if p else pid,
                            "sev": sev, "msg": msg})

    def log(self, pid, sev, msg):
        with self.lock:
            self._event(time.time(), pid, sev, msg)

    def update(self, results):
        with self.lock:
            for r in results:
                pid, t = r["id"], r["t"]
                if r["online"]:
                    self.seen.add(pid)
                elif pid not in self.seen:      # jamais répondu : on ne l'affiche pas
                    self.latest[pid] = r
                    continue
                prev = self.latest.get(pid)
                was_online = prev.get("online") if prev else None
                if r["online"] is False and was_online is not False:
                    self._event(t, pid, "fault", "Hors ligne : " + r.get("error", ""))
                elif r["online"] and was_online is False:
                    self._event(t, pid, "info", "De nouveau en ligne")
                if r["online"]:
                    old = {(a["reg"], a["bit"]) for a in prev.get("alarms", [])} if prev and was_online else set()
                    new = {(a["reg"], a["bit"]) for a in r["alarms"]}
                    for a in r["alarms"]:
                        if (a["reg"], a["bit"]) not in old:
                            self._event(t, pid, a["sev"], "%s : %s (%s)" % (a["name"], a["desc"], a["reg"]))
                    if prev and was_online:
                        for a in prev["alarms"]:
                            if (a["reg"], a["bit"]) not in new:
                                self._event(t, pid, "info", "Effacé : %s (%s)" % (a["name"], a["reg"]))
                    for key, v in r["values"].items():
                        self.hist[pid].setdefault(key, collections.deque(maxlen=self.maxlen)).append((t, v))
                self.latest[pid] = r

    def get(self, pid):
        with self.lock:
            r = self.latest.get(pid)
            return dict(r) if r and pid in self.seen else None

    def brief(self, pid):
        """Résumé d'un PSU pour les listes (accueil, page bus)."""
        r = self.latest.get(pid) or {}
        info = r.get("info") or {}
        p = self.meta.get(pid)
        return {"id": pid, "name": p.name if p else pid, "loc": p.loc if p else "", "bus": p.bus if p else None,
                "summary": r.get("summary", "pending") if pid in self.seen else "pending",
                "model": info.get("Modèle (MFR_MODEL)", ""), "serial": info.get("N° de série", ""),
                "pout": (r.get("values") or {}).get("pout"), "alarms": len(r.get("alarms") or [])}

    def visible(self, bus=None):
        with self.lock:
            return [self.brief(pid) for pid, p in self.meta.items()
                    if pid in self.seen and (bus is None or p.bus == bus)]

    def history(self, pid, since):
        with self.lock:
            series = self.hist.get(pid, {})
            return {k: [[round(t, 2), v] for t, v in dq if t > since] for k, dq in series.items()}

    def events_since(self, since, pid=None):
        with self.lock:
            return [e for e in self.events if e["id"] > since and (pid is None or e["psu"] == pid)]

    def export_csv(self, pid=None):
        out = io.StringIO()
        w = csv.writer(out)
        w.writerow(["time", "psu", "emplacement"] + TELEMETRY_KEYS)
        with self.lock:
            for i, series in self.hist.items():
                if pid and i != pid:
                    continue
                rows = {}
                for k, dq in series.items():
                    for t, v in dq:
                        rows.setdefault(round(t, 2), {})[k] = v
                p = self.meta[i]
                for t in sorted(rows):
                    r = rows[t]
                    w.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)), p.name, p.loc]
                               + [("%.3f" % r[k]) if k in r else "" for k in TELEMETRY_KEYS])
        return out.getvalue()


METRICS_META = {k: {"label": lab, "unit": u, "dec": d} for k, _c, u, d, lab in TELEMETRY}


class BusManager:
    """Un bus I2C : balayage à la demande, lecture périodique de ses PSU, accès exclusif (verrou)."""

    def __init__(self, num, mon):
        self.num, self.mon = num, mon
        self.lock = threading.RLock()
        self.bus = None
        self.cur = None          # (mux, canal) sélectionné
        self.psus, self.devices = [], []
        self.scanned_at, self.scanning, self.error = None, False, None
        self.thread = None

    def _open(self):
        if self.bus is None:
            self.bus = I2CBus(self.num)

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
        if self.cur and self.bus:
            safe(self.bus.write_byte, self.cur[0], 0)
        self.cur = None

    def add_psus(self, psus):
        with self.lock:
            known = {p.id for p in self.psus}
            new = [p for p in psus if p.id not in known]
            self.psus += new
        self.mon.state.add(new)
        if self.psus and (self.thread is None or not self.thread.is_alive()):
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def scan(self):
        if self.scanning:
            return
        self.scanning = True
        try:
            with self.lock:
                self._open()
                self.deselect()
                devices, psus = scan_one_bus(self.bus, self.num, self.mon.args, self.mon.ids)
                self.devices, self.error, self.scanned_at = devices, None, time.time()
            self.add_psus(psus)
        except OSError as e:
            self.error = "bus inaccessible : %s" % (e.strerror or e)
            self.scanned_at = time.time()
        finally:
            self.scanning = False

    def scan_async(self):
        if not self.scanning:
            threading.Thread(target=self.scan, daemon=True).start()

    def _loop(self):
        stop, args = self.mon.stop, self.mon.args
        while not stop.is_set():
            t0 = time.time()
            try:
                results = []
                with self.lock:
                    self._open()
                    for psu in list(self.psus):
                        try:
                            self._select(psu)
                        except OSError as e:
                            psu.reset()
                            results.append({"id": psu.id, "name": psu.name, "loc": psu.loc, "t": time.time(),
                                            "online": False, "summary": "offline",
                                            "error": "mux 0x%02X : %s" % (psu.mux, e.strerror or e)})
                            continue
                        results.append(read_psu(self.bus, psu))
                    self.deselect()
                self.mon.state.update(results)
                if args.csv:
                    with self.mon.csv_lock:
                        write_csv(args.csv, results)
            except Exception as e:   # un incident sur le bus ne doit pas tuer le thread
                print("bus %d : %s" % (self.num, e), file=sys.stderr)
            stop.wait(max(0.05, args.interval - (time.time() - t0)))

    def run(self, psu, fn):
        """fn(bus) sur ce PSU (mux sélectionné, page positionnée), sans gêner les mesures."""
        with self.lock:
            self._open()
            try:
                self._select(psu)
                if psu.page is not None:
                    self.bus.write_byte_data(psu.addr, CMD_PAGE, psu.page)
                return fn(self.bus)
            finally:
                self.deselect()

    def info(self, full=False):
        st = self.mon.state
        out = {"num": self.num, "hint": BUS_HINT.get(self.num, ""), "scanned_at": self.scanned_at,
               "scanning": self.scanning, "error": self.error, "devices": len(self.devices),
               "psu_list": st.visible(self.num)}
        out["psus"] = len(out["psu_list"])
        if full:
            devs = []
            with st.lock:
                for d in self.devices:
                    d = {k: v for k, v in d.items() if k != "a"}
                    if d.get("psu"):
                        d.update(st.brief(d["psu"]))
                    devs.append(d)
                # PSU connus de ce bus mais absents du balayage (configuration manuelle)
                listed = {d.get("psu") for d in self.devices}
                for p in self.psus:
                    if p.id not in listed and p.id in st.seen:
                        b = st.brief(p.id)
                        b.update(addr="0x%02X" % p.addr, where="direct" if p.mux is None else
                                 "mux 0x%02X canal %d" % (p.mux, p.channel), kind="psu", psu=p.id,
                                 label="Alimentation PMBus (configurée)", detail="")
                        devs.append(b)
            out["device_list"] = devs
        return out

    def close(self):
        with self.lock:
            if self.bus:
                self.deselect()
                self.bus.close()
                self.bus = None


class Monitor:
    def __init__(self, args):
        self.args = args
        self.state = State(args.interval, args.history)
        self.stop = threading.Event()
        self.ids = {}            # identité (fabricant, modèle, série) -> id : repère les doublons
        self.buses = {}
        self.lock = threading.Lock()
        self.csv_lock = threading.Lock()
        self.backend = "simulation" if args.mock else ("smbus2" if _SMBus else "ioctl")

    def available(self):
        if self.args.mock:
            return [self.args.bus]
        nums = list_buses()
        if self.args.bus_explicit and self.args.bus not in nums:
            nums.append(self.args.bus)
        return sorted(nums)

    def bus(self, num):
        with self.lock:
            if num not in self.buses:
                self.buses[num] = BusManager(num, self)
            return self.buses[num]

    def find(self, pid):
        for bm in list(self.buses.values()):
            for p in bm.psus:
                if p.id == pid:
                    return bm, p
        raise KeyError(pid)

    def read_registers(self, pid):
        bm, p = self.find(pid)
        return bm.run(p, lambda bus: read_registers(bus, p))

    def settings(self, pid):
        bm, p = self.find(pid)
        return bm.run(p, lambda bus: read_settings(bus, p))

    def write(self, pid, code, value):
        bm, p = self.find(pid)
        back = bm.run(p, lambda bus: write_setting(bus, p, code, value))
        self.state.log(pid, "warn", "Écriture %s = %g" % (WRITABLE_BY_CODE[code][1], value))
        p.reset()   # limites relues au prochain cycle
        return back

    def control(self, pid, action):
        bm, p = self.find(pid)
        bm.run(p, lambda bus: do_control(bus, p, action))
        self.state.log(pid, "warn", "Action manuelle : %s" % action)
        p.reset()

    def halt(self):
        self.stop.set()
        for bm in list(self.buses.values()):
            if bm.thread:
                bm.thread.join(timeout=self.args.interval + 5)
            bm.close()


def make_handler(mon, auth, control):
    state = mon.state
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

        def _send(self, body, ctype, code=200, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(json.dumps(obj).encode(), "application/json", code)

        def _deny(self):
            self.send_response(401)
            self.send_header("WWW-Authenticate", 'Basic realm="PMBus Monitor"')
            self.send_header("Content-Length", "0")
            self.end_headers()

        def _common(self):
            return {"time": time.time(), "interval": mon.args.interval, "control": control,
                    "mock": bool(mon.args.mock), "backend": mon.backend}

        def _hw(self, fn):
            """Appel matériel à la demande, erreurs traduites en JSON."""
            try:
                return self._json(fn())
            except KeyError:
                return self._json({"error": "PSU inconnu"}, 404)
            except ValueError as e:
                return self._json({"error": str(e)}, 400)
            except OSError as e:
                return self._json({"error": "accès I2C impossible : %s" % (e.strerror or e)}, 502)
            except Exception as e:   # jamais de requête qui meurt sans réponse
                return self._json({"error": "erreur interne : %s" % e}, 500)

        def do_GET(self):
            if not self._authorized():
                return self._deny()
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            arg = lambda k, d="": q.get(k, [d])[0]

            def num(k, default=0.0):
                try:
                    return float(arg(k, default))
                except ValueError:
                    return default

            if u.path in ("/", "/index.html"):
                self._send(page, "text/html; charset=utf-8")
            elif u.path == "/api/buses":
                d = self._common()
                d.update(buses=[mon.bus(n).info() for n in mon.available()], psus=state.visible())
                self._json(d)
            elif u.path == "/api/bus":
                n = int(num("num", -1))
                if n not in mon.available():
                    return self._json({"error": "bus %d introuvable" % n}, 404)
                d = self._common()
                d.update(mon.bus(n).info(full=True))
                self._json(d)
            elif u.path == "/api/psu":
                pid = arg("id")
                if pid not in state.meta:
                    return self._json({"error": "PSU inconnu"}, 404)
                d = self._common()
                p = state.meta[pid]
                d.update(id=pid, bus=p.bus, name=p.name, loc=p.loc, metrics=METRICS_META,
                         data=state.get(pid))
                self._json(d)
            elif u.path == "/api/defs":
                self._send(defs_json, "application/json")
            elif u.path == "/api/history":
                self._json(state.history(arg("id"), num("since")))
            elif u.path == "/api/events":
                self._json(state.events_since(int(num("since")), arg("id") or None))
            elif u.path == "/api/registers":
                self._hw(lambda: {"registers": mon.read_registers(arg("id"))})
            elif u.path == "/api/settings":
                self._hw(lambda: mon.settings(arg("id")))
            elif u.path == "/api/export.csv":
                name = "pmbus_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
                self._send(state.export_csv(arg("id") or None).encode(), "text/csv; charset=utf-8",
                           extra={"Content-Disposition": 'attachment; filename="%s"' % name})
            elif u.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

        def do_POST(self):
            if not self._authorized():
                return self._deny()
            # protection CSRF : seul notre JavaScript envoie cet en-tête (un autre site ne le peut pas sans CORS)
            if self.headers.get("X-Requested-With") != "pmbus-monitor":
                return self._json({"error": "requête refusée"}, 403)
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, TypeError):
                return self._json({"error": "JSON invalide"}, 400)
            path = urllib.parse.urlparse(self.path).path
            if path == "/api/scan":
                try:
                    bnum = int(body.get("bus"))
                except (TypeError, ValueError):
                    return self._json({"error": "bus invalide"}, 400)
                if bnum not in mon.available():
                    return self._json({"error": "bus %d introuvable" % bnum}, 404)
                mon.bus(bnum).scan_async()
                return self._json({"scanning": True})
            if path not in ("/api/action", "/api/write"):
                return self.send_error(404)
            if not control:
                return self._json({"error": "modifications désactivées : relancer avec --auth user:mdp --control"}, 403)
            pid = str(body.get("id", ""))
            if path == "/api/action":
                return self._hw(lambda: (mon.control(pid, str(body.get("action", ""))), {"status": "ok"})[1])
            try:
                code = int_auto(body.get("code"))
                value = float(body.get("value"))
            except (TypeError, ValueError):
                return self._json({"error": "code ou valeur invalide"}, 400)
            self._hw(lambda: {"status": "ok", "value": mon.write(pid, code, value)})

        def log_message(self, *_):
            pass

    return Handler


# --------------------------------------------------------------------------
# Page web (tout en un : HTML + CSS + JS, aucune ressource externe)
# --------------------------------------------------------------------------
HTML_PAGE = r"""<!doctype html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PMBus Monitor</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1c1f24;--mut:#6b7280;--bd:#e5e7eb;--ok:#16a34a;--warn:#d97706;--fault:#dc2626;--off:#6b7280;--acc:#2563eb;--hov:#eef2ff}
@media(prefers-color-scheme:dark){:root{--bg:#111418;--card:#1a1e24;--fg:#e8eaed;--mut:#9aa3af;--bd:#2a3039;--acc:#60a5fa;--hov:#1e2836}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif}
a{color:var(--acc)}
header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--bd)}
.brand{font-weight:700;font-size:17px;color:var(--fg);text-decoration:none}
#crumbs{color:var(--mut);font-size:14px}#crumbs a{color:var(--mut);text-decoration:none}#crumbs a:hover{color:var(--acc)}
.grow{flex:1}#meta{color:var(--mut);font-size:12px}
main{padding:16px;max-width:1400px;margin:0 auto}
h1{font-size:20px;margin:0}h2{font-size:16px;margin:24px 0 10px}
h3{font-size:12px;margin:16px 0 6px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em}
.mut{color:var(--mut);font-size:13px}
.pill{font-size:12px;padding:3px 10px;border-radius:99px;color:#fff;background:var(--ok);white-space:nowrap;display:inline-block}
.pill.warn{background:var(--warn)}.pill.fault{background:var(--fault)}.pill.off,.pill.offline,.pill.pending{background:var(--off)}
.sim{background:var(--warn);color:#fff;font-size:12px;padding:2px 8px;border-radius:6px}
.bar{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:14px}
.btn{background:var(--acc);color:#fff;border:0;border-radius:6px;padding:7px 13px;font:inherit;font-size:14px;cursor:pointer;text-decoration:none;display:inline-block;white-space:nowrap}
.btn.gray{background:var(--card);color:var(--fg);border:1px solid var(--bd)}.btn.red{background:var(--fault)}.btn.green{background:var(--ok)}.btn.amber{background:var(--warn)}
.btn:disabled{opacity:.5;cursor:not-allowed}.btn.sm{padding:4px 10px;font-size:13px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--bd);border-left:4px solid var(--acc);border-radius:10px;padding:14px;color:var(--fg);text-decoration:none;display:block}
.card.ok{border-left-color:var(--ok)}.card.warn{border-left-color:var(--warn)}.card.fault{border-left-color:var(--fault)}.card.off,.card.offline,.card.pending{border-left-color:var(--off)}
a.card:hover{border-color:var(--acc);background:var(--hov)}
.top{display:flex;justify-content:space-between;align-items:center;gap:8px}
.name{font-weight:600;font-size:16px}.loc{color:var(--mut);font-size:12px;margin-top:2px}
.stats{margin-top:10px;font-size:14px}.go{margin-top:10px;color:var(--acc);font-size:13px;text-align:right}
.chips{display:flex;flex-wrap:wrap;gap:4px;margin-top:8px}
.chip{font-size:11px;padding:2px 7px;border-radius:6px;background:var(--warn);color:#fff}
.chip.fault{background:var(--fault)}.chip.info{background:var(--off)}
.panel{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:14px;margin-bottom:14px;overflow-x:auto}
table{border-collapse:collapse;width:100%;font-size:14px}
td,th{padding:7px 8px;border-bottom:1px solid var(--bd);text-align:left;vertical-align:middle}
th{color:var(--mut);font-weight:500;font-size:12px}td.n{text-align:right;font-variant-numeric:tabular-nums}
tr.seg td{background:var(--bg);font-weight:600;font-size:13px;color:var(--mut)}
tr.click{cursor:pointer}tr.click:hover td{background:var(--hov)}
code{font-family:ui-monospace,monospace;font-size:13px}
.kind{font-size:11px;padding:2px 8px;border-radius:6px;border:1px solid var(--bd);white-space:nowrap}
.kind.psu{background:var(--acc);color:#fff;border-color:transparent}.kind.mux{background:#7c3aed;color:#fff;border-color:transparent}
.kind.doublon{background:var(--off);color:#fff;border-color:transparent}
.tabs{display:flex;gap:2px;border-bottom:1px solid var(--bd);margin-bottom:14px;overflow-x:auto}
.tabs a{padding:10px 16px;color:var(--mut);text-decoration:none;border-bottom:3px solid transparent;white-space:nowrap}
.tabs a.on{color:var(--fg);border-color:var(--acc);font-weight:600}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px;margin-bottom:14px}
.tile{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:12px}
.tile .l{color:var(--mut);font-size:12px}.tile .v{font-size:26px;font-weight:600;font-variant-numeric:tabular-nums;margin-top:2px}
.tile .u{font-size:14px;color:var(--mut);font-weight:400;margin-left:3px}
.reg{margin:10px 0}.reg b{font-size:13px}.reg code{color:var(--mut);margin-left:6px}
.bits{display:flex;flex-wrap:wrap;gap:4px;margin-top:5px}
.bit{font-size:11px;padding:2px 7px;border-radius:6px;border:1px solid var(--bd);color:var(--mut)}
.bit.set{color:#fff;border-color:transparent;background:var(--warn)}.bit.set.fault{background:var(--fault)}.bit.set.info{background:var(--off)}
.ok-line{color:var(--ok)}.err{color:var(--fault)}
.note{background:var(--hov);border-radius:8px;padding:10px 12px;font-size:14px;margin-bottom:12px}
select,input{font:inherit;color:var(--fg);background:var(--card);border:1px solid var(--bd);border-radius:6px;padding:5px 8px}
input[type=number]{width:120px}
#charts{display:grid;grid-template-columns:repeat(auto-fill,minmax(420px,1fr));gap:12px}
@media(max-width:520px){#charts{grid-template-columns:1fr}.tile .v{font-size:22px}}
.ch{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:10px}
.ch h3{margin:0 0 4px}.ch canvas{width:100%;height:190px;display:block}
.empty{color:var(--mut);padding:24px;text-align:center}
.res{font-size:13px}.res.ok{color:var(--ok)}.res.bad{color:var(--fault)}
</style></head><body>
<header><a class="brand" href="#/">PMBus Monitor</a><nav id="crumbs"></nav><span class="grow"></span><span id="sim"></span><span id="meta"></span></header>
<main id="view"></main>
<script>
const SEV={ok:"OK",warn:"Avertissement",fault:"Défaut",off:"Éteint",offline:"Hors ligne",pending:"…",info:"Info"};
const KIND={psu:"Alimentation",mux:"Mux I2C",eeprom:"EEPROM",systeme:"Système BBB",autre:"Autre",doublon:"Doublon"};
const TABS=[["mesures","Mesures"],["graphiques","Graphiques"],["erreurs","Erreurs"],["reglages","Réglages"],["registres","Registres"]];
const $=id=>document.getElementById(id);
const esc=s=>String(s==null?"":s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const fmt=(v,d)=>v==null?"–":v.toFixed(d);
const hex=(v,n)=>"0x"+v.toString(16).toUpperCase().padStart(n,"0");
const hm=t=>new Date(t*1000).toLocaleTimeString();
let tok=0,timer=null,defs=null,cur=null,lastCommon={interval:2};

async function api(url){const r=await fetch(url,{cache:"no-store"});if(r.status===401){location.reload();throw new Error("auth")}
  const j=await r.json();if(!r.ok)throw new Error(j.error||("HTTP "+r.status));return j}
async function post(url,body){
  const r=await fetch(url,{method:"POST",headers:{"Content-Type":"application/json","X-Requested-With":"pmbus-monitor"},body:JSON.stringify(body||{})});
  const j=await r.json().catch(()=>({}));if(!r.ok)throw new Error(j.error||("HTTP "+r.status));return j}
function crumbs(list){$("crumbs").innerHTML=list.map(([t,h])=>h?`<a href="${h}">${esc(t)}</a>`:`<span>${esc(t)}</span>`).join(" › ")}
function common(j){lastCommon=j;$("sim").innerHTML=j.mock?'<span class="sim">SIMULATION</span>':"";$("meta").textContent="maj "+hm(j.time)}
/* boucle de rafraîchissement liée à la page affichée */
function loop(fn,ms){
  const my=++tok;clearTimeout(timer);
  const tick=async()=>{if(my!==tok)return;try{await fn()}catch(e){if(e.message!=="auth")$("meta").textContent="erreur : "+e.message}
    if(my===tok)timer=setTimeout(tick,typeof ms==="function"?ms():ms)};
  tick();
}
window.onhashchange=route;
function route(){
  const r=location.hash.replace(/^#\/?/,"").split("/").filter(Boolean);
  window.scrollTo(0,0);
  if(r[0]==="bus")busPage(+r[1]);else if(r[0]==="psu")psuPage(decodeURIComponent(r[1]),r[2]||"mesures");else home();
}

/* ================= Accueil : choix du bus ================= */
function psuCard(p){
  return `<a class="card ${p.summary}" href="#/psu/${encodeURIComponent(p.id)}"><div class="top"><span class="name">${esc(p.model||p.name)}</span><span class="pill ${p.summary}">${SEV[p.summary]}</span></div>
  <div class="loc">${esc(p.loc)}${p.serial?" · SN "+esc(p.serial):""}</div>
  <div class="stats">${p.pout!=null?"POUT "+p.pout.toFixed(0)+" W":""}${p.alarms?` · <span class="err">${p.alarms} alarme(s)</span>`:""}</div><div class="go">Ouvrir ›</div></a>`;
}
function home(){
  cur=null;crumbs([["Accueil"]]);
  $("view").innerHTML=`<div class="bar"><h1>Choisis un bus I2C</h1></div><div id="buses" class="grid"></div>
   <h2>PSU détectés</h2><div id="quick" class="grid"></div>`;
  loop(async()=>{
    const j=await api("api/buses");common(j);
    $("buses").innerHTML=j.buses.map(b=>{
      const st=b.scanning?'<span class="pill pending">balayage…</span>':b.error?'<span class="pill fault">erreur</span>':"";
      const txt=b.scanning?"balayage en cours…":b.error?esc(b.error):b.scanned_at?`${b.devices} appareil(s) · ${b.psus} PSU`:"pas encore balayé — ouvre-le pour lancer la détection";
      return `<a class="card" href="#/bus/${b.num}"><div class="top"><span class="name">Bus I2C ${b.num}</span>${st}</div>
       <div class="loc">/dev/i2c-${b.num}${b.hint?" · "+esc(b.hint):""}</div><div class="stats">${txt}</div>
       <div class="chips">${b.psu_list.map(p=>`<span class="pill ${p.summary}">${esc(p.model||p.name)}</span>`).join("")}</div><div class="go">Ouvrir ›</div></a>`;
    }).join("")||'<div class="empty">Aucun bus I2C trouvé (/dev/i2c-*). Active le bus (config-pin / overlay).</div>';
    $("quick").innerHTML=j.psus.length?j.psus.map(psuCard).join(""):'<div class="empty">Aucun PSU détecté pour l\'instant. Ouvre un bus pour voir ce qui y est branché.</div>';
  },3000);
}

/* ================= Page d'un bus : liste des adresses ================= */
function busPage(n){
  cur=null;crumbs([["Accueil","#/"],["Bus "+n]]);
  $("view").innerHTML=`<div class="bar"><a class="btn gray" href="#/">← Accueil</a><h1>Bus I2C ${n}</h1><span id="hint" class="mut"></span>
    <span class="grow"></span><span id="scaninfo" class="mut"></span><button class="btn" id="rescan">↻ Rescanner</button></div>
    <div class="panel"><table id="devs"><tr><td class="empty">Chargement…</td></tr></table></div>
    <p class="mut">Seules les alimentations PMBus sont réellement interrogées ; les autres descriptions (« probable ») sont déduites de l'adresse.
    Clique sur une alimentation pour ouvrir sa page.</p>`;
  let asked=false;
  $("rescan").onclick=async()=>{try{await post("api/scan",{bus:n});$("scaninfo").textContent="balayage en cours…";$("rescan").disabled=true}catch(e){alert(e.message)}};
  loop(async()=>{
    const b=await api("api/bus?num="+n);common(b);
    $("hint").textContent=b.hint?"/dev/i2c-"+n+" · "+b.hint:"/dev/i2c-"+n;
    if(!b.scanned_at&&!b.scanning&&!asked){asked=true;await post("api/scan",{bus:n});b.scanning=true}
    $("scaninfo").textContent=b.scanning?"balayage en cours…":b.error?b.error:b.scanned_at?"balayé à "+hm(b.scanned_at):"";
    $("rescan").disabled=b.scanning;
    const devs=b.device_list;
    if(!devs.length){$("devs").innerHTML=`<tr><td class="empty">${b.scanning?"Balayage du bus en cours (quelques secondes)…":b.error?esc(b.error):"Aucun appareil ne répond sur ce bus. Vérifie le câblage (SDA, SCL, masse) et les tirages."}</td></tr>`;return}
    let h=`<tr><th>Adresse</th><th>Type</th><th>Description</th><th>État</th><th></th></tr>`,seg=null;
    for(const d of devs){
      if(d.where!==seg){seg=d.where;h+=`<tr class="seg"><td colspan="5">${seg==="direct"?"Directement sur le bus":esc(seg[0].toUpperCase()+seg.slice(1))}</td></tr>`}
      const isPsu=d.kind==="psu";
      h+=`<tr class="${isPsu?"click":""}" ${isPsu?`data-id="${esc(d.psu)}"`:""}><td><code>${d.addr}</code></td><td><span class="kind ${d.kind}">${KIND[d.kind]||d.kind}</span></td>
        <td>${esc(d.label)}${d.detail?`<div class="loc">${esc(d.detail)}</div>`:""}</td>
        <td>${isPsu?`<span class="pill ${d.summary}">${SEV[d.summary]}</span>`:""}</td><td>${isPsu?'<span class="btn sm">Ouvrir ›</span>':""}</td></tr>`;
    }
    $("devs").innerHTML=h;
    document.querySelectorAll("#devs tr[data-id]").forEach(tr=>tr.onclick=()=>location.hash="#/psu/"+encodeURIComponent(tr.dataset.id));
  },()=>2000);
}

/* ================= Page d'un PSU ================= */
async function psuPage(id,tab){
  if(!cur||cur.id!==id)cur={id,hist:{},histSince:0,events:[],evId:0,regRows:null,settings:null,chartKey:"",j:null};
  cur.tab=tab;
  if(!defs)defs=await api("api/defs").catch(()=>[]);
  let j;
  try{j=await api("api/psu?id="+encodeURIComponent(id))}catch(e){
    crumbs([["Accueil","#/"],["PSU"]]);$("view").innerHTML=`<div class="bar"><a class="btn gray" href="#/">← Accueil</a></div><div class="empty">${esc(e.message)}</div>`;return}
  cur.j=j;common(j);
  const back="#/bus/"+j.bus;
  crumbs([["Accueil","#/"],["Bus "+j.bus,back],[j.name]]);
  $("view").innerHTML=`<div class="bar"><a class="btn gray" href="${back}">← Bus ${j.bus}</a>
     <div><h1 id="ptitle">${esc(j.name)}</h1><div class="loc" id="psub">${esc(j.loc)}</div></div><span class="grow"></span><span id="pstate"></span></div>
    <nav class="tabs">${TABS.map(([k,t])=>`<a href="#/psu/${encodeURIComponent(id)}/${k}" class="${k===tab?"on":""}">${t}</a>`).join("")}</nav>
    <div id="tabc"></div>`;
  const once={reglages:showSettings,registres:showRegisters}[tab];
  if(once)once();
  if(tab==="graphiques")initCharts();
  loop(async()=>{
    const j=await api("api/psu?id="+encodeURIComponent(id));cur.j=j;common(j);
    const d=j.data,info=(d&&d.info)||{};
    $("ptitle").textContent=info["Modèle (MFR_MODEL)"]?(info["Fabricant (MFR_ID)"]?info["Fabricant (MFR_ID)"]+" ":"")+info["Modèle (MFR_MODEL)"]:j.name;
    $("psub").textContent=j.loc+(info["N° de série"]?" · SN "+info["N° de série"]:"");
    $("pstate").innerHTML=d?`<span class="pill ${d.summary}">${SEV[d.summary]}</span>`:'<span class="pill pending">lecture…</span>';
    const ev=await api(`api/events?id=${encodeURIComponent(id)}&since=${cur.evId}`);
    for(const e of ev){cur.events.push(e);cur.evId=Math.max(cur.evId,e.id)}
    if(tab==="mesures")showMeasures(j);
    if(tab==="erreurs")showErrors(j);
    if(tab==="graphiques")await updateCharts();
  },()=>Math.max(1000,lastCommon.interval*1000));
}
function offlineMsg(d){return d?`<div class="note err">Hors ligne : ${esc(d.error||"")}</div>`:'<div class="empty">Première lecture en cours…</div>'}

/* ---- Onglet Mesures ---- */
function showMeasures(j){
  const d=j.data;if(!d||!d.online){$("tabc").innerHTML=offlineMsg(d);return}
  const M=j.metrics,v=d.values;
  const tile=(l,val,u)=>`<div class="tile"><div class="l">${l}</div><div class="v">${val}<span class="u">${u}</span></div></div>`;
  let t="";
  for(const k of Object.keys(M))if(k in v)t+=tile(M[k].label,fmt(v[k],M[k].dec),M[k].unit);
  if(v.pin>5&&v.pout!=null)t+=tile("Rendement (POUT / PIN)",(v.pout/v.pin*100).toFixed(1),"%");
  if(d.operation!=null)t+=tile("OPERATION",d.operation&0x80?"ON":"OFF",hex(d.operation,2));
  let h=`<div class="tiles">${t}</div>`;
  const inf=Object.entries(d.info||{});
  if(inf.length)h+=`<div class="panel"><h3 style="margin-top:0">Identité</h3><table>`+inf.map(([k,x])=>`<tr><td>${esc(k)}</td><td>${esc(x)}</td></tr>`).join("")+`</table></div>`;
  if(d.limits&&d.limits.length){
    const g={};d.limits.forEach(l=>(g[l.group]=g[l.group]||[]).push(l));
    h+=`<div class="panel"><h3 style="margin-top:0">Limites et valeurs nominales (lecture)</h3>`+Object.entries(g).map(([n,ls])=>`<h3>${esc(n)}</h3><table>`+
      ls.map(l=>`<tr><td>${l.label}</td><td><code>${hex(l.code,2)}</code></td><td class="n">${l.value.toFixed(3)} ${l.unit}</td></tr>`).join("")+`</table>`).join("")+`</div>`;
  }
  $("tabc").innerHTML=h;
}

/* ---- Onglet Erreurs ---- */
function bitsHtml(reg,val){
  const d=(defs||[]).find(x=>x.reg===reg);if(!d)return "";
  return d.bits.map(([b,n,s,desc])=>`<span class="bit ${val&(1<<b)?"set "+s:""}" title="${esc(desc)}">${b} · ${esc(n)}</span>`).join("");
}
function showErrors(j){
  const d=j.data;let h="";
  if(j.control)h+=`<div class="bar"><button class="btn gray" onclick="act('clear_faults')">Effacer les défauts (CLEAR_FAULTS)</button></div>`;
  if(!d||!d.online)h+=offlineMsg(d);
  else{
    h+=`<div class="panel"><h3 style="margin-top:0">Alarmes actives</h3>`;
    const flags=d.word_flags.filter(f=>f.name!=="OFF");
    if(!d.alarms.length&&!flags.length)h+=`<div class="ok-line">Aucune alarme</div>`;
    else if(!d.alarms.length)h+=`<div>Pas de détail dans les registres STATUS_* ; résumé STATUS_WORD : ${flags.map(f=>`<span class="chip ${f.sev}" title="${esc(f.desc)}">${esc(f.name)}</span>`).join(" ")}</div>`;
    else h+=`<table><tr><th>Gravité</th><th>Code</th><th>Registre / bit</th><th>Description</th></tr>`+
      d.alarms.map(a=>`<tr><td><span class="chip ${a.sev}">${SEV[a.sev]}</span></td><td><b>${esc(a.name)}</b></td><td>${a.reg} bit ${a.bit}</td><td>${esc(a.desc)}</td></tr>`).join("")+`</table>`;
    h+=`</div><div class="panel"><h3 style="margin-top:0">Registres de statut (survole un bit pour sa signification)</h3>`;
    h+=`<div class="reg"><b>STATUS_WORD</b><code>${hex(d.status_word,4)}</code><div class="bits">${bitsHtml("STATUS_WORD",d.status_word)}</div></div>`;
    for(const [r,v] of Object.entries(d.regs))h+=`<div class="reg"><b>${r}</b><code>${hex(v,2)}</code><div class="bits">${bitsHtml(r,v)}</div></div>`;
    h+=`</div>`;
  }
  h+=`<div class="panel"><h3 style="margin-top:0">Journal de ce PSU</h3><table><tr><th>Heure</th><th>Gravité</th><th>Événement</th></tr>`+
    (cur.events.length?cur.events.slice().reverse().map(e=>`<tr><td>${new Date(e.t*1000).toLocaleString()}</td><td><span class="chip ${e.sev}">${SEV[e.sev]}</span></td><td>${esc(e.msg)}</td></tr>`).join(""):
    `<tr><td colspan="3" class="empty">Aucun événement depuis le démarrage</td></tr>`)+`</table></div>`;
  $("tabc").innerHTML=h;
}
async function act(action,label){
  const txt={clear_faults:"Effacer les défauts mémorisés",on:"Mettre en MARCHE (OPERATION = 0x80)",off:"COUPER la sortie immédiatement (OPERATION = 0x00)",
    soft_off:"Arrêt progressif (OPERATION = 0x40)",margin_low:"Marge basse (OPERATION = 0x94)",margin_high:"Marge haute (OPERATION = 0x98)"}[action];
  if(!confirm(txt+" ?"))return;
  try{await post("api/action",{id:cur.id,action});if(cur.tab==="reglages")showSettings()}catch(e){alert("Échec : "+e.message)}
}

/* ---- Onglet Graphiques ---- */
function initCharts(){
  $("tabc").innerHTML=`<div class="bar"><label>Période : <select id="range"><option value="300">5 min</option><option value="900">15 min</option>
    <option value="1800">30 min</option><option value="3600" selected>1 h</option></select></label><span class="grow"></span>
    <a class="btn gray" href="api/export.csv?id=${encodeURIComponent(cur.id)}">⬇ Exporter CSV</a></div><div id="charts"><div class="empty">Chargement…</div></div>`;
  $("range").onchange=paintAll;cur.chartKey="";
}
async function updateCharts(){
  const h=await api(`api/history?id=${encodeURIComponent(cur.id)}&since=${cur.histSince}`);
  for(const [k,pts] of Object.entries(h)){const a=cur.hist[k]=cur.hist[k]||[];
    for(const p of pts){a.push(p);if(p[0]>cur.histSince)cur.histSince=p[0]}if(a.length>4000)a.splice(0,a.length-4000)}
  const M=cur.j.metrics,keys=Object.keys(M).filter(k=>cur.hist[k]&&cur.hist[k].length);
  if(keys.includes("pin")&&keys.includes("pout"))keys.push("eff");
  if(keys.join()!==cur.chartKey){
    cur.chartKey=keys.join();
    $("charts").innerHTML=keys.length?keys.map(k=>{const m=k==="eff"?{label:"Rendement (POUT / PIN)",unit:"%"}:M[k];
      return `<div class="ch"><h3>${m.label} <span style="text-transform:none">(${m.unit})</span></h3><canvas data-k="${k}"></canvas></div>`}).join(""):'<div class="empty">Pas encore de mesures…</div>';
    document.querySelectorAll("#charts canvas").forEach(cv=>{cv.onmousemove=e=>{cv._hx=e.offsetX;paint(cv)};cv.onmouseleave=()=>{cv._hx=null;paint(cv)}});
  }
  paintAll();
}
function paintAll(){document.querySelectorAll("#charts canvas").forEach(paint)}
function series(k){
  if(k!=="eff")return cur.hist[k]||[];
  const m=new Map(cur.hist.pout||[]);return (cur.hist.pin||[]).filter(p=>p[1]>5&&m.has(p[0])).map(p=>[p[0],m.get(p[0])/p[1]*100]);
}
function paint(cv){
  const k=cv.dataset.k,M=cur.j.metrics,dec=k==="eff"?1:M[k].dec,unit=k==="eff"?"%":M[k].unit,pts=series(k);
  const t1=cur.j.time;let t0=t1-parseInt($("range").value);
  if(pts.length&&pts[0][0]>t0)t0=Math.min(pts[0][0],t1-30);
  const col=getComputedStyle(document.documentElement).getPropertyValue("--acc");
  drawChart(cv,[{name:k==="eff"?"Rendement":M[k].label,color:col,pts}],unit,Math.min(dec+1,3),t0,t1,lastCommon.interval*3);
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
      if(best&&Math.abs(best[0]-t)<gap*2){lines.push([s.color,best[1].toFixed(dec)+" "+unit]);tt=best[0]}}
    if(lines.length){
      const head=new Date(tt*1000).toLocaleTimeString(),bw=Math.max(c.measureText(head).width,...lines.map(l=>c.measureText(l[1]).width))+26,bh=16*(lines.length+1)+6;
      let bx=cv._hx+10;if(bx+bw>w-R)bx=cv._hx-bw-10;
      c.fillStyle=card;c.strokeStyle=bd;c.fillRect(bx,T+4,bw,bh);c.strokeRect(bx,T+4,bw,bh);
      c.textAlign="left";c.fillStyle=mut;c.fillText(head,bx+8,T+18);
      lines.forEach((l,i)=>{c.fillStyle=l[0];c.fillRect(bx+8,T+25+i*16,8,8);c.fillStyle=fg;c.fillText(l[1],bx+22,T+33+i*16)});
    }
  }
}

/* ---- Onglet Réglages ---- */
async function showSettings(){
  const ctl=lastCommon.control||(cur.j&&cur.j.control);
  $("tabc").innerHTML='<div class="empty">Lecture des valeurs modifiables…</div>';
  let s;
  try{s=await api("api/settings?id="+encodeURIComponent(cur.id))}catch(e){$("tabc").innerHTML=`<div class="note err">${esc(e.message)}</div>`;return}
  if(cur.tab!=="reglages")return;
  let h=ctl?"":`<div class="note">Lecture seule. Pour modifier les valeurs, relance le script avec <code>--auth utilisateur:motdepasse --control</code>.</div>`;
  if(s.write_protect)h+=`<div class="note err">WRITE_PROTECT = ${hex(s.write_protect,2)} : le PSU risque de refuser les écritures.</div>`;
  h+=`<div class="panel"><h3 style="margin-top:0">Commandes</h3><div class="bar" style="margin:0">
    <span>OPERATION : <b>${s.operation==null?"n/a":hex(s.operation,2)+(s.operation&0x80?" (ON)":" (OFF)")}</b></span>
    <button class="btn green" ${ctl?"":"disabled"} onclick="act('on')">ON</button>
    <button class="btn red" ${ctl?"":"disabled"} onclick="act('off')">OFF</button>
    <button class="btn gray" ${ctl?"":"disabled"} onclick="act('soft_off')">Arrêt progressif</button>
    <button class="btn amber" ${ctl?"":"disabled"} onclick="act('margin_low')">Marge basse</button>
    <button class="btn amber" ${ctl?"":"disabled"} onclick="act('margin_high')">Marge haute</button>
    <button class="btn gray" ${ctl?"":"disabled"} onclick="act('clear_faults')">Effacer les défauts</button>
    <span class="grow"></span><button class="btn gray" onclick="showSettings()">↻ Relire</button></div></div>`;
  const g={};s.settings.forEach(x=>(g[x.group]=g[x.group]||[]).push(x));
  for(const [name,rows] of Object.entries(g)){
    h+=`<div class="panel"><h3 style="margin-top:0">${esc(name)}</h3><table><tr><th>Registre</th><th>Code</th><th class="n">Valeur actuelle</th><th>Nouvelle valeur</th><th></th><th></th></tr>`;
    for(const x of rows){
      const sup=x.value!=null,dis=!ctl||!sup?"disabled":"";
      h+=`<tr><td><b>${x.name}</b></td><td><code>${x.code}</code></td><td class="n" id="v${x.code}">${sup?x.value.toFixed(3)+" "+x.unit:'<span class="mut">non supporté</span>'}</td>
        <td><input type="number" step="any" id="i${x.code}" ${dis} placeholder="${esc(x.unit)}"></td>
        <td><button class="btn sm" ${dis} onclick="writeReg('${x.code}','${x.name}','${esc(x.unit)}')">Appliquer</button></td><td class="res" id="r${x.code}"></td></tr>`;
    }
    h+=`</table></div>`;
  }
  $("tabc").innerHTML=h;
}
async function writeReg(code,name,unit){
  const v=parseFloat($("i"+code).value);
  if(isNaN(v)){$("r"+code).textContent="valeur invalide";$("r"+code).className="res bad";return}
  if(!confirm(`Écrire ${v} ${unit} dans ${name} (${code}) ?\nUne mauvaise valeur peut couper ou endommager l'alimentation.`))return;
  $("r"+code).textContent="écriture…";$("r"+code).className="res";
  try{const r=await post("api/write",{id:cur.id,code,value:v});
    $("v"+code).textContent=r.value==null?"?":r.value.toFixed(3)+" "+unit;$("r"+code).textContent="✓ écrit";$("r"+code).className="res ok";$("i"+code).value=""}
  catch(e){$("r"+code).textContent="✗ "+e.message;$("r"+code).className="res bad"}
}

/* ---- Onglet Registres ---- */
function showRegisters(){
  $("tabc").innerHTML=`<div class="bar"><button class="btn" id="rread">Lire tous les registres</button>
    <input type="search" id="rfilter" placeholder="filtrer (nom, code, catégorie)…"><span id="rstat" class="mut"></span></div>
    <div id="regs" class="panel"><div class="empty">« Lire tous les registres » interroge ~105 registres PMBus (quelques secondes).</div></div>`;
  $("rfilter").oninput=renderRegs;
  $("rread").onclick=async()=>{
    $("rread").disabled=true;$("rstat").textContent="lecture en cours…";
    try{const j=await api("api/registers?id="+encodeURIComponent(cur.id));cur.regRows=j.registers;renderRegs();
      $("rstat").textContent=cur.regRows.filter(x=>x.val!=="n/a").length+" / "+cur.regRows.length+" registres supportés — "+new Date().toLocaleTimeString()}
    catch(e){$("rstat").textContent="erreur : "+e.message}
    $("rread").disabled=false;
  };
  if(cur.regRows)renderRegs();
}
function renderRegs(){
  if(!cur.regRows)return;
  const f=$("rfilter").value.toLowerCase(),rows=cur.regRows.filter(r=>!f||(r.name+r.code+r.cat+r.desc).toLowerCase().includes(f));
  const g={};rows.forEach(r=>(g[r.cat]=g[r.cat]||[]).push(r));
  $("regs").innerHTML=Object.entries(g).map(([c,rs])=>`<h3>${esc(c)}</h3><table><tr><th>Code</th><th>Registre</th><th>Valeur</th><th>Description</th></tr>`+
    rs.map(r=>`<tr><td><code>${r.code}</code></td><td><b>${esc(r.name)}</b></td><td>${esc(r.val)}</td><td class="mut">${esc(r.desc)}</td></tr>`).join("")+`</table>`).join("")||'<div class="empty">Aucun résultat</div>';
}
route();
</script></body></html>
"""


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


def print_short_banner(scheme, port, args):
    ips = [args.host] if args.host != "0.0.0.0" else (local_ips() or ["<ip-de-la-BBB>"])
    print("PMBus monitor - interface web :")
    for ip in ips:
        print("  %s://%s:%d" % (scheme, ip, port), flush=True)


def run_web(args):
    mon = Monitor(args)
    manual = bool(args.config or args.pages) or (args.addr is not None and not args.autodetect)
    if manual:   # PSU décrits par --config / --addr : pas de balayage automatique
        psus = psus_from_config(args.config, args.bus)[0] if args.config else psus_from_args(args)
        unique_names(psus)
        for b in sorted({p.bus for p in psus}):
            mon.bus(b).add_psus([p for p in psus if p.bus == b])
        auto = []
    else:        # balayage en tâche de fond des bus (tous sauf 0, ou celui de -b)
        auto = [args.bus] if args.bus_explicit else [n for n in mon.available() if n > 0]
        for n in auto:
            mon.bus(n).scan_async()

    port = args.port or (8443 if args.https else 8080)
    server = Server((args.host, port), make_handler(mon, args.auth, args.control))
    scheme = "http"
    if args.https:
        if args.cert and args.key:
            crt, key = args.cert, args.key
        else:
            crt, key = ensure_cert(args.certdir, args.regen_cert)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        ctx.load_cert_chain(crt, key)
        server.ctx = ctx
        scheme = "https"

    def auto_rescan():   # tant qu'aucun PSU n'est vu, on rebalaye toutes les 30 s (branchement à chaud)
        while not mon.stop.wait(30):
            if auto and not mon.state.seen:
                for n in auto:
                    mon.bus(n).scan_async()

    threading.Thread(target=auto_rescan, daemon=True).start()
    print_short_banner(scheme, port, args)
    if args.verbose:
        print("Pilote I2C : %s ; bus balayés : %s" % (mon.backend, ", ".join(map(str, auto)) or "aucun (config)"),
              flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        mon.halt()


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
    p.add_argument("--mock", action="store_true", help="simulation : 3 PSU fictifs sur le bus 2 (aucun matériel)")
    w = p.add_argument_group("interface web")
    p.add_argument("--terminal", action="store_true", help="affichage terminal au lieu de l'interface web")
    w.add_argument("--web", action="store_true", help="(défaut) servir l'interface web, HTTPS par défaut")
    w.add_argument("--port", type=int, help="port (défaut 8080 en HTTP, 8443 en HTTPS)")
    w.add_argument("--host", default="0.0.0.0", help="adresse d'écoute (défaut toutes)")
    w.add_argument("--https", action="store_true", help="servir en HTTPS (certificat auto-signé) au lieu de HTTP")
    w.add_argument("--verbose", action="store_true", help="afficher le détail (détection, état des PSU) dans le terminal")
    w.add_argument("--certdir", default=os.path.expanduser("~/.pmbus_monitor"),
                   help="dossier du certificat auto-signé (défaut ~/.pmbus_monitor)")
    w.add_argument("--cert", help="certificat PEM à utiliser (avec --key)")
    w.add_argument("--key", help="clé privée PEM (avec --cert)")
    w.add_argument("--regen-cert", action="store_true", help="régénérer le certificat auto-signé")
    w.add_argument("--auth", metavar="USER:MOT_DE_PASSE", default=os.environ.get("PMBUS_AUTH"),
                   help="protéger la page par mot de passe (ou variable PMBUS_AUTH)")
    w.add_argument("--control", action="store_true",
                   help="autoriser les actions d'écriture depuis la page (effacer défauts, ON/OFF) ; exige --auth")
    w.add_argument("--history", type=int, default=3600, help="durée d'historique gardée en mémoire, en s (défaut 3600)")
    args = p.parse_args()
    args.channels = parse_channels(args.channels)
    args.bus_explicit = args.bus is not None
    if args.bus is None:
        args.bus = 2
    # L'interface web est le mode par défaut ; --once / --json / --terminal donnent le mode terminal.
    args.web = not (args.terminal or args.once or args.json)

    if args.mock:
        global I2CBus
        I2CBus = MockBus
        args.bus_explicit = True   # le simulateur n'existe que sur le bus choisi (2 par défaut)

    if args.scan:
        discover(args)
        return

    if args.control and not (args.auth and args.web):
        sys.exit("--control modifie le matériel : il exige --auth utilisateur:motdepasse (et l'interface web).")

    cfg_interval = None
    if args.config:
        try:
            cfg_interval = psus_from_config(args.config, args.bus)[1]
        except (OSError, ValueError, KeyError) as e:
            sys.exit("Configuration impossible : %s" % e)
    args.interval = args.interval or cfg_interval or 2.0
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    if args.web:
        run_web(args)
        return

    def build(report=True):
        if args.config:
            return psus_from_config(args.config, args.bus)[0]
        if args.autodetect or (args.addr is None and not args.pages):
            return discover(args, report and args.verbose)
        return psus_from_args(args)

    try:
        psus = build()
    except (OSError, ValueError, KeyError) as e:
        sys.exit("Configuration impossible : %s" % e)
    if not psus:
        sys.exit("Aucun PSU trouvé. Lance --scan pour voir ce qui répond sur les bus.")

    unique_names(psus)
    if args.verbose:
        print("%d PSU : %s" % (len(psus), ", ".join("%s (%s)" % (x.name, x.loc) for x in psus)))
    workers = []
    try:
        for b in sorted({x.bus for x in psus}):
            workers.append(BusWorker(b, [x for x in psus if x.bus == b]))
        run_terminal(workers, args)
    except OSError as e:
        sys.exit("Impossible d'ouvrir le bus I2C : %s\nVérifie que le bus est activé et les droits." % e)
    except KeyboardInterrupt:
        pass
    finally:
        for w in workers:
            w.close()


if __name__ == "__main__":
    main()
