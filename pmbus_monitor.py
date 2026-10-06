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
    elif action == "on":
        bus.write_byte_data(a, CMD_OPERATION, 0x80)
    elif action == "off":
        bus.write_byte_data(a, CMD_OPERATION, 0x00)
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
    res = {"name": psu.name, "loc": psu.loc, "t": time.time()}
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


def discover(args, report=True):
    """Balaye les bus I2C (tous, ou celui demandé avec -b), les mux du PDB et les adresses.
    Renvoie la liste des PSU PMBus trouvés."""
    say = print if report else (lambda *a, **k: None)
    if args.bus_explicit:
        nums = [args.bus]
    else:   # le bus 0 de la BBB est interne (PMIC, EEPROM) : ignoré sauf avec -b 0
        nums = sorted(n for n in (int(f.split("-")[1]) for f in os.listdir("/dev")
                                  if f.startswith("i2c-") and f.split("-")[1].isdigit()) if n > 0)
    if not nums:
        say("Aucun /dev/i2c-N (N>0) : active le bus I2C (config-pin / overlay) et vérifie les droits.")
        return []
    say("Bus I2C à balayer : %s" % ", ".join(map(str, nums)))
    psus = []
    ids = {}    # identité (fabricant, modèle, série) -> emplacement déjà retenu

    def check(bus, num, addrs, mux=None, ch=None):
        for a in sorted(addrs):
            if args.addr is not None and a not in args.addr:
                continue
            where = "direct" if mux is None else "mux 0x%02X canal %d" % (mux, ch)
            if looks_like_pmbus(bus, a):
                ident = tuple(read_text(bus, a, c) for c in (0x99, 0x9A, 0x9E))
                if ident[2]:                   # N° de série lu : deux emplacements identiques = même PSU
                    if ident in ids:
                        say("  0x%02X (%s) ignoré : doublon de %s (série %s)" % (a, where, ids[ident], ident[2]))
                        continue
                    ids[ident] = "bus %d 0x%02X %s" % (num, a, where)
                psus.append(Psu("PSU%d" % (len(psus) + 1), num, a, mux, ch))
                say("  -> PSU PMBus en 0x%02X (%s) %s" % (a, where, " ".join(x for x in ident if x)))
            elif a in seen:
                say("  0x%02X (%s) répond mais n'est pas reconnu comme PMBus" % (a, where))

    for num in nums:
        try:
            bus = I2CBus(num)
        except OSError as e:
            say("Bus %d : ouverture impossible (%s)" % (num, e))
            continue
        try:
            say("Bus %d (pilote %s) :" % (num, bus.backend))
            cands = [args.mux] if args.mux is not None else [a for a in range(0x70, 0x78)
                                                              if safe(bus.read_byte, a) is not None]
            muxes = [m for m in cands if is_mux(bus, m)]       # remis à 0 par is_mux
            direct = scan_bus(bus)
            say("  adresses qui répondent : %s" % hexl(direct))
            if muxes:
                say("  mux I2C détecté(s) : %s" % hexl(muxes))
            skip = set(muxes) | set(range(0x50, 0x58))   # mux et EEPROM FRU : pas des PSU
            seen = set(direct)
            # on sonde 0x58-0x67 même si le balayage ne les a pas vues (certains PSU ignorent l'octet seul)
            check(bus, num, (set(direct) | set(PSU_RANGE)) - skip)
            for m in muxes:
                for ch in (args.channels if args.mux is not None else range(8)):
                    try:
                        bus.write_byte(m, 1 << ch)
                    except OSError:
                        continue
                    found = [a for a in scan_bus(bus) if a != m]
                    seen = set(found)
                    if found:
                        say("  mux 0x%02X canal %d : %s" % (m, ch, hexl(found)))
                    check(bus, num, (set(found) | set(PSU_RANGE)) - skip - set(muxes), m, ch)
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
        self.interval = interval
        self.history_s = history_s
        self.events = collections.deque(maxlen=1000)
        self.seq = 0
        self.reset(psus)

    def reset(self, psus):
        with self.lock:
            self.order = [p.name for p in psus]
            self.seen = set()      # PSU déjà vus en ligne : les autres restent invisibles
            self.latest = {}
            self.hist = {p.name: {} for p in psus}
            self.maxlen = max(120, int(self.history_s / max(self.interval, 0.2)))

    def export_csv(self):
        out = io.StringIO()
        w = csv.writer(out)
        w.writerow(["time", "psu"] + TELEMETRY_KEYS)
        with self.lock:
            for name, series in self.hist.items():
                rows = {}
                for k, dq in series.items():
                    for t, v in dq:
                        rows.setdefault(round(t, 2), {})[k] = v
                for t in sorted(rows):
                    r = rows[t]
                    w.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)), name]
                               + [("%.3f" % r[k]) if k in r else "" for k in TELEMETRY_KEYS])
        return out.getvalue()

    def _event(self, t, psu, sev, msg):
        self.seq += 1
        self.events.append({"id": self.seq, "t": t, "psu": psu, "sev": sev, "msg": msg})

    def log(self, psu, sev, msg):
        with self.lock:
            self._event(time.time(), psu, sev, msg)

    def update(self, results):
        with self.lock:
            for r in results:
                name, t = r["name"], r["t"]
                if r["online"]:
                    self.seen.add(name)
                elif name not in self.seen:      # jamais répondu : on ne l'affiche pas
                    self.latest[name] = r
                    continue
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
            psus = [self.latest[n] for n in self.order if n in self.seen]
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
.btn{background:var(--acc);color:#fff;border:0;border-radius:6px;padding:6px 12px;font:inherit;font-size:13px;cursor:pointer}
.btn.gray{background:var(--card);color:var(--fg);border:1px solid var(--bd)}.btn.red{background:var(--fault)}.btn.green{background:var(--ok)}
.btn:disabled{opacity:.5;cursor:wait}.acts{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}
a.btn{text-decoration:none;display:inline-block}input[type=search]{font:inherit;color:var(--fg);background:var(--card);border:1px solid var(--bd);border-radius:6px;padding:5px 8px}
.sim{background:var(--warn);color:#fff;font-size:12px;padding:2px 8px;border-radius:6px}
</style></head><body>
<header><h1>PMBus Monitor</h1><span id="summary"></span><span id="sim"></span><button class="btn gray" id="rescan" title="Relancer la détection des bus, mux et PSU">↻ Rescanner</button><span id="meta">chargement…</span></header>
<nav id="tabs">
 <button data-tab="overview" class="on">Vue d'ensemble</button>
 <button data-tab="details">Détails &amp; codes d'erreur</button>
 <button data-tab="graphs">Graphiques</button>
 <button data-tab="registers">Registres</button>
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
   <span id="legend"></span><a class="btn gray" href="api/export.csv">⬇ Exporter CSV</a></div>
  <div id="charts"></div></section>
 <section id="tab-registers" hidden>
  <div class="bar"><label>PSU : <select id="rsel"></select></label>
   <button class="btn" id="rread">Lire les registres</button>
   <input type="search" id="rfilter" placeholder="filtrer (nom, code, catégorie)…"><span id="rstat" style="color:var(--mut);font-size:13px"></span></div>
  <div id="regs" class="panel"><div class="empty">Choisis un PSU puis « Lire les registres » (lecture complète, quelques secondes).</div></div></section>
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

async function post(url,body){
  const r=await fetch(url,{method:"POST",headers:{"Content-Type":"application/json","X-Requested-With":"pmbus-monitor"},body:JSON.stringify(body||{})});
  const j=await r.json().catch(()=>({}));if(!r.ok)throw new Error(j.error||("HTTP "+r.status));return j;
}
$("rescan").onclick=async e=>{
  const b=e.target;b.disabled=true;b.textContent="Détection…";
  try{const j=await post("api/rescan");hist={};histSince=0;histLoaded=false;chartKey="";events=[];evId=0;
    $("meta").textContent=j.psus+" PSU détecté(s)"}catch(x){alert("Rescan impossible : "+x.message)}
  b.disabled=false;b.textContent="↻ Rescanner";
};
let regRows=[];
function renderRegs(){
  const f=$("rfilter").value.toLowerCase(),rows=regRows.filter(r=>!f||(r.name+r.code+r.cat+r.desc).toLowerCase().includes(f));
  if(!regRows.length)return;
  const cats={};rows.forEach(r=>(cats[r.cat]=cats[r.cat]||[]).push(r));
  $("regs").innerHTML=Object.entries(cats).map(([c,rs])=>`<h3>${esc(c)}</h3><table><tr><th>Code</th><th>Registre</th><th>Valeur</th><th>Description</th></tr>`+
    rs.map(r=>`<tr><td><code>${r.code}</code></td><td><b>${esc(r.name)}</b></td><td class="n" style="text-align:left">${esc(r.val)}</td><td style="color:var(--mut)">${esc(r.desc)}</td></tr>`).join("")+`</table>`).join("")||'<div class="empty">Aucun résultat</div>';
}
$("rfilter").oninput=renderRegs;
$("rread").onclick=async e=>{
  const b=e.target;b.disabled=true;$("rstat").textContent="lecture en cours…";
  try{const r=await fetch("api/registers?psu="+encodeURIComponent($("rsel").value),{cache:"no-store"}),j=await r.json();
    if(!r.ok)throw new Error(j.error);regRows=j.registers;renderRegs();
    $("rstat").textContent=regRows.filter(x=>x.val!=="n/a").length+" / "+regRows.length+" registres supportés — "+new Date().toLocaleTimeString()}
  catch(x){$("rstat").textContent="erreur : "+x.message}
  b.disabled=false;
};
async function act(psu,action){
  const txt={clear_faults:"Effacer les défauts mémorisés",on:"Mettre en MARCHE (OPERATION=0x80)",off:"ÉTEINDRE la sortie (OPERATION=0x00)"}[action];
  if(!confirm(txt+" sur "+psu+" ?"))return;
  try{await post("api/action",{psu,action});tick(true)}catch(x){alert("Échec : "+x.message)}
}
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
  if(!data.psus.length){$("grid").innerHTML='<div class="empty">Aucun PSU détecté. Le script retente toutes les 30 s ; bouton « Rescanner » pour forcer. Détail : <code>--scan</code> ou <code>--verbose</code> ; sinon <code>-b</code>, <code>--addr</code>, <code>--mux</code>.</div>';return}
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
    return `<div class="card ${p.summary} click" data-p="${esc(p.name)}"><div class="top"><span class="name">${esc(p.name)}</span><span class="pill ${p.summary}">${SEV[p.summary]}</span></div><div class="loc">${esc(p.loc)}</div><div class="loc" style="margin-top:-6px">${esc([p.info&&p.info["Modèle (MFR_MODEL)"],p.info&&p.info["N° de série"]&&("SN "+p.info["N° de série"])].filter(Boolean).join(" · "))}</div><dl>${rows}</dl><div class="chips">${chips}</div></div>`;
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
  if(data.control)h+=`<div class="acts"><button class="btn gray" data-a="clear_faults" data-p="${esc(p.name)}">Effacer les défauts</button><button class="btn green" data-a="on" data-p="${esc(p.name)}">ON</button><button class="btn red" data-a="off" data-p="${esc(p.name)}">OFF</button></div>`;
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
  document.querySelectorAll("#detail [data-a]").forEach(b=>b.onclick=()=>act(b.dataset.p,b.dataset.a));
  document.querySelectorAll("#detail details").forEach(d=>d.ontoggle=()=>d.open?openSec.add(d.dataset.k):openSec.delete(d.dataset.k));
}
function renderRegSel(){
  const opts=data.psus.map(p=>`<option value="${esc(p.name)}">${esc(p.name)}</option>`).join("");
  if($("rsel").dataset.o!==opts){const v=$("rsel").value;$("rsel").innerHTML=opts;$("rsel").dataset.o=opts;if(v)$("rsel").value=v}
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
  if(tab==="registers")renderRegSel();
  $("sim").innerHTML=data.mock?'<span class="sim">SIMULATION</span>':"";
}
async function tick(once){
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
  if(!once)setTimeout(tick,Math.max(1000,(data?data.interval:2)*1000));
}
tick();
</script></body></html>
"""


# --------------------------------------------------------------------------
# Serveur HTTP(S)
# --------------------------------------------------------------------------
class Monitor:
    """Pilote les threads de lecture (un par bus) ; peut tout relancer après un nouveau balayage."""

    def __init__(self, args, build):
        self.args = args
        self.build = build            # build(report) -> liste de PSU
        self.state = State([], args.interval, args.history)
        self.lock = threading.RLock()
        self.csv_lock = threading.Lock()
        self.psus, self.workers, self.threads = [], [], []
        self.stop = threading.Event()

    def start(self, psus):
        with self.lock:
            unique_names(psus)
            self.psus = psus
            self.state.reset(psus)
            self.stop = threading.Event()
            self.workers, self.threads = [], []
            for b in sorted({x.bus for x in psus}):
                mine = [x for x in psus if x.bus == b]
                try:
                    self.workers.append(BusWorker(b, mine))
                except OSError as e:
                    print("bus %d inaccessible : %s" % (b, e), file=sys.stderr)
                    self.state.update([{"name": x.name, "loc": x.loc, "t": time.time(), "online": False,
                                        "summary": "offline", "error": "bus %d inaccessible : %s" % (b, e)}
                                       for x in mine])
            self.threads = [threading.Thread(target=self._loop, args=(w, self.stop), daemon=True)
                            for w in self.workers]
            for t in self.threads:
                t.start()

    def halt(self):
        with self.lock:
            self.stop.set()
            for t in self.threads:
                t.join(timeout=self.args.interval + 5)
            for w in self.workers:
                w.close()
            self.workers, self.threads = [], []

    def rescan(self):
        with self.lock:
            self.halt()
            self.start(self.build(False))
            return self.psus

    def _loop(self, w, stop):
        while not stop.is_set():
            t0 = time.time()
            try:
                results = w.poll()
                self.state.update(results)
                if self.args.csv:
                    with self.csv_lock:
                        write_csv(self.args.csv, results)
            except Exception as e:   # un incident sur un bus ne doit pas tuer le thread
                print("bus %d : %s" % (w.num, e), file=sys.stderr)
            stop.wait(max(0.05, self.args.interval - (time.time() - t0)))

    def _find(self, name):
        for w in self.workers:
            for p in w.psus:
                if p.name == name:
                    return w, p
        raise KeyError(name)

    def read_registers(self, name):
        w, p = self._find(name)
        return w.run(p, lambda bus: read_registers(bus, p))

    def control(self, name, action):
        w, p = self._find(name)
        w.run(p, lambda bus: do_control(bus, p, action))
        self.state.log(p.name, "warn", "Action manuelle : %s" % action)
        p.reset()   # on relit tout au prochain cycle (limites, statuts)


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

        def do_GET(self):
            if not self._authorized():
                return self._deny()
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
                snap.update(metrics=METRICS_META, control=control, backend=mon.backend,
                            mock=bool(mon.args.mock))
                self._json(snap)
            elif u.path == "/api/defs":
                self._send(defs_json, "application/json")
            elif u.path == "/api/history":
                self._json(state.history(num("since")))
            elif u.path == "/api/events":
                self._json(state.events_since(int(num("since"))))
            elif u.path == "/api/registers":
                try:
                    self._json({"registers": mon.read_registers(q.get("psu", [""])[0])})
                except KeyError:
                    self._json({"error": "PSU inconnu"}, 404)
                except OSError as e:
                    self._json({"error": "lecture impossible : %s" % (e.strerror or e)}, 502)
            elif u.path == "/api/export.csv":
                name = "pmbus_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
                self._send(state.export_csv().encode(), "text/csv; charset=utf-8",
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
            if path == "/api/rescan":
                mon.rescan()
                time.sleep(min(6, mon.args.interval * 2 + 3))    # laisse le temps à la première lecture
                self._json({"psus": len(mon.state.seen)})
            elif path == "/api/action":
                if not control:
                    return self._json({"error": "actions désactivées (lancer avec --control)"}, 403)
                try:
                    mon.control(str(body.get("psu", "")), str(body.get("action", "")))
                    self._json({"status": "ok"})
                except KeyError:
                    self._json({"error": "PSU inconnu"}, 404)
                except ValueError as e:
                    self._json({"error": str(e)}, 400)
                except OSError as e:
                    self._json({"error": "écriture impossible : %s" % (e.strerror or e)}, 502)
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


def wait_first_poll(state, psus, timeout=20):
    end = time.time() + timeout
    while time.time() < end:
        with state.lock:
            if all(p.name in state.latest for p in psus):
                return
        time.sleep(0.2)


def print_short_banner(scheme, port, args):
    ips = [args.host] if args.host != "0.0.0.0" else (local_ips() or ["<ip-de-la-BBB>"])
    print("PMBus monitor - interface web :")
    for ip in ips:
        print("  %s://%s:%d" % (scheme, ip, port), flush=True)


def print_banner(state, psus, workers, scheme, port, args, backend=""):
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
    print(" Pilote I2C : %s%s" % (backend, "" if backend != "ioctl" else "  (conseil : pip install smbus2)"))
    if args.control:
        print(" ACTIONS D'ÉCRITURE ACTIVÉES (effacer défauts, ON/OFF) - protégées par le mot de passe")
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


def run_web(psus, args, build):
    mon = Monitor(args, build)
    mon.backend = "simulation" if args.mock else ("smbus2" if _SMBus else "ioctl")
    mon.start(psus)
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

    def auto_rescan():   # tant qu'aucun PSU n'est trouvé, on retente toutes les 30 s (branchement à chaud)
        while True:
            time.sleep(30)
            if not mon.state.seen:
                try:
                    mon.rescan()
                except Exception as e:
                    print("rescan : %s" % e, file=sys.stderr)

    threading.Thread(target=auto_rescan, daemon=True).start()
    if args.verbose:
        wait_first_poll(mon.state, psus)
        print_banner(mon.state, psus, mon.workers, scheme, port, args, mon.backend)
    else:
        print_short_banner(scheme, port, args)
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

    def build(report=True):
        if args.config:
            return psus_from_config(args.config, args.bus)[0]
        if args.autodetect or (args.addr is None and not args.pages):
            return discover(args, report and args.verbose)
        return psus_from_args(args)

    try:
        if args.config:
            cfg_interval = psus_from_config(args.config, args.bus)[1]
        psus = build()
    except (OSError, ValueError, KeyError) as e:
        sys.exit("Configuration impossible : %s" % e)
    args.interval = args.interval or cfg_interval or 2.0
    if not psus and not args.web:
        sys.exit("Aucun PSU trouvé. Lance --scan pour voir ce qui répond sur les bus.")
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    if args.web:
        run_web(psus, args, build)
        return

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
