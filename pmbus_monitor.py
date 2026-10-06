#!/usr/bin/env python3
"""
pmbus_monitor.py - Moniteur PMBus mono-fichier pour BeagleBone Black.

Sans dépendance externe (Python 3 standard + ioctl /dev/i2c-N).

Deux modes d'accès aux PSU :
  - direct : les PSU sont sur le bus I2C de la BBB      -> --addr 0x58 0x59
  - via PDB: les PSU sont derrière un mux I2C (PCA9548) -> --mux 0x70 --channels 0 1 2 3
             (--addr donne alors l'adresse des PSU sur chaque canal)

Exemples :
  ./pmbus_monitor.py --scan                         # cherche les périphériques
  ./pmbus_monitor.py --addr 0x58 0x59               # monitoring direct, rafraîchi chaque 2 s
  ./pmbus_monitor.py --mux 0x70 --channels 0-3 --addr 0x58
  ./pmbus_monitor.py --addr 0x58 --once --json      # une mesure, sortie JSON
  ./pmbus_monitor.py --addr 0x58 --csv log.csv -i 5 # log CSV toutes les 5 s

BBB : I2C2 = /dev/i2c-2 (P9_19 SCL / P9_20 SDA) est le bus par défaut.
      I2C1 = /dev/i2c-1 (P9_17 SCL / P9_18 SDA) nécessite un overlay.
"""

import argparse
import csv
import ctypes
import fcntl
import json
import os
import signal
import sys
import time

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

    def _select(self, addr):
        if addr != self.addr:
            fcntl.ioctl(self.fd, I2C_SLAVE_FORCE, addr)
            self.addr = addr

    def _xfer(self, addr, rw, cmd, size, data=None):
        self._select(addr)
        d = data or _SmbusData()
        arg = _SmbusIoctl(rw, cmd, size, ctypes.pointer(d))
        fcntl.ioctl(self.fd, I2C_SMBUS, arg)
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
        n = min(d.block[0], 32)
        return bytes(d.block[1:1 + n])


# --------------------------------------------------------------------------
# Registres PMBus
# --------------------------------------------------------------------------
CMD_PAGE = 0x00
CMD_VOUT_MODE = 0x20
CMD_STATUS_WORD = 0x79
CMD_STATUS_VOUT = 0x7A
CMD_STATUS_IOUT = 0x7B
CMD_STATUS_INPUT = 0x7C
CMD_STATUS_TEMP = 0x7D
CMD_STATUS_CML = 0x7E
CMD_STATUS_FAN12 = 0x81
CMD_READ_VIN = 0x88
CMD_READ_IIN = 0x89
CMD_READ_VOUT = 0x8B
CMD_READ_IOUT = 0x8C
CMD_READ_TEMP1 = 0x8D
CMD_READ_TEMP2 = 0x8E
CMD_READ_TEMP3 = 0x8F
CMD_READ_FAN1 = 0x90
CMD_READ_FAN2 = 0x91
CMD_READ_POUT = 0x96
CMD_READ_PIN = 0x97
CMD_MFR_ID = 0x99
CMD_MFR_MODEL = 0x9A
CMD_MFR_REVISION = 0x9B
CMD_MFR_SERIAL = 0x9E

# (clé, commande, unité, format d'affichage) - lues en LINEAR11 sauf VOUT
TELEMETRY = [
    ("vin",   CMD_READ_VIN,   "V",   "%.1f"),
    ("iin",   CMD_READ_IIN,   "A",   "%.2f"),
    ("pin",   CMD_READ_PIN,   "W",   "%.0f"),
    ("vout",  CMD_READ_VOUT,  "V",   "%.2f"),
    ("iout",  CMD_READ_IOUT,  "A",   "%.2f"),
    ("pout",  CMD_READ_POUT,  "W",   "%.0f"),
    ("temp1", CMD_READ_TEMP1, "C",   "%.0f"),
    ("temp2", CMD_READ_TEMP2, "C",   "%.0f"),
    ("temp3", CMD_READ_TEMP3, "C",   "%.0f"),
    ("fan1",  CMD_READ_FAN1,  "rpm", "%.0f"),
    ("fan2",  CMD_READ_FAN2,  "rpm", "%.0f"),
]

STATUS_WORD_BITS = [
    (15, "VOUT"), (14, "IOUT/POUT"), (13, "INPUT"), (12, "MFR"),
    (11, "PGOOD#"), (10, "FANS"), (9, "OTHER"), (8, "UNKNOWN"),
    (7, "BUSY"), (6, "OFF"), (5, "VOUT_OV"), (4, "IOUT_OC"),
    (3, "VIN_UV"), (2, "TEMP"), (1, "CML"), (0, "NONE_OF_ABOVE"),
]


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


def decode_status_word(sw):
    return [name for bit, name in STATUS_WORD_BITS if sw & (1 << bit)]


# --------------------------------------------------------------------------
# Lecture d'un PSU
# --------------------------------------------------------------------------
def read_text(bus, addr, cmd):
    try:
        return bus.read_block_data(addr, cmd).decode("ascii", "replace").strip("\x00 ")
    except OSError:
        return ""


def read_psu(bus, addr, with_info=False):
    """Retourne un dict de mesures ; 'error' est présent si le PSU ne répond pas."""
    res = {}
    try:
        sw = bus.read_word_data(addr, CMD_STATUS_WORD)
    except OSError as e:
        return {"error": "pas de réponse (%s)" % (e.strerror or e)}
    res["status_word"] = sw
    res["flags"] = decode_status_word(sw)

    try:
        vout_mode = bus.read_byte_data(addr, CMD_VOUT_MODE)
    except OSError:
        vout_mode = None

    for key, cmd, _unit, _fmt in TELEMETRY:
        try:
            raw = bus.read_word_data(addr, cmd)
        except OSError:
            continue  # commande non supportée par ce PSU
        # VOUT : LINEAR16 si VOUT_MODE indique le mode linéaire (bits 7:5 == 0)
        if key == "vout" and vout_mode is not None and (vout_mode >> 5) == 0:
            res[key] = decode_linear16(raw, vout_mode)
        else:
            res[key] = decode_linear11(raw)

    if with_info:
        for key, cmd in (("mfr", CMD_MFR_ID), ("model", CMD_MFR_MODEL),
                         ("rev", CMD_MFR_REVISION), ("serial", CMD_MFR_SERIAL)):
            res[key] = read_text(bus, addr, cmd)
    return res


# --------------------------------------------------------------------------
# Mux I2C du PDB (PCA9546/9548/9545 : écrire un octet = masque de canaux)
# --------------------------------------------------------------------------
def mux_select(bus, mux_addr, channel):
    bus.write_byte(mux_addr, 1 << channel)


def mux_off(bus, mux_addr):
    try:
        bus.write_byte(mux_addr, 0)
    except OSError:
        pass


# --------------------------------------------------------------------------
# Scan
# --------------------------------------------------------------------------
def scan_bus(bus):
    found = []
    for a in range(0x03, 0x78):
        try:
            if 0x30 <= a <= 0x37 or 0x50 <= a <= 0x5F:
                bus.read_byte_data(a, 0x00)   # évite de corrompre des EEPROM
            else:
                bus.read_byte(a)
            found.append(a)
        except OSError:
            pass
    return found


def do_scan(bus, args):
    def show(label, addrs):
        s = " ".join("0x%02X" % a for a in addrs) or "(rien)"
        print("%s: %s" % (label, s))

    show("Bus %d" % args.bus, scan_bus(bus))
    if args.mux is not None:
        for ch in args.channels:
            try:
                mux_select(bus, args.mux, ch)
                show("  mux 0x%02X canal %d" % (args.mux, ch), scan_bus(bus))
            except OSError as e:
                print("  mux 0x%02X canal %d: erreur %s" % (args.mux, ch, e))
        mux_off(bus, args.mux)


# --------------------------------------------------------------------------
# Collecte / affichage
# --------------------------------------------------------------------------
def build_targets(args):
    """Liste de (label, channel|None, addr)."""
    targets = []
    if args.mux is None:
        for a in args.addr:
            targets.append(("0x%02X" % a, None, a))
    else:
        for ch in args.channels:
            for a in args.addr:
                targets.append(("ch%d/0x%02X" % (ch, a), ch, a))
    return targets


def collect(bus, args, targets, with_info):
    out = []
    for label, ch, addr in targets:
        if ch is not None:
            try:
                mux_select(bus, args.mux, ch)
            except OSError as e:
                out.append((label, {"error": "mux: %s" % (e.strerror or e)}))
                continue
        out.append((label, read_psu(bus, addr, with_info)))
    if args.mux is not None:
        mux_off(bus, args.mux)
    return out


def fmt_value(d, key):
    for k, _c, unit, fmt in TELEMETRY:
        if k == key:
            return (fmt % d[key]) if key in d else "-"
    return "-"


def render_table(results):
    cols = [("PSU", 10), ("VIN", 7), ("IIN", 7), ("PIN", 7), ("VOUT", 7),
            ("IOUT", 7), ("POUT", 7), ("T1", 5), ("T2", 5), ("FAN1", 6), ("STATUS", 0)]
    keys = [None, "vin", "iin", "pin", "vout", "iout", "pout", "temp1", "temp2", "fan1", None]
    lines = ["".join(n.ljust(w) if w else n for n, w in cols)]
    lines.append("-" * 78)
    for label, d in results:
        if "error" in d:
            lines.append("%-10s %s" % (label, d["error"]))
            continue
        cells = [label.ljust(10)]
        for (name, w), k in zip(cols[1:-1], keys[1:-1]):
            cells.append(fmt_value(d, k).ljust(w))
        cells.append("OK" if d["status_word"] == 0 else ",".join(d["flags"]) or "0x%04X" % d["status_word"])
        lines.append("".join(cells))
    lines.append("")
    lines.append("Unités: V, A, W, T=°C, FAN=rpm")
    return "\n".join(lines)


def render_info(results):
    lines = []
    for label, d in results:
        if "error" in d:
            continue
        lines.append("%-10s %s %s rev=%s sn=%s" % (
            label, d.get("mfr", ""), d.get("model", ""), d.get("rev", ""), d.get("serial", "")))
    return "\n".join(lines)


CSV_KEYS = [k for k, _c, _u, _f in TELEMETRY]


def write_csv(path, results):
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["timestamp", "psu", "status_word"] + CSV_KEYS + ["error"])
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        for label, d in results:
            w.writerow([ts, label, "0x%04X" % d["status_word"] if "status_word" in d else ""]
                       + [("%.3f" % d[k]) if k in d else "" for k in CSV_KEYS]
                       + [d.get("error", "")])


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def int_auto(s):
    return int(s, 0)


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


def main():
    p = argparse.ArgumentParser(description="Moniteur PMBus pour BeagleBone Black",
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument("-b", "--bus", type=int, default=2, help="numéro du bus I2C (défaut 2)")
    p.add_argument("-a", "--addr", type=int_auto, nargs="+", default=[0x58],
                   help="adresse(s) PMBus des PSU, ex: 0x58 0x59 (défaut 0x58)")
    p.add_argument("--mux", type=int_auto, default=None,
                   help="adresse du mux I2C du PDB (ex: 0x70) ; active le mode PDB")
    p.add_argument("--channels", nargs="+", default=["0-7"],
                   help="canaux du mux à lire, ex: 0 1 2 ou 0-3 (défaut 0-7)")
    p.add_argument("-i", "--interval", type=float, default=2.0, help="période en s (défaut 2)")
    p.add_argument("--once", action="store_true", help="une seule mesure puis quitter")
    p.add_argument("--scan", action="store_true", help="scanner le bus (et les canaux du mux)")
    p.add_argument("--info", action="store_true", help="afficher fabricant/modèle/série")
    p.add_argument("--json", action="store_true", help="sortie JSON (une ligne par mesure)")
    p.add_argument("--csv", metavar="FICHIER", help="ajouter les mesures à un fichier CSV")
    args = p.parse_args()
    args.channels = parse_channels(args.channels)

    try:
        bus = I2CBus(args.bus)
    except OSError as e:
        sys.exit("Impossible d'ouvrir /dev/i2c-%d : %s\n"
                 "Vérifie que le bus est activé (overlay) et que tu as les droits." % (args.bus, e))

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        if args.scan:
            do_scan(bus, args)
            return

        targets = build_targets(args)
        first = True
        while True:
            results = collect(bus, args, targets, with_info=args.info and first)
            if args.csv:
                write_csv(args.csv, results)
            if args.json:
                print(json.dumps({"time": time.time(), "psu": dict(results)}), flush=True)
            else:
                if not args.once and sys.stdout.isatty():
                    sys.stdout.write("\x1b[2J\x1b[H")
                print("PMBus monitor - bus %d - %s" % (args.bus, time.strftime("%H:%M:%S")))
                if args.info and first:
                    print(render_info(results))
                print(render_table(results), flush=True)
            first = False
            if args.once:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == "__main__":
    main()
