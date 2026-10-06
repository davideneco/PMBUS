#!/usr/bin/env python3
"""
================================================================================
 PMBus Multi-PSU Monitor & Web GUI Dashboard (BeagleBone Black / Linux SBC)
================================================================================
 - 100% Autonome : Fichier unique avec serveur Web HTTP, API REST, flux SSE et UI embarquée.
 - Détection automatique de 1 à 5+ modules d'alimentation (PSU) sur le bus I2C.
 - Support multi-PSU : Vue d'ensemble globale (Rack Power Grid) et Deep-Dive par PSU.
 - Décodage exhaustif selon la norme PMBus 1.2 / 1.3 :
   * Télémétries physiques de précision (VIN, VOUT, IIN, IOUT, PIN, POUT, Rendement, Températures 1/2/3, Ventilateurs RPM)
   * Matrice d'état bit-à-bit (STATUS_WORD, STATUS_BYTE, STATUS_VOUT, STATUS_IOUT, STATUS_INPUT, STATUS_TEMPERATURE, STATUS_CML, STATUS_FANS_1_2)
   * Table complète des 80+ registres PMBus avec recherche et filtrage par catégorie
   * Oscilloscope graphique Canvas multi-courbes temps réel avec crosshair
   * Enregistreur continu de données CSV multi-PSU
   * Commandes de contrôle (CLEAR_FAULTS, OPERATION ON/OFF/Margins, VOUT_COMMAND, seuils VIN_ON/OFF, TON_RISE)
 - Moteur de communication I2C tolérant aux pannes (Pacing delay, retries automatiques, cache Sample & Hold)
 - Mode Simulation / Mock complet et réaliste intégré (auto-fallback sur PC).
================================================================================
"""

import os
import sys
import time
import math
import json
import csv
import io
import socket
import argparse
import threading
import urllib.parse
import ssl
import subprocess
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

# ==============================================================================
# 1. TABLE EXHAUSTIVE DES REGISTRES PMBUS 1.2 / 1.3
# ==============================================================================

PMBUS_SAFE_REGISTERS = [
    # --- CONFIGURATION & CONTROLE ---
    {"code": 0x00, "name": "PAGE", "type": "byte_page", "cat": "Configuration", "desc": "Active channel/phase selection (0 to 31, 0xFF=All)"},
    {"code": 0x01, "name": "OPERATION", "type": "byte_op", "cat": "Configuration", "desc": "Active power stage state (0x80=On, 0x00=Off, Margins)"},
    {"code": 0x02, "name": "ON_OFF_CONFIG", "type": "byte_onoff", "cat": "Configuration", "desc": "Hardware CONTROL pin and software enable logic"},
    {"code": 0x10, "name": "WRITE_PROTECT", "type": "byte_wp", "cat": "Configuration", "desc": "Register write protection status"},
    {"code": 0x19, "name": "CAPABILITY", "type": "byte_cap", "cat": "Configuration", "desc": "Supported features (PEC, Max speed, SMBALERT#)"},

    # --- ENTREE SECTEUR / BUS DC (VIN / IIN / PIN) ---
    {"code": 0x35, "name": "VIN_ON", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Input voltage turn-on / startup threshold (V)"},
    {"code": 0x36, "name": "VIN_OFF", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Input voltage turn-off / undervoltage shutdown threshold (V)"},
    {"code": 0x55, "name": "VIN_OV_FAULT_LIMIT", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Input Overvoltage Fault shutdown threshold (V)"},
    {"code": 0x56, "name": "VIN_OV_FAULT_RESPONSE", "type": "byte_resp", "cat": "Startup & Input (VIN)", "desc": "Response profile for Input Overvoltage Fault"},
    {"code": 0x57, "name": "VIN_OV_WARN_LIMIT", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Input Overvoltage Warning advisory threshold (V)"},
    {"code": 0x58, "name": "VIN_UV_WARN_LIMIT", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Input Undervoltage Warning advisory threshold (V)"},
    {"code": 0x59, "name": "VIN_UV_FAULT_LIMIT", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Input Undervoltage Fault shutdown threshold (V)"},
    {"code": 0x5A, "name": "VIN_UV_FAULT_RESPONSE", "type": "byte_resp", "cat": "Startup & Input (VIN)", "desc": "Response profile for Input Undervoltage Fault"},
    {"code": 0x5B, "name": "IIN_OC_FAULT_LIMIT", "type": "word_curr", "cat": "Startup & Input (VIN)", "desc": "Input Overcurrent Fault shutdown threshold (A)"},
    {"code": 0x5C, "name": "IIN_OC_FAULT_RESPONSE", "type": "byte_resp", "cat": "Startup & Input (VIN)", "desc": "Response profile for Input Overcurrent Fault"},
    {"code": 0x5D, "name": "IIN_OC_WARN_LIMIT", "type": "word_curr", "cat": "Startup & Input (VIN)", "desc": "Input Overcurrent Warning advisory threshold (A)"},
    {"code": 0x6B, "name": "PIN_OP_WARN_LIMIT", "type": "word_power", "cat": "Startup & Input (VIN)", "desc": "Input Overpower Warning advisory threshold (W)"},
    {"code": 0x88, "name": "READ_VIN", "type": "word_volt", "cat": "Startup & Input (VIN)", "desc": "Real-time measured input voltage (V)"},
    {"code": 0x89, "name": "READ_IIN", "type": "word_curr", "cat": "Startup & Input (VIN)", "desc": "Real-time measured input current (A)"},
    {"code": 0x97, "name": "READ_PIN", "type": "word_power", "cat": "Startup & Input (VIN)", "desc": "Real-time measured input power (W)"},

    # --- TENSION DE SORTIE & LIMITES (VOUT) ---
    {"code": 0x20, "name": "VOUT_MODE", "type": "byte_vmode", "cat": "Output Voltage (VOUT)", "desc": "VOUT data format (Linear/VID/Direct) and exponent"},
    {"code": 0x21, "name": "VOUT_COMMAND", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Target output voltage command setpoint (V)"},
    {"code": 0x22, "name": "VOUT_TRIM", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Fine trimming offset for output voltage (V)"},
    {"code": 0x23, "name": "VOUT_CAL_OFFSET", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output voltage calibration offset (V)"},
    {"code": 0x24, "name": "VOUT_MAX", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Upper safety clamp limit for VOUT (V)"},
    {"code": 0x25, "name": "VOUT_MARGIN_HIGH", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Target voltage for high margin testing (V)"},
    {"code": 0x26, "name": "VOUT_MARGIN_LOW", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Target voltage for low margin testing (V)"},
    {"code": 0x27, "name": "VOUT_TRANSITION_RATE", "type": "word_v_ms", "cat": "Output Voltage (VOUT)", "desc": "Output voltage slew rate during transitions (V/ms)"},
    {"code": 0x28, "name": "VOUT_DROOP", "type": "word_droop", "cat": "Output Voltage (VOUT)", "desc": "Active voltage positioning loadline slope (mV/A)"},
    {"code": 0x29, "name": "VOUT_SCALE_LOOP", "type": "word_linear_ratio", "cat": "Output Voltage (VOUT)", "desc": "Feedback divider scaling ratio"},
    {"code": 0x40, "name": "VOUT_OV_FAULT_LIMIT", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output Overvoltage Fault shutdown threshold (V)"},
    {"code": 0x41, "name": "VOUT_OV_FAULT_RESPONSE", "type": "byte_resp", "cat": "Output Voltage (VOUT)", "desc": "Response profile for VOUT Overvoltage Fault"},
    {"code": 0x42, "name": "VOUT_OV_WARN_LIMIT", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output Overvoltage Warning advisory threshold (V)"},
    {"code": 0x43, "name": "VOUT_UV_WARN_LIMIT", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output Undervoltage Warning advisory threshold (V)"},
    {"code": 0x44, "name": "VOUT_UV_FAULT_LIMIT", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output Undervoltage Fault shutdown threshold (V)"},
    {"code": 0x45, "name": "VOUT_UV_FAULT_RESPONSE", "type": "byte_resp", "cat": "Output Voltage (VOUT)", "desc": "Response profile for VOUT Undervoltage Fault"},
    {"code": 0x5E, "name": "POWER_GOOD_ON", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output voltage threshold for Power Good assertion (V)"},
    {"code": 0x5F, "name": "POWER_GOOD_OFF", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Output voltage threshold for Power Good deassertion (V)"},
    {"code": 0x8B, "name": "READ_VOUT", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Real-time measured output voltage (V)"},

    # --- SEQUENCEMENT & TEMPORISATIONS (TON / TOFF) ---
    {"code": 0x60, "name": "TON_DELAY", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-on delay time from enable to ramp (ms)"},
    {"code": 0x61, "name": "TON_RISE", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-on rise / soft-start ramp time (ms)"},
    {"code": 0x62, "name": "TON_MAX_FAULT_LIMIT", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Maximum allowed turn-on transition time (ms)"},
    {"code": 0x63, "name": "TON_MAX_FAULT_RESPONSE", "type": "byte_resp", "cat": "Sequencing & Timings", "desc": "Response profile for TON Max Fault"},
    {"code": 0x64, "name": "TOFF_DELAY", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-off delay time (ms)"},
    {"code": 0x65, "name": "TOFF_FALL", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-off ramp-down fall time (ms)"},
    {"code": 0x66, "name": "TOFF_MAX_WARN_LIMIT", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-off maximum warning limit (ms)"},

    # --- COURANT & PUISSANCE DE SORTIE (IOUT / POUT) ---
    {"code": 0x31, "name": "POUT_MAX", "type": "word_power", "cat": "Current & Power (IOUT)", "desc": "Maximum continuous output power rating (W)"},
    {"code": 0x38, "name": "IOUT_CAL_GAIN", "type": "word_mohm", "cat": "Current & Power (IOUT)", "desc": "Current sense shunt resistance (mOhm)"},
    {"code": 0x39, "name": "IOUT_CAL_OFFSET", "type": "word_curr", "cat": "Current & Power (IOUT)", "desc": "Output current calibration offset (A)"},
    {"code": 0x46, "name": "IOUT_OC_FAULT_LIMIT", "type": "word_curr", "cat": "Current & Power (IOUT)", "desc": "Output Overcurrent Fault shutdown threshold (A)"},
    {"code": 0x47, "name": "IOUT_OC_FAULT_RESPONSE", "type": "byte_resp", "cat": "Current & Power (IOUT)", "desc": "Response profile for Output Overcurrent Fault"},
    {"code": 0x48, "name": "IOUT_OC_LV_FAULT_LIMIT", "type": "word_vout", "cat": "Current & Power (IOUT)", "desc": "Output Overcurrent Low-Voltage shutdown threshold (V)"},
    {"code": 0x49, "name": "IOUT_OC_LV_FAULT_RESPONSE", "type": "byte_resp", "cat": "Current & Power (IOUT)", "desc": "Response profile for OC Low-Voltage Fault"},
    {"code": 0x4A, "name": "IOUT_OC_WARN_LIMIT", "type": "word_curr", "cat": "Current & Power (IOUT)", "desc": "Output Overcurrent Warning advisory threshold (A)"},
    {"code": 0x4B, "name": "IOUT_UC_FAULT_LIMIT", "type": "word_curr", "cat": "Current & Power (IOUT)", "desc": "Output Undercurrent Fault shutdown threshold (A)"},
    {"code": 0x4C, "name": "IOUT_UC_FAULT_RESPONSE", "type": "byte_resp", "cat": "Current & Power (IOUT)", "desc": "Response profile for Output Undercurrent Fault"},
    {"code": 0x68, "name": "POUT_OP_FAULT_LIMIT", "type": "word_power", "cat": "Current & Power (IOUT)", "desc": "Output Overpower Fault shutdown threshold (W)"},
    {"code": 0x69, "name": "POUT_OP_FAULT_RESPONSE", "type": "byte_resp", "cat": "Current & Power (IOUT)", "desc": "Response profile for Output Overpower Fault"},
    {"code": 0x6A, "name": "POUT_OP_WARN_LIMIT", "type": "word_power", "cat": "Current & Power (IOUT)", "desc": "Output Overpower Warning advisory threshold (W)"},
    {"code": 0x8C, "name": "READ_IOUT", "type": "word_curr", "cat": "Current & Power (IOUT)", "desc": "Real-time measured output current (A)"},
    {"code": 0x96, "name": "READ_POUT", "type": "word_power", "cat": "Current & Power (IOUT)", "desc": "Real-time measured output power (W)"},

    # --- THERMIQUE & REFROIDISSEMENT ---
    {"code": 0x4F, "name": "OT_FAULT_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Overtemperature Fault shutdown threshold (deg C)"},
    {"code": 0x50, "name": "OT_FAULT_RESPONSE", "type": "byte_resp", "cat": "Thermal & Cooling", "desc": "Response profile for Overtemperature Fault"},
    {"code": 0x51, "name": "OT_WARN_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Overtemperature Warning advisory threshold (deg C)"},
    {"code": 0x52, "name": "UT_WARN_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Undertemperature Warning advisory threshold (deg C)"},
    {"code": 0x53, "name": "UT_FAULT_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Undertemperature Fault shutdown threshold (deg C)"},
    {"code": 0x54, "name": "UT_FAULT_RESPONSE", "type": "byte_resp", "cat": "Thermal & Cooling", "desc": "Response profile for Undertemperature Fault"},
    {"code": 0x3A, "name": "FAN_CONFIG_1_2", "type": "byte_hex", "cat": "Thermal & Cooling", "desc": "Fan 1 and 2 configuration parameters"},
    {"code": 0x3B, "name": "FAN_COMMAND_1", "type": "word_rpm", "cat": "Thermal & Cooling", "desc": "Fan 1 speed command setpoint (RPM or %)"},
    {"code": 0x3C, "name": "FAN_COMMAND_2", "type": "word_rpm", "cat": "Thermal & Cooling", "desc": "Fan 2 speed command setpoint (RPM or %)"},
    {"code": 0x8D, "name": "READ_TEMPERATURE_1", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Primary power stage temperature reading (deg C)"},
    {"code": 0x8E, "name": "READ_TEMPERATURE_2", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Secondary / Synchronous rectifier temperature (deg C)"},
    {"code": 0x8F, "name": "READ_TEMPERATURE_3", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Ambient / Air intake temperature reading (deg C)"},
    {"code": 0x90, "name": "READ_FAN_SPEED_1", "type": "word_rpm", "cat": "Thermal & Cooling", "desc": "Measured speed of cooling fan 1 (RPM)"},
    {"code": 0x91, "name": "READ_FAN_SPEED_2", "type": "word_rpm", "cat": "Thermal & Cooling", "desc": "Measured speed of cooling fan 2 (RPM)"},

    # --- REGULATION & HORLOGE ---
    {"code": 0x32, "name": "MAX_DUTY", "type": "word_percent", "cat": "Regulation & Timing", "desc": "Maximum permitted PWM duty cycle (%)"},
    {"code": 0x33, "name": "FREQUENCY_SWITCH", "type": "word_khz", "cat": "Regulation & Timing", "desc": "Target switching frequency (kHz)"},
    {"code": 0x94, "name": "READ_DUTY_CYCLE", "type": "word_percent", "cat": "Regulation & Timing", "desc": "Measured PWM duty cycle (%)"},
    {"code": 0x95, "name": "READ_FREQUENCY", "type": "word_khz", "cat": "Regulation & Timing", "desc": "Measured switching frequency (kHz)"},

    # --- STATUTS & DIAGNOSTICS PMBUS ---
    {"code": 0x78, "name": "STATUS_BYTE", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Summary status byte (critical faults)"},
    {"code": 0x79, "name": "STATUS_WORD", "type": "word_hex", "cat": "Status & Diagnostics", "desc": "Full 16-bit status word"},
    {"code": 0x7A, "name": "STATUS_VOUT", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Output voltage faults detail"},
    {"code": 0x7B, "name": "STATUS_IOUT", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Output current and power faults detail"},
    {"code": 0x7C, "name": "STATUS_INPUT", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Input voltage and current faults detail"},
    {"code": 0x7D, "name": "STATUS_TEMPERATURE", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Thermal faults detail"},
    {"code": 0x7E, "name": "STATUS_CML", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "I2C, CRC/PEC, memory and command errors"},
    {"code": 0x80, "name": "STATUS_MFR_SPECIFIC", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Manufacturer proprietary diagnostic flags"},
    {"code": 0x81, "name": "STATUS_FANS_1_2", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Cooling fans 1 and 2 fault status"},

    # --- INFORMATIONS CONSTRUCTEUR & CARACTERISTIQUES ---
    {"code": 0x98, "name": "PMBUS_REVISION", "type": "byte_hex", "cat": "Manufacturer Info", "desc": "Supported PMBus specification revision code"},
    {"code": 0x99, "name": "MFR_ID", "type": "block_ascii", "cat": "Manufacturer Info", "desc": "Power supply manufacturer identity string"},
    {"code": 0x9A, "name": "MFR_MODEL", "type": "block_ascii", "cat": "Manufacturer Info", "desc": "Equipment commercial model number string"},
    {"code": 0x9B, "name": "MFR_REVISION", "type": "block_ascii", "cat": "Manufacturer Info", "desc": "Hardware revision / firmware build string"},
    {"code": 0x9C, "name": "MFR_LOCATION", "type": "block_ascii", "cat": "Manufacturer Info", "desc": "Manufacturing facility location"},
    {"code": 0x9D, "name": "MFR_DATE", "type": "block_ascii", "cat": "Manufacturer Info", "desc": "Manufacturing date code (YYMMDD)"},
    {"code": 0x9E, "name": "MFR_SERIAL", "type": "block_ascii", "cat": "Manufacturer Info", "desc": "Unique device serial number"},
    {"code": 0xA0, "name": "MFR_VIN_MIN", "type": "word_volt", "cat": "Manufacturer Info", "desc": "Minimum guaranteed input operating voltage (V)"},
    {"code": 0xA1, "name": "MFR_VIN_MAX", "type": "word_volt", "cat": "Manufacturer Info", "desc": "Maximum guaranteed input operating voltage (V)"},
    {"code": 0xA2, "name": "MFR_IIN_MAX", "type": "word_curr", "cat": "Manufacturer Info", "desc": "Maximum rated input current (A)"},
    {"code": 0xA3, "name": "MFR_PIN_MAX", "type": "word_power", "cat": "Manufacturer Info", "desc": "Maximum rated input power (W)"},
    {"code": 0xA4, "name": "MFR_VOUT_MIN", "type": "word_vout", "cat": "Manufacturer Info", "desc": "Minimum adjustable output voltage (V)"},
    {"code": 0xA5, "name": "MFR_VOUT_MAX", "type": "word_vout", "cat": "Manufacturer Info", "desc": "Maximum adjustable output voltage (V)"},
    {"code": 0xA6, "name": "MFR_IOUT_MAX", "type": "word_curr", "cat": "Manufacturer Info", "desc": "Maximum continuous rated output current (A)"},
    {"code": 0xA7, "name": "MFR_POUT_MAX", "type": "word_power", "cat": "Manufacturer Info", "desc": "Maximum continuous rated output power (W)"},
    {"code": 0xA8, "name": "MFR_TAMBIENT_MAX", "type": "word_temp", "cat": "Manufacturer Info", "desc": "Maximum rated ambient temperature (deg C)"},
    {"code": 0xA9, "name": "MFR_TAMBIENT_MIN", "type": "word_temp", "cat": "Manufacturer Info", "desc": "Minimum rated ambient temperature (deg C)"}
]

PMBUS_STATUS_CHIPS = [
    {
        "reg": "STATUS_BYTE", "code": 0x78, "title": "STATUS_BYTE (0x78)",
        "chips": [
            ("Busy", 7, "Bit 7 (0x80) - Device is busy processing an internal command or task and cannot respond", False),
            ("Off", 6, "Bit 6 (0x40) - Output conversion stage is DISABLED (Unit is turned OFF)", False),
            ("VoutOV", 5, "Bit 5 (0x20) - Output Overvoltage Fault (Measured VOUT exceeded safety limit VOUT_OV_FAULT_LIMIT)", True),
            ("IoutOC", 4, "Bit 4 (0x10) - Output Overcurrent Fault (Measured IOUT exceeded safety limit IOUT_OC_FAULT_LIMIT)", True),
            ("VinUV", 3, "Bit 3 (0x08) - Input Undervoltage Fault (Measured VIN fell below safety limit VIN_UV_FAULT_LIMIT)", True),
            ("Temp", 2, "Bit 2 (0x04) - Temperature Fault or Warning (Overtemperature detected on power module)", True),
            ("CML", 1, "Bit 1 (0x02) - Communication, Memory, PEC or Logic Fault active in STATUS_CML register (0x7E)", True),
            ("Other", 0, "Bit 0 (0x01) - None of the above: Other hardware condition or sub-warning is asserted", False)
        ]
    },
    {
        "reg": "STATUS_WORD", "code": 0x79, "title": "STATUS_WORD (0x79)",
        "chips": [
            ("VOUT", 15, "Bit 15 (0x8000) - Output voltage fault or warning condition is currently active in STATUS_VOUT (0x7A)", True),
            ("IOUT/POUT", 14, "Bit 14 (0x4000) - Output current or power fault condition is active in STATUS_IOUT (0x7B)", True),
            ("INPUT", 13, "Bit 13 (0x2000) - Input voltage, current or power fault condition is active in STATUS_INPUT (0x7C)", True),
            ("MFR_SPEC", 12, "Bit 12 (0x1000) - Manufacturer specific custom diagnostic fault flag is asserted in STATUS_MFR (0x80)", False),
            ("POWER_GOOD#", 11, "Bit 11 (0x0800) - Power Good signal is Negated / Inactive (Output voltage out of regulation)", True),
            ("FANS", 10, "Bit 10 (0x0400) - Cooling fan speed fault or airflow restriction active in STATUS_FANS (0x81)", False),
            ("OTHER", 9, "Bit 9 (0x0200) - Other internal hardware fault condition detected", False),
            ("UNKNOWN", 8, "Bit 8 (0x0100) - Unknown or unclassified hardware event flag", False),
            ("BUSY", 7, "Bit 7 (0x0080) - Device is busy processing an internal command or task", False),
            ("OFF", 6, "Bit 6 (0x0040) - Output power conversion stage is DISABLED (Unit is turned OFF)", False),
            ("VOUT_OV", 5, "Bit 5 (0x0020) - Output Overvoltage Fault (VOUT > VOUT_OV_FAULT_LIMIT)", True),
            ("IOUT_OC", 4, "Bit 4 (0x0010) - Output Overcurrent Fault (IOUT > IOUT_OC_FAULT_LIMIT)", True),
            ("VIN_UV", 3, "Bit 3 (0x0008) - Input Undervoltage Fault (VIN < VIN_UV_FAULT_LIMIT)", True),
            ("TEMP", 2, "Bit 2 (0x0004) - Temperature Fault or Warning on power stage", True),
            ("CML", 1, "Bit 1 (0x0002) - Communication, Memory, PEC or Logic Fault active in STATUS_CML (0x7E)", True),
            ("NONE", 0, "Bit 0 (0x0001) - Other hardware condition asserted in status registers", False)
        ]
    },
    {
        "reg": "STATUS_VOUT", "code": 0x7A, "title": "STATUS_VOUT (0x7A)",
        "chips": [
            ("VoutOV_Flt", 7, "Bit 7 (0x80) - Output Overvoltage Fault shutdown threshold exceeded (VOUT > VOUT_OV_FAULT_LIMIT)", True),
            ("VoutOV_Warn", 6, "Bit 6 (0x40) - Output Overvoltage Warning advisory threshold exceeded (VOUT > VOUT_OV_WARN_LIMIT)", False),
            ("VoutUV_Warn", 5, "Bit 5 (0x20) - Output Undervoltage Warning advisory threshold reached (VOUT < VOUT_UV_WARN_LIMIT)", False),
            ("VoutUV_Flt", 4, "Bit 4 (0x10) - Output Undervoltage Fault shutdown threshold reached (VOUT < VOUT_UV_FAULT_LIMIT)", True),
            ("VoutMax_Warn", 3, "Bit 3 (0x08) - VOUT Max Warning: Target voltage setpoint was clamped to maximum safety limit VOUT_MAX", False),
            ("TonMax_Flt", 2, "Bit 2 (0x04) - Turn-On Time-Over Fault: Output voltage failed to reach regulation within TON_MAX delay", True),
            ("ToffMax_Warn", 1, "Bit 1 (0x02) - Turn-Off Time-Over Warning: Output voltage took longer than expected to ramp down", False),
            ("Tracking_Err", 0, "Bit 0 (0x01) - Output Voltage Tracking Error between master and slave voltage rails", False)
        ]
    },
    {
        "reg": "STATUS_IOUT", "code": 0x7B, "title": "STATUS_IOUT (0x7B)",
        "chips": [
            ("IoutOC_Flt", 7, "Bit 7 (0x80) - Output Overcurrent Fault shutdown threshold exceeded (IOUT > IOUT_OC_FAULT_LIMIT)", True),
            ("IoutOC_LV_Flt", 6, "Bit 6 (0x40) - Output Overcurrent Low-Voltage Fault shutdown (Overcurrent occurred with sagging voltage)", True),
            ("IoutOC_Warn", 5, "Bit 5 (0x20) - Output Overcurrent Warning advisory threshold reached (IOUT > IOUT_OC_WARN_LIMIT)", False),
            ("IoutUC_Flt", 4, "Bit 4 (0x10) - Output Undercurrent Fault shutdown threshold reached (IOUT < IOUT_UC_FAULT_LIMIT)", True),
            ("CurShare_Flt", 3, "Bit 3 (0x08) - Current Sharing Loop Fault: Unequal current sharing among parallel power stages", False),
            ("PwrLimiting", 2, "Bit 2 (0x04) - In Power Limiting / Current Limiting active (Power stage is operating out of voltage regulation)", False),
            ("PoutOP_Flt", 1, "Bit 1 (0x02) - Output Overpower Fault shutdown threshold exceeded (POUT > POUT_OP_FAULT_LIMIT)", True),
            ("PoutOP_Warn", 0, "Bit 0 (0x01) - Output Overpower Warning advisory threshold reached (POUT > POUT_OP_WARN_LIMIT)", False)
        ]
    },
    {
        "reg": "STATUS_INPUT", "code": 0x7C, "title": "STATUS_INPUT (0x7C)",
        "chips": [
            ("VinOV_Flt", 7, "Bit 7 (0x80) - Input Overvoltage Fault shutdown threshold exceeded (VIN > VIN_OV_FAULT_LIMIT)", True),
            ("VinOV_Warn", 6, "Bit 6 (0x40) - Input Overvoltage Warning advisory threshold reached (VIN > VIN_OV_WARN_LIMIT)", False),
            ("VinUV_Warn", 5, "Bit 5 (0x20) - Input Undervoltage Warning advisory threshold reached (VIN < VIN_UV_WARN_LIMIT)", False),
            ("VinUV_Flt", 4, "Bit 4 (0x10) - Input Undervoltage Fault shutdown threshold reached (VIN < VIN_UV_FAULT_LIMIT)", True),
            ("UnitOff_LowVin", 3, "Bit 3 (0x08) - Unit Turned Off Due to Low Input Voltage: Source voltage dropped below turn-off threshold VIN_OFF", False),
            ("IinOC_Flt", 2, "Bit 2 (0x04) - Input Overcurrent Fault shutdown threshold exceeded (IIN > IIN_OC_FAULT_LIMIT)", True),
            ("IinOC_Warn", 1, "Bit 1 (0x02) - Input Overcurrent Warning advisory threshold reached (IIN > IIN_OC_WARN_LIMIT)", False),
            ("PinOP_Warn", 0, "Bit 0 (0x01) - Input Overpower Warning advisory threshold reached (PIN > PIN_OP_WARN_LIMIT)", False)
        ]
    },
    {
        "reg": "STATUS_TEMPERATURE", "code": 0x7D, "title": "STATUS_TEMPERATURE (0x7D)",
        "chips": [
            ("OT_Flt", 7, "Bit 7 (0x80) - Overtemperature Fault shutdown threshold exceeded (Temperature > OT_FAULT_LIMIT)", True),
            ("OT_Warn", 6, "Bit 6 (0x40) - Overtemperature Warning advisory threshold reached (Temperature > OT_WARN_LIMIT)", False),
            ("UT_Warn", 5, "Bit 5 (0x20) - Undertemperature Warning advisory threshold reached (Temperature < UT_WARN_LIMIT)", False),
            ("UT_Flt", 4, "Bit 4 (0x10) - Undertemperature Fault shutdown threshold reached (Temperature < UT_FAULT_LIMIT)", True),
            ("Sensor1_Flt", 3, "Bit 3 (0x08) - Thermal Sensor 1 Hardware Fault (Open circuit, short circuit or invalid ADC readout)", False),
            ("Sensor2_Flt", 2, "Bit 2 (0x04) - Thermal Sensor 2 Hardware Fault (Open circuit, short circuit or invalid ADC readout)", False),
            ("Sensor3_Flt", 1, "Bit 1 (0x02) - Thermal Sensor 3 Hardware Fault (Open circuit, short circuit or invalid ADC readout)", False)
        ]
    },
    {
        "reg": "STATUS_CML", "code": 0x7E, "title": "STATUS_CML (0x7E)",
        "chips": [
            ("BadCommand", 7, "Bit 7 (0x80) - Invalid or Unsupported PMBus Command Code received by the slave microcontroller", True),
            ("BadData", 6, "Bit 6 (0x40) - Invalid or Out-of-Range Data Parameter byte received in PMBus transaction", True),
            ("PEC_Failed", 5, "Bit 5 (0x20) - Packet Error Check (PEC / CRC-8) Mismatch Failure detected on I2C frame", True),
            ("MemCorrupt", 4, "Bit 4 (0x10) - Memory Corruption Fault: Checksum error in internal NVM, EEPROM or Flash memory", True),
            ("ProcFlt", 3, "Bit 3 (0x08) - Processor or Logic Execution Fault in internal microcontroller sequencer", True),
            ("I2C_CommErr", 1, "Bit 1 (0x02) - Other I2C / SMBus Frame Communication Fault (Bus lockup, timeout or unexpected stop)", True),
            ("OtherLogic", 0, "Bit 0 (0x01) - Other Internal Memory or Logic Sequencer Error", False)
        ]
    },
    {
        "reg": "STATUS_FANS_1_2", "code": 0x81, "title": "STATUS_FANS_1_2 (0x81)",
        "chips": [
            ("Fan1_Flt", 7, "Bit 7 (0x80) - Fan 1 Fault: Rotor locked or speed below minimum operating RPM", True),
            ("Fan2_Flt", 6, "Bit 6 (0x40) - Fan 2 Fault: Rotor locked or speed below minimum operating RPM", True),
            ("Fan1_Warn", 5, "Bit 5 (0x20) - Fan 1 Warning: Tachometer speed deviation or impending bearing failure", False),
            ("Fan2_Warn", 4, "Bit 4 (0x10) - Fan 2 Warning: Tachometer speed deviation or impending bearing failure", False),
            ("Fan1_Override", 3, "Bit 3 (0x08) - Fan 1 Speed is overridden by hardware protection circuit", False),
            ("Fan2_Override", 2, "Bit 2 (0x04) - Fan 2 Speed is overridden by hardware protection circuit", False)
        ]
    }
]

# ==============================================================================
# 2. FONCTIONS DE CONVERSION MATHÉMATIQUE PMBUS
# ==============================================================================

def twos_complement(val: int, bits: int) -> int:
    if val is None: return 0
    return val - (1 << bits) if (val & (1 << (bits - 1))) else val

def linear11_decode(raw: int) -> float:
    if raw is None or raw in (0xFFFF, 0x8000, 0x7FFF): return 0.0
    mantissa = twos_complement(raw & 0x7FF, 11)
    exponent = twos_complement((raw >> 11) & 0x1F, 5)
    return mantissa * (2.0 ** exponent)

def linear11_encode(val: float) -> int:
    if val is None or val == 0: return 0
    abs_v = abs(val)
    e = math.floor(math.log2(abs_v / 1023.0)) if abs_v > 0 else 0
    e = max(-16, min(15, e))
    m = round(val / (2.0 ** e))
    while (m > 1023 or m < -1024) and e < 15:
        e += 1
        m = round(val / (2.0 ** e))
    return ((e & 0x1F) << 11) | (m & 0x7FF)

def vout_decode(raw: int, mode_byte: int = 0x15) -> float:
    if raw is None or raw in (0xFFFF, 0x8000, 0x7FFF): return 0.0
    exp = twos_complement(mode_byte & 0x1F, 5)
    return raw * (2.0 ** exp)

def vout_encode(val: float, mode_byte: int = 0x15) -> int:
    if val is None or val <= 0: return 0
    exp = twos_complement(mode_byte & 0x1F, 5)
    raw = round(val / (2.0 ** exp))
    return max(0, min(65535, raw))

def decode_fault_response(resp_byte: int) -> str:
    if resp_byte is None or resp_byte == -1: return "-"
    act = (resp_byte >> 6) & 0x03
    retry = resp_byte & 0x07
    if act == 0x00: return "Ignore fault"
    elif act == 0x01: return "Continue w/ Delay"
    elif act == 0x02: return "Disable 'til Cleared"
    elif act == 0x03: return f"Retry (Count={retry})" if retry else "Hiccup / Retry"
    return f"0x{resp_byte:02X}"

def format_decoded_value(code: int, rtype: str, raw: int, mode_byte: int = 0x15) -> str:
    if raw is None or raw == -1: return "n/a"
    if rtype == "byte_resp": return decode_fault_response(raw)
    elif rtype == "byte_vmode":
        exp = twos_complement(raw & 0x1F, 5)
        return f"Linear (exp= {exp})"
    elif rtype == "byte_op":
        if raw == 0x80: return "ON (Nominal)"
        elif raw == 0x00: return "Immediate OFF"
        elif raw == 0x40: return "Soft OFF"
        elif raw == 0x98: return "ON (Margin High)"
        elif raw == 0x94: return "ON (Margin Low)"
        return f"0x{raw:02X}"
    elif rtype == "byte_onoff": return "Ignore all" if raw == 0x00 else f"Config (0x{raw:02X})"
    elif rtype == "byte_wp": return "DISABLE Other" if raw == 0x00 else ("Enable ALL" if raw == 0x80 else f"0x{raw:02X}")
    elif rtype == "byte_cap":
        pec = "PEC " if (raw & 0x80) else ""
        spd = "400kHz " if ((raw >> 5) & 0x03) == 1 else "100kHz "
        alert = "SMBAL#" if (raw & 0x10) else ""
        return f"{pec}{spd}{alert}".strip() or f"0x{raw:02X}"
    elif rtype == "byte_page": return f"{raw:02X}h (All)" if raw == 0xFF else f"{raw:02X}h (Ch {raw})"
    elif rtype in ["byte", "byte_hex"]: return f"0x{raw:02X}"
    elif rtype == "word_hex": return f"0x{raw:04X}"
    elif rtype == "word_vout": return f"{vout_decode(raw, mode_byte):.3f} V"
    elif rtype == "word_volt": return f"{linear11_decode(raw):.3f} V"
    elif rtype == "word_curr": return f"{linear11_decode(raw):.2f} A"
    elif rtype == "word_power": return f"{linear11_decode(raw):.2f} W"
    elif rtype == "word_temp": return f"{linear11_decode(raw):.2f} C"
    elif rtype == "word_rpm": return f"{linear11_decode(raw):.0f} RPM"
    elif rtype == "word_mohm": return f"{linear11_decode(raw):.3f} mOhm"
    elif rtype == "word_droop": return f"{linear11_decode(raw):.4f} mV/A"
    elif rtype == "word_v_ms": return f"{linear11_decode(raw):.4f} V/ms"
    elif rtype == "word_ms": return f"{linear11_decode(raw):.2f} ms"
    elif rtype == "word_percent": return f"{linear11_decode(raw):.1f} %"
    elif rtype == "word_khz": return f"{linear11_decode(raw):.1f} kHz"
    elif rtype == "word_linear_ratio": return f"{linear11_decode(raw):.4f}"
    else: return f"{linear11_decode(raw):.2f}"

# ==============================================================================
# 3. ENREGISTREUR DE DONNÉES CSV MULTI-PSU (DATA LOGGER)
# ==============================================================================

class MultiPSULogger:
    def __init__(self):
        self.is_logging = False
        self.records = []
        self._lock = threading.Lock()

    def start(self):
        with self._lock:
            self.records = []
            self.is_logging = True

    def stop(self):
        with self._lock:
            self.is_logging = False

    def log_snapshot(self, overview_data: dict):
        if not self.is_logging or not overview_data: return
        with self._lock:
            row = {
                "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "Total_PIN_W": overview_data.get("total_pin", 0),
                "Total_POUT_W": overview_data.get("total_pout", 0),
                "Total_IOUT_A": overview_data.get("total_iout", 0),
                "Global_Eff_Pct": overview_data.get("global_efficiency", 0),
                "Active_Faults_Total": overview_data.get("total_active_faults", 0)
            }
            for i, psu in enumerate(overview_data.get("psus", [])):
                p_id = f"PSU{i+1}_{psu.get('addr_hex', '0x00')}"
                row[f"{p_id}_State"] = "ON" if psu.get("power_state") else "OFF"
                row[f"{p_id}_VIN_V"] = psu.get("vin", 0)
                row[f"{p_id}_VOUT_V"] = psu.get("vout", 0)
                row[f"{p_id}_IIN_A"] = psu.get("iin", 0)
                row[f"{p_id}_IOUT_A"] = psu.get("iout", 0)
                row[f"{p_id}_PIN_W"] = psu.get("pin", 0)
                row[f"{p_id}_POUT_W"] = psu.get("pout", 0)
                row[f"{p_id}_Temp1_C"] = psu.get("temp1", 0)
                row[f"{p_id}_Temp2_C"] = psu.get("temp2", 0)
                row[f"{p_id}_Fan1_RPM"] = psu.get("fan1_rpm", 0)

            self.records.append(row)
            if len(self.records) > 10000:
                self.records.pop(0)

    def get_csv(self) -> str:
        with self._lock:
            if not self.records:
                return "Timestamp,Total_PIN_W,Total_POUT_W,Total_IOUT_A,Global_Eff_Pct,Active_Faults_Total\n"
            out = io.StringIO()
            w = csv.DictWriter(out, fieldnames=list(self.records[0].keys()))
            w.writeheader()
            w.writerows(self.records)
            return out.getvalue()

# ==============================================================================
# 4. ABSTRACTION MATÉRIELLE & PILOTE INDIVIDUEL DE PSU
# ==============================================================================

class PSUDevice:
    """Représente une alimentation (PSU) physique ou émulée sur le bus PMBus."""

    def __init__(self, bus_id: int, addr: int, is_mock: bool = False, slot_num: int = 1):
        self.bus_id = bus_id
        self.addr = addr
        self.is_mock = is_mock
        self.slot_num = slot_num
        self.bus = None
        self._lock = threading.Lock()
        self.noise_seed = (slot_num * 17.3) + addr

        # Cache d'état (Sample & Hold)
        self._cache = {
            "vin": 230.0 if not is_mock else (230.0 + (slot_num % 3) * 1.2),
            "vout": 12.0,
            "iin": 0.0,
            "iout": 0.0,
            "pin": 0.0,
            "pout": 0.0,
            "eff": 0.0,
            "temp1": 38.0 + slot_num * 1.5,
            "temp2": 42.0 + slot_num * 2.0,
            "temp3": 28.0,
            "fan1_rpm": 2400 + slot_num * 150,
            "fan2_rpm": 2400 + slot_num * 150,
            "op": 0x80 if is_mock else 0xFF,
            "mode": 0x15, # Linear16 exp=-11
            "sw": 0xFFFF,
            "status_vout": 0x00,
            "status_iout": 0x00,
            "status_input": 0x00,
            "status_temp": 0x00,
            "status_cml": 0x00,
            "status_fans": 0x00,
            "vout_cmd": 12.00,
            "vin_on": 180.0,
            "vin_off": 160.0,
            "ton_rise": 15.0,
            "ton_delay": 0.0,
            "pg_on": 11.2,
            "pg_off": 10.5,
            "mfr_id": f"STANDARD_PWR_{slot_num}",
            "mfr_model": f"CRPS-3000W-V{slot_num}",
            "mfr_revision": "REV-2.4",
            "mfr_serial": f"SN-PSU-{addr:02X}-{1000 + slot_num}",
            "mfr_date": "260615",
            "mfr_location": "FACILITY-A",
            "pmbus_rev": "0x22 (v1.2)"
        }

        if not self.is_mock:
            self._connect_hw()

    def _connect_hw(self):
        try:
            from smbus2 import SMBus
            self.bus = SMBus(self.bus_id)
        except Exception:
            self.bus = None
            self.is_mock = True

    def read_byte_retry(self, reg: int, default: int = 0) -> int:
        if self.is_mock or not self.bus:
            return self._mock_read_byte(reg)
        for attempt in range(2):
            try:
                val = self.bus.read_byte_data(self.addr, reg)
                time.sleep(0.003) # Pacing delay
                return val
            except Exception:
                time.sleep(0.005)
        return default

    def read_word_retry(self, reg: int, default: int = 0) -> int:
        if self.is_mock or not self.bus:
            return self._mock_read_word(reg)
        for attempt in range(2):
            try:
                val = self.bus.read_word_data(self.addr, reg)
                time.sleep(0.003) # Pacing delay
                return val
            except Exception:
                time.sleep(0.005)
        return default

    def write_byte_retry(self, reg: int, val: int) -> bool:
        if self.is_mock or not self.bus:
            self._mock_write_byte(reg, val)
            return True
        for attempt in range(2):
            try:
                self.bus.write_byte_data(self.addr, reg, val & 0xFF)
                time.sleep(0.01)
                return True
            except Exception:
                time.sleep(0.01)
        return False

    def write_word_retry(self, reg: int, val: int) -> bool:
        if self.is_mock or not self.bus:
            self._mock_write_word(reg, val)
            return True
        for attempt in range(2):
            try:
                self.bus.write_word_data(self.addr, reg, val & 0xFFFF)
                time.sleep(0.01)
                return True
            except Exception:
                time.sleep(0.01)
        return False

    def read_block_ascii(self, reg: int, default: str = "n/a") -> str:
        if self.is_mock or not self.bus:
            return self._mock_read_ascii(reg)
        try:
            data = self.bus.read_i2c_block_data(self.addr, reg, 16)
            time.sleep(0.005)
            if data:
                length = data[0]
                if 0 < length <= 15:
                    s = "".join([chr(c) for c in data[1:1+length] if 32 <= c <= 126])
                    if s.strip(): return s.strip()
                s = "".join([chr(c) for c in data if 32 <= c <= 126])
                if s.strip(): return s.strip()
        except Exception:
            pass
        return default

    # --- SIMULATEUR PHYSIQUE (MOCK) ---
    def _mock_read_byte(self, reg: int) -> int:
        if reg == 0x00: return 0 # PAGE
        if reg == 0x01: return self._cache["op"]
        if reg == 0x20: return self._cache["mode"]
        if reg == 0x78: return self._cache["sw"] & 0xFF
        if reg == 0x7A: return self._cache["status_vout"]
        if reg == 0x7B: return self._cache["status_iout"]
        if reg == 0x7C: return self._cache["status_input"]
        if reg == 0x7D: return self._cache["status_temp"]
        if reg == 0x7E: return self._cache["status_cml"]
        if reg == 0x81: return self._cache["status_fans"]
        if reg == 0x98: return 0x22 # PMBus 1.2
        return 0x00

    def _mock_write_byte(self, reg: int, val: int):
        if reg == 0x01: self._cache["op"] = val

    def _mock_read_word(self, reg: int) -> int:
        t = time.time() + self.noise_seed
        noise = math.sin(t * 1.8) * 0.04
        is_on = (self._cache["op"] in [0x80, 0x88, 0x98, 0xA8])

        if reg == 0x79: return self._cache["sw"]
        if reg == 0x21: return vout_encode(self._cache["vout_cmd"], self._cache["mode"])
        if reg == 0x35: return linear11_encode(self._cache["vin_on"])
        if reg == 0x36: return linear11_encode(self._cache["vin_off"])
        if reg == 0x61: return linear11_encode(self._cache["ton_rise"])
        if reg == 0x5E: return vout_encode(self._cache["pg_on"], self._cache["mode"])
        if reg == 0x5F: return vout_encode(self._cache["pg_off"], self._cache["mode"])

        if reg == 0x88: # READ_VIN
            vin = 230.0 + math.sin(t * 0.3) * 1.5 + noise * 5.0
            return linear11_encode(vin)
        if reg == 0x8B: # READ_VOUT
            vout = (self._cache["vout_cmd"] + noise) if is_on else 0.05
            return vout_encode(vout, self._cache["mode"])
        if reg == 0x8C: # READ_IOUT
            # Charge équilibrée entre 35A et 120A par PSU
            iout = (65.0 + math.sin(t * 0.5 + self.slot_num) * 25.0 + noise * 10.0) if is_on else 0.0
            return linear11_encode(max(0.0, iout))
        if reg == 0x96: # READ_POUT
            vout = self._cache["vout_cmd"] if is_on else 0.0
            iout = (65.0 + math.sin(t * 0.5 + self.slot_num) * 25.0) if is_on else 0.0
            return linear11_encode(vout * iout)
        if reg == 0x97: # READ_PIN
            vout = self._cache["vout_cmd"] if is_on else 0.0
            iout = (65.0 + math.sin(t * 0.5 + self.slot_num) * 25.0) if is_on else 0.0
            pout = vout * iout
            pin = (pout / 0.94) + (18.0 if is_on else 3.5)
            return linear11_encode(pin)
        if reg == 0x89: # READ_IIN
            vin = 230.0
            vout = self._cache["vout_cmd"] if is_on else 0.0
            iout = (65.0 + math.sin(t * 0.5 + self.slot_num) * 25.0) if is_on else 0.0
            pin = ((vout * iout) / 0.94) + (18.0 if is_on else 3.5)
            return linear11_encode(pin / vin)
        if reg == 0x8D: # READ_TEMPERATURE_1
            t1 = (42.0 + (self.slot_num * 2.0) + math.sin(t * 0.1) * 3.5) if is_on else 27.0
            return linear11_encode(t1)
        if reg == 0x8E: # READ_TEMPERATURE_2
            t2 = (46.0 + (self.slot_num * 1.8) + math.sin(t * 0.08) * 4.0) if is_on else 28.0
            return linear11_encode(t2)
        if reg == 0x8F: # READ_TEMPERATURE_3
            return linear11_encode(26.5 + math.sin(t * 0.05) * 1.2)
        if reg == 0x90: # READ_FAN_SPEED_1
            rpm = (3200 + (self.slot_num * 120) + math.sin(t * 0.4) * 200) if is_on else 1200
            return linear11_encode(rpm)
        if reg == 0x91: # READ_FAN_SPEED_2
            rpm = (3150 + (self.slot_num * 110) + math.sin(t * 0.4) * 180) if is_on else 1200
            return linear11_encode(rpm)

        return 0x0000

    def _mock_write_word(self, reg: int, val: int):
        if reg == 0x21: self._cache["vout_cmd"] = vout_decode(val, self._cache["mode"])
        elif reg == 0x35: self._cache["vin_on"] = linear11_decode(val)
        elif reg == 0x36: self._cache["vin_off"] = linear11_decode(val)
        elif reg == 0x61: self._cache["ton_rise"] = linear11_decode(val)

    def _mock_read_ascii(self, reg: int) -> str:
        if reg == 0x99: return self._cache["mfr_id"]
        if reg == 0x9A: return self._cache["mfr_model"]
        if reg == 0x9B: return self._cache["mfr_revision"]
        if reg == 0x9C: return self._cache["mfr_location"]
        if reg == 0x9D: return self._cache["mfr_date"]
        if reg == 0x9E: return self._cache["mfr_serial"]
        return "n/a"

    # --- COMMANDES ET CONTROLES DE LA PSU ---
    def clear_faults(self) -> bool:
        with self._lock:
            if not self.is_mock and self.bus:
                try:
                    self.bus.write_byte(self.addr, 0x03)
                    time.sleep(0.1)
                except Exception:
                    try:
                        self.bus.write_byte_data(self.addr, 0x03, 0x00)
                        time.sleep(0.1)
                    except Exception:
                        return False
            self._cache["sw"] = 0x0000
            self._cache["status_vout"] = 0x00
            self._cache["status_iout"] = 0x00
            self._cache["status_input"] = 0x00
            self._cache["status_temp"] = 0x00
            self._cache["status_cml"] = 0x00
            self._cache["status_fans"] = 0x00
            return True

    def set_operation(self, op_code: int) -> bool:
        with self._lock:
            ok = self.write_byte_retry(0x01, op_code)
            if ok: self._cache["op"] = op_code
            return ok

    def set_vout(self, target_v: float) -> bool:
        with self._lock:
            mode = self.read_byte_retry(0x20, self._cache["mode"])
            raw = vout_encode(target_v, mode)
            ok = self.write_word_retry(0x21, raw)
            if ok: self._cache["vout_cmd"] = target_v
            return ok

    def set_startup_thresholds(self, vin_on: float = None, vin_off: float = None, ton_rise: float = None) -> bool:
        with self._lock:
            if vin_on is not None:
                self.write_word_retry(0x35, linear11_encode(vin_on))
                self._cache["vin_on"] = vin_on
            if vin_off is not None:
                self.write_word_retry(0x36, linear11_encode(vin_off))
                self._cache["vin_off"] = vin_off
            if ton_rise is not None:
                self.write_word_retry(0x61, linear11_encode(ton_rise))
                self._cache["ton_rise"] = ton_rise
            return True

    # --- LECTURE COMPLÈTE DE LA TÉLÉMÉTRIE ---
    def get_telemetry(self) -> dict:
        with self._lock:
            mode = self.read_byte_retry(0x20, self._cache["mode"])
            self._cache["mode"] = mode

            raw_vin = self.read_word_retry(0x88, None)
            if raw_vin is not None:
                val = linear11_decode(raw_vin)
                self._cache["vin"] = val if 0 <= val <= 600 else 0.0

            raw_vout = self.read_word_retry(0x8B, None)
            if raw_vout is not None:
                val = vout_decode(raw_vout, mode)
                self._cache["vout"] = val if 0 <= val <= 100 else 0.0

            raw_iin = self.read_word_retry(0x89, None)
            if raw_iin is not None:
                val = linear11_decode(raw_iin)
                self._cache["iin"] = val if -50 <= val <= 500 else 0.0

            raw_iout = self.read_word_retry(0x8C, None)
            if raw_iout is not None:
                val = linear11_decode(raw_iout)
                self._cache["iout"] = val if -50 <= val <= 1000 else 0.0

            raw_pout = self.read_word_retry(0x96, None)
            if raw_pout is not None and raw_pout not in (0, 0xFFFF):
                val = linear11_decode(raw_pout)
                self._cache["pout"] = val if -500 <= val <= 10000 else 0.0
            else:
                self._cache["pout"] = self._cache["vout"] * self._cache["iout"]

            raw_pin = self.read_word_retry(0x97, None)
            if raw_pin is not None and raw_pin not in (0, 0xFFFF):
                val = linear11_decode(raw_pin)
                self._cache["pin"] = val if -500 <= val <= 12000 else 0.0
            else:
                self._cache["pin"] = self._cache["vin"] * self._cache["iin"]

            raw_t1 = self.read_word_retry(0x8D, None)
            if raw_t1 is not None:
                val = linear11_decode(raw_t1)
                self._cache["temp1"] = val if -40 <= val <= 200 else 0.0
                
            raw_t2 = self.read_word_retry(0x8E, None)
            if raw_t2 is not None:
                val = linear11_decode(raw_t2)
                self._cache["temp2"] = val if -40 <= val <= 200 else 0.0
                
            raw_t3 = self.read_word_retry(0x8F, None)
            if raw_t3 is not None:
                val = linear11_decode(raw_t3)
                self._cache["temp3"] = val if -40 <= val <= 200 else 0.0

            raw_fan1 = self.read_word_retry(0x90, None)
            if raw_fan1 is not None:
                val = linear11_decode(raw_fan1)
                self._cache["fan1_rpm"] = val if 0 <= val <= 30000 else 0.0
                
            raw_fan2 = self.read_word_retry(0x91, None)
            if raw_fan2 is not None:
                val = linear11_decode(raw_fan2)
                self._cache["fan2_rpm"] = val if 0 <= val <= 30000 else 0.0

            raw_op = self.read_byte_retry(0x01, None)
            if raw_op is not None: self._cache["op"] = raw_op

            # Statuts
            self._cache["sw"] = self.read_word_retry(0x79, self._cache["sw"])
            self._cache["status_vout"] = self.read_byte_retry(0x7A, 0x00)
            self._cache["status_iout"] = self.read_byte_retry(0x7B, 0x00)
            self._cache["status_input"] = self.read_byte_retry(0x7C, 0x00)
            self._cache["status_temp"] = self.read_byte_retry(0x7D, 0x00)
            self._cache["status_cml"] = self.read_byte_retry(0x7E, 0x00)
            self._cache["status_fans"] = self.read_byte_retry(0x81, 0x00)

            # Rendement
            pin = self._cache["pin"]
            pout = self._cache["pout"]
            eff = min(100.0, max(0.0, (pout / pin) * 100.0)) if pin > 10.0 and pout > 0 else 0.0

            # Décodage des status strips
            status_vals = {
                "STATUS_BYTE": self._cache["sw"] & 0xFF,
                "STATUS_WORD": self._cache["sw"],
                "STATUS_VOUT": self._cache["status_vout"],
                "STATUS_IOUT": self._cache["status_iout"],
                "STATUS_INPUT": self._cache["status_input"],
                "STATUS_TEMPERATURE": self._cache["status_temp"],
                "STATUS_CML": self._cache["status_cml"],
                "STATUS_FANS_1_2": self._cache["status_fans"]
            }

            status_strips = []
            active_faults_total = 0
            critical_faults = 0

            for reg_info in PMBUS_STATUS_CHIPS:
                r_name = reg_info["reg"]
                r_val = status_vals.get(r_name, 0)
                chips_out = []
                for label, bit, desc, is_crit in reg_info["chips"]:
                    is_set = bool(r_val & (1 << bit))
                    if is_set:
                        active_faults_total += 1
                        if is_crit: critical_faults += 1
                    chips_out.append({
                        "label": label,
                        "bit": bit,
                        "reg": r_name,
                        "desc": desc,
                        "active": is_set,
                        "critical": is_crit
                    })
                status_strips.append({
                    "reg": r_name,
                    "code_hex": f"0x{reg_info['code']:02X}",
                    "title": reg_info["title"],
                    "val_hex": f"0x{r_val:04X}" if r_name == "STATUS_WORD" else f"0x{r_val:02X}",
                    "chips": chips_out
                })

            op = self._cache["op"]
            sw = self._cache["sw"]
            # En PMBus 1.0/1.1, STATUS_WORD bit 6 = Unit Off. S'il est à 0, l'unité est ON.
            # OPERATION = 0x80 signifie ON. 
            # VOUT > 0.5V confirme que la puissance sort.
            status_word_off = bool(sw & 0x0040)
            is_powered = (self._cache["vout"] > 0.5) or (op in [0x80, 0x88, 0x98, 0xA8]) or (not status_word_off and sw != 0xFFFF)

            return {
                "bus_id": self.bus_id,
                "addr_hex": f"{self.bus_id}_0x{self.addr:02X}",
                "slot_num": self.slot_num,
                "is_mock": self.is_mock,
                "power_state": is_powered,
                "op_code": op,
                "vin": round(self._cache["vin"], 2),
                "vout": round(self._cache["vout"], 3),
                "iin": round(self._cache["iin"], 2),
                "iout": round(self._cache["iout"], 2),
                "pin": round(self._cache["pin"], 1),
                "pout": round(self._cache["pout"], 1),
                "efficiency": round(eff, 1),
                "temp1": round(self._cache["temp1"], 1),
                "temp2": round(self._cache["temp2"], 1),
                "temp3": round(self._cache["temp3"], 1),
                "fan1_rpm": round(self._cache["fan1_rpm"]),
                "fan2_rpm": round(self._cache["fan2_rpm"]),
                "status_word": f"0x{self._cache['sw']:04X}",
                "status_strips": status_strips,
                "active_faults": active_faults_total,
                "critical_faults": critical_faults,
                "mfr": {
                    "id": self.read_block_ascii(0x99, self._cache["mfr_id"]),
                    "model": self.read_block_ascii(0x9A, self._cache["mfr_model"]),
                    "revision": self.read_block_ascii(0x9B, self._cache["mfr_revision"]),
                    "location": self.read_block_ascii(0x9C, self._cache["mfr_location"]),
                    "date": self.read_block_ascii(0x9D, self._cache["mfr_date"]),
                    "serial": self.read_block_ascii(0x9E, self._cache["mfr_serial"]),
                    "pmbus_rev": f"0x{self.read_byte_retry(0x98, 0x22):02X}"
                },
                "startup": {
                    "vin_on": round(self._cache["vin_on"], 2),
                    "vin_off": round(self._cache["vin_off"], 2),
                    "ton_rise": round(self._cache["ton_rise"], 2),
                    "vout_cmd": round(self._cache["vout_cmd"], 2)
                }
            }

    def get_registers(self) -> list:
        regs = []
        mode = self.read_byte_retry(0x20, self._cache["mode"])
        for r in PMBUS_SAFE_REGISTERS:
            code = r["code"]
            rtype = r["type"]
            val_str = "n/a"
            try:
                if rtype == "block_ascii":
                    val_str = self.read_block_ascii(code, "n/a")
                elif rtype.startswith("byte"):
                    raw = self.read_byte_retry(code, -1)
                    val_str = format_decoded_value(code, rtype, raw, mode)
                else:
                    raw = self.read_word_retry(code, -1)
                    val_str = format_decoded_value(code, rtype, raw, mode)
            except Exception:
                pass
            regs.append({"name": r["name"], "hex": f"0x{code:02X}", "cat": r["cat"], "desc": r["desc"], "val": val_str})
        return regs

# ==============================================================================
# 5. GESTIONNAIRE MULTI-PSU (SCAN & AGGREGATION)
# ==============================================================================

def detect_available_i2c_buses() -> list:
    """Détecte les bus I2C disposant d'équipements PMBus (Linux)."""
    buses = []
    if os.path.exists('/dev'):
        try:
            from smbus2 import SMBus
        except ImportError:
            SMBus = None
            
        for f in os.listdir('/dev'):
            if f.startswith('i2c-'):
                try:
                    num = int(f.split('-')[1])
                    if num > 0 and num not in buses:
                        # Si SMBus dispo, on teste vite fait s'il y a du PMBus sur ce bus
                        has_pmbus = False
                        if SMBus:
                            try:
                                with SMBus(num) as s:
                                    for addr in (0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x50):
                                        try:
                                            s.read_byte_data(addr, 0x78)
                                            has_pmbus = True
                                            break
                                        except Exception:
                                            pass
                            except Exception:
                                pass
                        else:
                            has_pmbus = True # Fallback si pas de lib
                            
                        if has_pmbus:
                            buses.append(num)
                except Exception:
                    pass
    if not buses:
        buses = [2, 1, 0] # Bus 2 est le bus standard P9 sur BeagleBone Black
    return sorted(buses)

class MultiPSUManager:
    """Gère le banc multi-alimentations I2C/PMBus (1 à 5+ PSU)."""

    def __init__(self, bus_id: int = 2, force_mock: bool = False):
        self.bus_id = bus_id
        self.force_mock = force_mock
        self.psus = {} # addr_int -> PSUDevice
        self.logger = MultiPSULogger()
        self._lock = threading.Lock()
        self.scan_and_init(bus_id)

    def scan_and_init(self, bus_id_or_list):
        with self._lock:
            self.psus.clear()

            if isinstance(bus_id_or_list, list):
                buses_to_scan = bus_id_or_list
                if buses_to_scan: self.bus_id = buses_to_scan[0]
            else:
                buses_to_scan = [bus_id_or_list] if bus_id_or_list is not None else [self.bus_id]
                self.bus_id = buses_to_scan[0]

            detected_devices = []
            if not self.force_mock:
                for b in buses_to_scan:
                    detected_devices.extend(self._hardware_scan(b))

            # Si aucune PSU physique n'est détectée ou si Mock forcé : émuler 5 PSUs réalistes
            if not detected_devices:
                print(f"[INFO] Aucune PSU physique détectée. Initialisation de 5 PSU en Mode Simulation (Mock).")
                # Adresses PMBus standards (0x58 à 0x5C)
                mock_addrs = [0x58, 0x59, 0x5A, 0x5B, 0x5C]
                for idx, addr in enumerate(mock_addrs):
                    key = f"{self.bus_id}_0x{addr:02X}"
                    self.psus[key] = PSUDevice(bus_id=self.bus_id, addr=addr, is_mock=True, slot_num=idx+1)
            else:
                desc = ", ".join([f"Bus {d['bus']} @ 0x{d['addr']:02X}" for d in detected_devices])
                print(f"[INFO] {len(detected_devices)} PSU(s) PMBus détectée(s) : {desc}")
                for idx, d in enumerate(detected_devices):
                    key = f"{d['bus']}_0x{d['addr']:02X}"
                    self.psus[key] = PSUDevice(bus_id=d['bus'], addr=d['addr'], is_mock=False, slot_num=idx+1)

    def _hardware_scan(self, bus_num: int) -> list:
        found = []
        try:
            from smbus2 import SMBus
            with SMBus(bus_num) as s:
                # Plage d'adresses standards strictes pour PSU serveur PMBus (0x58 à 0x67)
                # On exclut 0x50-0x57 car ce sont des adresses réservées aux EEPROMs (mémoires) 
                # qui répondent toujours et créent des "fausses PSUs".
                for addr in range(0x58, 0x68):
                    try:
                        status = s.read_byte_data(addr, 0x78)
                        vout_raw = s.read_word_data(addr, 0x8B)
                        
                        if status != 0xFF and vout_raw not in (0xFFFF, 0x7FFF):
                            if status == 0x00 and vout_raw == 0x0000:
                                vin_raw = s.read_word_data(addr, 0x88)
                                if vin_raw == 0x0000:
                                    continue
                                    
                            found.append({"bus": bus_num, "addr": addr})
                    except Exception:
                        pass
        except Exception:
            pass
        return found

    def get_overview(self) -> dict:
        with self._lock:
            psu_list = []
            total_pin = 0.0
            total_pout = 0.0
            total_iout = 0.0
            total_faults = 0
            any_on = False

            for addr in sorted(self.psus.keys()):
                dev = self.psus[addr]
                t = dev.get_telemetry()
                psu_list.append(t)
                total_pin += t.get("pin", 0.0)
                total_pout += t.get("pout", 0.0)
                total_iout += t.get("iout", 0.0)
                total_faults += t.get("active_faults", 0)
                if t.get("power_state"): any_on = True

            global_eff = min(100.0, max(0.0, (total_pout / total_pin) * 100.0)) if total_pin > 20.0 and total_pout > 0 else 0.0

            overview = {
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "bus_id": self.bus_id,
                "psu_count": len(psu_list),
                "is_mock": any(p.get("is_mock") for p in psu_list),
                "global_power_state": any_on,
                "total_pin": round(total_pin, 1),
                "total_pout": round(total_pout, 1),
                "total_iout": round(total_iout, 2),
                "global_efficiency": round(global_eff, 1),
                "total_active_faults": total_faults,
                "psus": psu_list,
                "logger": {
                    "is_logging": self.logger.is_logging,
                    "records_count": len(self.logger.records)
                }
            }

            if self.logger.is_logging:
                self.logger.log_snapshot(overview)

            return overview

    def get_psu_telemetry(self, addr_key: str) -> dict:
        with self._lock:
            if addr_key in self.psus:
                return self.psus[addr_key].get_telemetry()
            return {"error": f"PSU {addr_key} non trouvée"}

    def get_psu_registers(self, addr_key: str) -> list:
        with self._lock:
            if addr_key in self.psus:
                return self.psus[addr_key].get_registers()
            return []

    def clear_all_faults(self) -> bool:
        with self._lock:
            for dev in self.psus.values():
                dev.clear_faults()
            return True

    def power_all(self, state: bool) -> bool:
        with self._lock:
            op_code = 0x80 if state else 0x00
            for dev in self.psus.values():
                dev.set_operation(op_code)
            return True

# ==============================================================================
# 6. SERVEUR HTTP REST & STREAMING TEMPS RÉEL SSE
# ==============================================================================

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class PMBusMultiApiHandler(SimpleHTTPRequestHandler):
    manager: MultiPSUManager = None

    def log_message(self, format, *args):
        pass # Console silencieuse pour garder la lisibilité

    def send_json(self, data: dict, status_code: int = 200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/api/buses":
            self.send_json({"buses": detect_available_i2c_buses(), "current_bus": self.manager.bus_id})
            return

        elif path == "/api/scan":
            bus_param = query.get("bus", [str(self.manager.bus_id)])[0]
            if bus_param == "all":
                bus_ids = detect_available_i2c_buses()
                self.manager.scan_and_init(bus_ids)
            else:
                try: bus_id = int(bus_param)
                except Exception: bus_id = self.manager.bus_id
                self.manager.scan_and_init(bus_id)
                
            self.send_json(self.manager.get_overview())
            return

        elif path == "/api/overview":
            self.send_json(self.manager.get_overview())
            return

        elif path == "/api/telemetry":
            addr_key = query.get("addr", ["2_0x58"])[0]
            self.send_json(self.manager.get_psu_telemetry(addr_key))
            return

        elif path == "/api/registers":
            addr_key = query.get("addr", ["2_0x58"])[0]
            self.send_json({"addr": addr_key, "registers": self.manager.get_psu_registers(addr_key)})
            return

        elif path == "/api/logger/export":
            csv_data = self.manager.logger.get_csv().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", f"attachment; filename=pmbus_multi_psu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            self.send_header("Content-Length", str(len(csv_data)))
            self.end_headers()
            self.wfile.write(csv_data)
            return

        # FLUX TEMPS REEL SERVER-SENT EVENTS (SSE)
        elif path == "/api/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            try:
                while True:
                    data = self.manager.get_overview()
                    payload = f"data: {json.dumps(data)}\n\n".encode("utf-8")
                    self.wfile.write(payload)
                    self.wfile.flush()
                    time.sleep(0.5) # 2 Hz
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        # INTERFACE WEB EMBARQUEE (HTML/CSS/JS)
        elif path == "/" or path == "/index.html" or path == "":
            html_bytes = HTML_DASHBOARD.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html_bytes)))
            self.end_headers()
            self.wfile.write(html_bytes)
            return

        self.send_json({"error": "Endpoint introuvable"}, 404)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        length = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(length) if length > 0 else b"{}"
        try: body = json.loads(post_data.decode("utf-8")) if post_data else {}
        except Exception: body = {}

        if path == "/api/action":
            action = body.get("action")
            addr_key = body.get("addr", "2_0x58")

            if action == "clear_all_faults":
                ok = self.manager.clear_all_faults()
                self.send_json({"status": "ok" if ok else "error", "action": "clear_all_faults"})
                return

            elif action == "power_all":
                state = body.get("state", True)
                ok = self.manager.power_all(state)
                self.send_json({"status": "ok" if ok else "error", "power_state": state})
                return

            elif action == "clear_faults":
                if addr_key in self.manager.psus:
                    ok = self.manager.psus[addr_key].clear_faults()
                    self.send_json({"status": "ok" if ok else "error", "addr": addr_key})
                else: self.send_json({"status": "error", "error": "PSU non trouvée"}, 404)
                return

            elif action == "set_operation":
                op_code = body.get("op", 0x80)
                if addr_key in self.manager.psus:
                    ok = self.manager.psus[addr_key].set_operation(int(op_code))
                    self.send_json({"status": "ok" if ok else "error", "addr": addr_key, "op": op_code})
                else: self.send_json({"status": "error", "error": "PSU non trouvée"}, 404)
                return

            elif action == "set_vout":
                vout = body.get("vout", 12.0)
                if addr_key in self.manager.psus:
                    ok = self.manager.psus[addr_key].set_vout(float(vout))
                    self.send_json({"status": "ok" if ok else "error", "addr": addr_key, "vout": vout})
                else: self.send_json({"status": "error", "error": "PSU non trouvée"}, 404)
                return

            elif action == "set_startup":
                vin_on = body.get("vin_on")
                vin_off = body.get("vin_off")
                ton_rise = body.get("ton_rise")
                if addr_key in self.manager.psus:
                    ok = self.manager.psus[addr_key].set_startup_thresholds(vin_on, vin_off, ton_rise)
                    self.send_json({"status": "ok" if ok else "error", "addr": addr_key})
                else: self.send_json({"status": "error", "error": "PSU non trouvée"}, 404)
                return

            elif action == "logger_start":
                self.manager.logger.start()
                self.send_json({"status": "ok", "logging": True})
                return

            elif action == "logger_stop":
                self.manager.logger.stop()
                self.send_json({"status": "ok", "logging": False})
                return

        self.send_json({"error": "Endpoint introuvable"}, 404)

# ==============================================================================
# 7. INTERFACE GRAPHIQUE WEB EMBARQUÉE (HTML5 / CSS3 / JAVASCRIPT VANILLA)
# ==============================================================================

HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PMBus Multi-PSU Power Dashboard</title>
<style>
:root {
    --bg: #090d16;
    --card: #111827;
    --card-hover: #1f293d;
    --input: #0b1120;
    --border: #1e293b;
    --text: #f8fafc;
    --muted: #94a3b8;
    --cyan: #06b6d4;
    --blue: #38bdf8;
    --green: #22c55e;
    --amber: #f59e0b;
    --red: #ef4444;
    --purple: #c084fc;
    --indigo: #818cf8;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, sans-serif; min-height: 100vh; display: flex; flex-direction: column; }
header { background: var(--card); padding: 12px 24px; display: flex; align-items: center; justify-content: space-between; border-bottom: 1px solid var(--border); flex-wrap: wrap; gap: 12px; }
.brand { display: flex; align-items: center; gap: 12px; }
.logo-icon { width: 38px; height: 38px; border-radius: 8px; background: linear-gradient(135deg, #06b6d4, #3b82f6); display: flex; align-items: center; justify-content: center; font-weight: 900; font-size: 14px; color: #fff; box-shadow: 0 0 15px rgba(6,182,212,0.4); }

.nav-tabs { display: flex; gap: 8px; flex-wrap: wrap; }
.tab-btn { background: none; border: 1px solid transparent; color: var(--muted); padding: 8px 16px; border-radius: 6px; font-weight: 700; cursor: pointer; transition: all 0.2s; display: inline-flex; align-items: center; gap: 8px; font-size: 13px; }
.tab-btn:hover { color: #fff; background: #1e293b; }
.tab-btn.active { background: #1e293b; color: #fff; border-color: #334155; }

.conn-badge { display: inline-flex; align-items: center; gap: 6px; padding: 5px 12px; border-radius: 20px; font-family: monospace; font-size: 11px; font-weight: 700; background: #0b1120; border: 1px solid var(--border); color: #94a3b8; transition: all 0.3s; }
.conn-dot { width: 8px; height: 8px; border-radius: 50%; background: #64748b; }
.conn-badge.live { border-color: #16a34a; background: #14532d; color: #86efac; }
.conn-badge.live .conn-dot { background: #22c55e; box-shadow: 0 0 8px #22c55e; }
.conn-badge.reconnecting { border-color: #d97706; background: #78350f; color: #fde68a; }
.conn-badge.reconnecting .conn-dot { background: #f59e0b; animation: pulse 0.8s infinite; }
.conn-badge.offline { border-color: #dc2626; background: #7f1d1d; color: #fca5a5; }
.conn-badge.offline .conn-dot { background: #ef4444; }

@keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.4; } }

.main-wrap { max-width: 1600px; width: 100%; margin: 0 auto; padding: 20px; flex: 1; }
.view-panel { display: none; }
.view-panel.active { display: block; }

.hero-bar { background: var(--card); padding: 16px 20px; border-radius: 8px; border: 1px solid var(--border); display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 14px; margin-bottom: 20px; }
.btn { background: #2563eb; color: #fff; border: none; padding: 8px 16px; border-radius: 6px; font-weight: 700; cursor: pointer; display: inline-flex; align-items: center; gap: 6px; transition: all 0.2s; font-size: 13px; }
.btn:hover { filter: brightness(1.15); transform: translateY(-1px); }
.btn-green { background: #16a34a; }
.btn-red { background: #dc2626; }
.btn-gray { background: #1e293b; border: 1px solid #334155; color: #cbd5e1; }
.btn-amber { background: #d97706; }

select, input { background: var(--input); border: 1px solid var(--border); color: #fff; padding: 7px 12px; border-radius: 6px; font-family: monospace; font-size: 13px; }

/* KPI Grid Styles */
.kpi-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 14px; margin-bottom: 20px; }
.kpi-card { background: var(--card); padding: 16px; border-radius: 8px; border: 1px solid var(--border); position: relative; overflow: hidden; }
.kpi-card span { font-size: 11px; color: var(--muted); text-transform: uppercase; font-weight: 700; letter-spacing: 0.5px; }
.kpi-card strong { font-family: monospace; font-size: 24px; display: block; margin-top: 6px; }

/* Multi-PSU Grid Cards */
.psu-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(290px, 1fr)); gap: 16px; margin-bottom: 24px; }
.psu-card { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 18px; transition: all 0.2s; position: relative; cursor: pointer; }
.psu-card:hover { border-color: var(--cyan); transform: translateY(-2px); background: var(--card-hover); box-shadow: 0 8px 25px rgba(0,0,0,0.4); }
.psu-card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; border-bottom: 1px solid #1e293b; padding-bottom: 8px; }
.badge-state { font-family: monospace; font-size: 11px; font-weight: 800; padding: 4px 8px; border-radius: 4px; background: #14532d; color: #86efac; border: 1px solid #16a34a; }
.badge-state.off { background: #7f1d1d; color: #fca5a5; border-color: #dc2626; }
.badge-fault { font-family: monospace; font-size: 11px; font-weight: 800; padding: 4px 8px; border-radius: 4px; background: #991b1b; color: #fecaca; border: 1px solid #ef4444; animation: pulse 1s infinite; }

.psu-metric-row { display: flex; justify-content: space-between; font-size: 12px; margin-bottom: 6px; font-family: monospace; }
.psu-metric-row .lbl { color: var(--muted); }
.psu-metric-row .val { font-weight: 700; }

/* PSU Selector Pills */
.psu-pills-bar { display: flex; gap: 8px; margin-bottom: 16px; flex-wrap: wrap; background: var(--card); padding: 10px 14px; border-radius: 8px; border: 1px solid var(--border); align-items: center; }
.psu-pill { background: #0b1120; border: 1px solid var(--border); color: #cbd5e1; padding: 6px 14px; border-radius: 6px; font-family: monospace; font-size: 13px; font-weight: 700; cursor: pointer; transition: all 0.15s; }
.psu-pill:hover { border-color: var(--cyan); color: #fff; }
.psu-pill.active { background: #06b6d4; color: #000; border-color: #06b6d4; }

/* Control Cards */
.ctrl-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); gap: 16px; margin-bottom: 20px; }
.ctrl-card { background: var(--card); padding: 18px; border-radius: 8px; border: 1px solid var(--border); }
.ctrl-card h4 { font-size: 13px; color: #fff; margin-bottom: 12px; display: flex; justify-content: space-between; align-items: center; }

/* Oscilloscope Chart */
.chart-container { background: var(--card); padding: 18px; border-radius: 8px; border: 1px solid var(--border); margin-bottom: 20px; position: relative; }
.chart-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; flex-wrap: wrap; gap: 10px; }
.chart-legend { display: flex; gap: 10px; align-items: center; font-size: 11px; font-family: monospace; font-weight: 700; flex-wrap: wrap; }
.legend-chip { display: inline-flex; align-items: center; gap: 6px; background: #0b1120; border: 1px solid var(--border); padding: 4px 8px; border-radius: 4px; cursor: pointer; user-select: none; }
.legend-color { width: 10px; height: 10px; border-radius: 2px; }
.canvas-wrap { position: relative; width: 100%; height: 280px; background: #060a12; border-radius: 6px; border: 1px solid var(--border); overflow: hidden; }
canvas { width: 100%; height: 100%; display: block; }
.chart-crosshair-tooltip { position: absolute; background: rgba(11, 17, 32, 0.95); border: 1px solid var(--blue); border-radius: 6px; padding: 8px 12px; font-size: 11px; font-family: monospace; color: #fff; pointer-events: none; z-index: 20; display: none; line-height: 1.4; box-shadow: 0 4px 20px rgba(0,0,0,0.8); }

/* Status Strips */
.status-strip-box { background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 12px 16px; margin-bottom: 12px; }
.status-strip-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; font-size: 12px; font-family: monospace; font-weight: 700; color: var(--cyan); }
.status-chips-wrap { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; }
.status-chip { font-family: monospace; font-size: 11px; font-weight: 700; padding: 5px 10px; border-radius: 4px; background: #0b1120; border: 1px solid var(--border); color: #64748b; transition: all 0.15s; user-select: none; cursor: help; }
.status-chip:hover { border-color: var(--blue); color: #fff; }
.status-chip.active-red { background: #7f1d1d; color: #fecaca; border-color: #ef4444; box-shadow: 0 0 10px rgba(239, 68, 68, 0.6); animation: pulse 1s infinite; font-weight: 900; }

/* Tooltip Custom */
.custom-tooltip { position: fixed; background: #0b1120; border: 1px solid var(--cyan); color: #fff; padding: 10px 14px; border-radius: 8px; font-size: 12px; pointer-events: none; z-index: 10000; box-shadow: 0 10px 25px rgba(0,0,0,0.85); max-width: 340px; opacity: 0; transform: translateY(6px); transition: opacity 0.15s ease, transform 0.15s ease; display: none; line-height: 1.4; }
.custom-tooltip.show { opacity: 1; transform: translateY(0); display: block; }
.custom-tooltip.alert { border-color: #ef4444; box-shadow: 0 0 20px rgba(239,68,68,0.5); }

/* Tables */
.table { width: 100%; border-collapse: collapse; font-family: monospace; font-size: 12px; }
.table th, .table td { padding: 9px 12px; border-bottom: 1px solid var(--border); text-align: left; }
.table th { background: var(--input); color: var(--muted); position: sticky; top: 0; z-index: 10; font-weight: 700; }

.toast { position: fixed; bottom: 20px; right: 20px; background: var(--card); border: 1px solid var(--blue); color: #fff; padding: 10px 18px; border-radius: 6px; font-size: 12px; font-weight: 600; opacity: 0; transform: translateY(10px); transition: all 0.2s ease; pointer-events: none; z-index: 999; }
.toast.show { opacity: 1; transform: translateY(0); }
</style>
</head>
<body>

<header>
    <div class="brand">
        <div class="logo-icon">PSU</div>
        <div>
            <h1 style="font-size:16px;">PMBus Multi-PSU Lab Pro</h1>
            <p style="font-size:11px;color:var(--muted)">Supervision &amp; Contrôle Temps Réel PMBus 1.2/1.3 pour Banc de Test</p>
        </div>
    </div>
    <div style="display:flex;align-items:center;gap:14px;flex-wrap:wrap;">
        <div id="badgeConnState" class="conn-badge live">
            <span class="conn-dot"></span>
            <span id="txtConnState">LIVE</span>
        </div>
        <div class="nav-tabs">
            <button class="tab-btn active" id="btnNavOverview">⚡ Global Rack Grid</button>
            <button class="tab-btn" id="btnNavDeep">🔍 PSU Deep-Dive</button>
            <button class="tab-btn" id="btnNavScope">📈 Graphique</button>
            <button class="tab-btn" id="btnNavRegs">📜 PMBus Registers</button>
            <button class="tab-btn" id="btnNavLogger">💾 CSV Logger</button>
        </div>
    </div>
</header>

<div class="main-wrap">

    <!-- VIEW 1: GLOBAL MULTI-PSU OVERVIEW -->
    <div class="view-panel active" id="viewOverview">
        <!-- Top Toolbar -->
        <div class="hero-bar">
            <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap;">
                <span style="font-weight:700;font-size:13px;">Bus I2C :</span>
                <select id="selI2CBus"></select>
                <button class="btn" id="btnRescan">🔍 Scanner le Bus</button>
                <span style="color:var(--muted);font-size:12px;" id="lblDetectedCount">5 PSU(s) active(s)</span>
            </div>
            <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;">
                <span style="font-size:11px;color:var(--muted);font-weight:700;">BROADCAST :</span>
                <button class="btn btn-green" id="btnAllOn">⚡ Allumer Toutes</button>
                <button class="btn btn-red" id="btnAllOff">⏻ Éteindre Toutes</button>
                <button class="btn btn-gray" id="btnAllClear">🧹 CLEAR_FAULTS (0x03)</button>
            </div>
        </div>

        <!-- Global Aggregated KPIs -->
        <div class="kpi-grid">
            <div class="kpi-card"><span>Puissance Totale Entrée (PIN)</span><strong style="color:var(--indigo);" id="kpiTotalPin">0.0 W</strong></div>
            <div class="kpi-card"><span>Puissance Totale Sortie (POUT)</span><strong style="color:var(--purple);" id="kpiTotalPout">0.0 W</strong></div>
            <div class="kpi-card"><span>Courant Total Distribué (IOUT)</span><strong style="color:var(--amber);" id="kpiTotalIout">0.00 A</strong></div>
            <div class="kpi-card"><span>Rendement Global Système</span><strong style="color:var(--green);" id="kpiGlobalEff">0.0 %</strong></div>
            <div class="kpi-card"><span>État Global du Banc</span><strong style="color:#fff;" id="kpiGlobalState">ACTIVÉ (ON)</strong></div>
            <div class="kpi-card"><span>Alarmes / Défauts Actifs</span><strong style="color:var(--red);" id="kpiGlobalFaults">0 DÉFAUT</strong></div>
        </div>

        <h3 style="font-size:13px;color:var(--muted);margin-bottom:14px;text-transform:uppercase;letter-spacing:0.5px;">ALIMENTATIONS DÉTECTÉES EN PARALLÈLE :</h3>
        <div class="psu-grid" id="gridPsuCards">
            <!-- Dynamically populated PSU Cards -->
        </div>
    </div>

    <!-- VIEW 2: INDIVIDUAL PSU DEEP-DIVE -->
    <div class="view-panel" id="viewDeep">
        <!-- PSU Selector Pills Bar -->
        <div class="psu-pills-bar">
            <span style="font-size:11px;color:var(--muted);font-weight:700;">SÉLECTION DE LA PSU :</span>
            <div id="boxPsuPills" style="display:flex;gap:6px;flex-wrap:wrap;"></div>
            <div style="margin-left:auto;display:flex;gap:8px;align-items:center;">
                <span class="badge-state" id="deepBadgePower">ON</span>
                <button class="btn btn-red" id="btnDeepClearFaults" style="font-size:11px;padding:5px 10px;">Clear Faults (0x03)</button>
            </div>
        </div>

        <!-- PSU ID Bar -->
        <div style="background:var(--card);padding:12px 18px;border-radius:8px;border:1px solid var(--border);margin-bottom:16px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;font-family:monospace;font-size:12px;">
            <div>
                <span style="color:var(--cyan);font-weight:700;" id="txtPsuModel">CRPS-3000W</span>
                <span style="color:var(--muted);margin-left:10px;" id="txtPsuMfr">MFR: STANDARD_PWR</span>
            </div>
            <div style="color:var(--muted);">
                <span id="txtPsuSerial">SN: 1001</span> | <span id="txtPsuRev">REV-2.4</span> | <span id="txtPsuPmbusRev">PMBus 1.2</span>
            </div>
        </div>

        <!-- Telemetry KPIs for selected PSU -->
        <div class="kpi-grid">
            <div class="kpi-card"><span>Tension Entrée (VIN)</span><strong style="color:var(--cyan);" id="deepVin">0.00 V</strong></div>
            <div class="kpi-card"><span>Tension Sortie (VOUT)</span><strong style="color:var(--blue);" id="deepVout">0.000 V</strong></div>
            <div class="kpi-card"><span>Courant Entrée (IIN)</span><strong style="color:var(--amber);" id="deepIin">0.00 A</strong></div>
            <div class="kpi-card"><span>Courant Sortie (IOUT)</span><strong style="color:var(--amber);" id="deepIout">0.00 A</strong></div>
            <div class="kpi-card"><span>Puissance Entrée (PIN)</span><strong style="color:var(--indigo);" id="deepPin">0.0 W</strong></div>
            <div class="kpi-card"><span>Puissance Sortie (POUT)</span><strong style="color:var(--purple);" id="deepPout">0.0 W</strong></div>
            <div class="kpi-card"><span>Rendement Instantané</span><strong style="color:var(--green);" id="deepEff">0.0 %</strong></div>
            <div class="kpi-card"><span>Températures (T1/T2/T3)</span><strong style="color:var(--red);font-size:18px;" id="deepTemps">-- / -- C</strong></div>
            <div class="kpi-card"><span>Ventilateurs (FAN 1/2)</span><strong style="color:var(--blue);font-size:18px;" id="deepFans">-- RPM</strong></div>
        </div>

        <!-- Interactive Controls -->
        <div class="ctrl-grid">
            <!-- VOUT COMMAND -->
            <div class="ctrl-card">
                <h4>
                    <span>Tension de Consigne (VOUT_COMMAND - 0x21)</span>
                    <strong style="color:var(--blue);font-family:monospace;" id="lblDeepVoutCmd">12.00 V</strong>
                </h4>
                <div style="display:flex;gap:10px;align-items:center;margin-bottom:10px;">
                    <input type="number" id="txtDeepSetVout" step="0.05" min="5.0" max="15.0" value="12.00" style="width:110px;">
                    <button class="btn" id="btnDeepApplyVout">Régler VOUT (0x21)</button>
                </div>
                <div style="display:flex;gap:6px;flex-wrap:wrap;">
                    <button class="btn btn-gray" style="font-size:11px;padding:3px 8px;" onclick="setDeepVoutPreset(11.50)">Preset 11.50V</button>
                    <button class="btn btn-gray" style="font-size:11px;padding:3px 8px;" onclick="setDeepVoutPreset(12.00)">Preset 12.00V</button>
                    <button class="btn btn-gray" style="font-size:11px;padding:3px 8px;" onclick="setDeepVoutPreset(12.50)">Preset 12.50V</button>
                </div>
            </div>

            <!-- OPERATION -->
            <div class="ctrl-card">
                <h4>
                    <span>Étage de Puissance (OPERATION - 0x01)</span>
                    <strong style="color:var(--cyan);font-family:monospace;" id="lblDeepOpCode">0x80 (ON)</strong>
                </h4>
                <div style="display:flex;gap:6px;flex-wrap:wrap;margin-bottom:8px;">
                    <button class="btn btn-green" id="btnDeepOpOn">Power ON (0x80)</button>
                    <button class="btn btn-amber" id="btnDeepOpMarginH">Margin High (0x98)</button>
                    <button class="btn btn-amber" id="btnDeepOpMarginL">Margin Low (0x94)</button>
                    <button class="btn btn-red" id="btnDeepOpOff">Power OFF (0x00)</button>
                </div>
            </div>

            <!-- STARTUP THRESHOLDS -->
            <div class="ctrl-card" style="grid-column: 1 / -1;">
                <h4>
                    <span>Seuils de Démarrage &amp; Soft-Start (VIN_ON, VIN_OFF, TON_RISE)</span>
                    <button class="btn btn-gray" style="font-size:11px;padding:3px 8px;" id="btnDeepSaveStartup">Enregistrer</button>
                </h4>
                <div style="display:grid;grid-template-columns:repeat(auto-fit, minmax(200px, 1fr));gap:12px;">
                    <div style="background:#0b1120;padding:10px 12px;border-radius:6px;border:1px solid var(--border);">
                        <span style="font-size:11px;color:var(--muted);">Démarrage (VIN_ON - 0x35)</span>
                        <div style="display:flex;gap:6px;align-items:center;margin-top:4px;">
                            <input type="number" id="txtDeepVinOn" step="1.0" style="width:90px;" value="180">
                            <span>V</span>
                        </div>
                    </div>
                    <div style="background:#0b1120;padding:10px 12px;border-radius:6px;border:1px solid var(--border);">
                        <span style="font-size:11px;color:var(--muted);">Arrêt Sous-Tension (VIN_OFF - 0x36)</span>
                        <div style="display:flex;gap:6px;align-items:center;margin-top:4px;">
                            <input type="number" id="txtDeepVinOff" step="1.0" style="width:90px;" value="160">
                            <span>V</span>
                        </div>
                    </div>
                    <div style="background:#0b1120;padding:10px 12px;border-radius:6px;border:1px solid var(--border);">
                        <span style="font-size:11px;color:var(--muted);">Rampe Soft-Start (TON_RISE - 0x61)</span>
                        <div style="display:flex;gap:6px;align-items:center;margin-top:4px;">
                            <input type="number" id="txtDeepTonRise" step="1.0" style="width:90px;" value="15">
                            <span>ms</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- PMBUS STATUS BIT MATRIX -->
        <h4 style="font-size:12px;color:var(--muted);margin-bottom:10px;text-transform:uppercase;">MATRICE DES REGISTRES D'ÉTAT PMBUS (0x78 - 0x81) :</h4>
        <div id="boxDeepStatusStrips"></div>
    </div>

    <!-- VIEW 3: REAL-TIME GRAPH -->
    <div class="view-panel" id="viewScope">
        <div class="chart-container">
            <div class="chart-header">
                <div>
                    <strong style="font-size:14px;">Graphique Télémétrique Temps Réel</strong>
                    <span style="font-size:11px;color:var(--muted);margin-left:8px;">(Axe Y Gauche: Volts | Axe Y Droite: Ampères / Watts / °C)</span>
                </div>
                <div class="chart-legend" id="chartPsuLegend" style="margin-top:8px; margin-bottom:8px;">
                    <span style="font-size:11px;color:var(--muted);">PSU(s) affichée(s) :</span>
                    <!-- Rempli dynamiquement -->
                </div>
                <div class="chart-legend">
                    <label class="legend-chip"><input type="checkbox" id="chkVin" checked><div class="legend-color" style="background:var(--cyan);"></div><span style="color:var(--cyan);">VIN</span></label>
                    <label class="legend-chip"><input type="checkbox" id="chkVout" checked><div class="legend-color" style="background:var(--blue);"></div><span style="color:var(--blue);">VOUT</span></label>
                    <label class="legend-chip"><input type="checkbox" id="chkIin" checked><div class="legend-color" style="background:var(--amber);"></div><span style="color:var(--amber);">IIN</span></label>
                    <label class="legend-chip"><input type="checkbox" id="chkIout" checked><div class="legend-color" style="background:#f97316;"></div><span style="color:#f97316;">IOUT</span></label>
                    <label class="legend-chip"><input type="checkbox" id="chkPin"><div class="legend-color" style="background:var(--indigo);"></div><span style="color:var(--indigo);">PIN</span></label>
                    <label class="legend-chip"><input type="checkbox" id="chkPout"><div class="legend-color" style="background:var(--purple);"></div><span style="color:var(--purple);">POUT</span></label>
                    <label class="legend-chip"><input type="checkbox" id="chkT1" checked><div class="legend-color" style="background:var(--red);"></div><span style="color:var(--red);">T1 (°C)</span></label>
                </div>
            </div>
            <div class="canvas-wrap" id="canvasContainer">
                <canvas id="scopeCanvas"></canvas>
                <div id="scopeCrosshair" class="chart-crosshair-tooltip"></div>
            </div>
        </div>
    </div>

    <!-- VIEW 4: FULL PMBUS REGISTERS TABLE -->
    <div class="view-panel" id="viewRegs">
        <div style="background:var(--card);padding:16px;border-radius:8px;border:1px solid var(--border);">
            <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:14px;flex-wrap:wrap;gap:10px;">
                <div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap;">
                    <span style="font-weight:700;font-size:12px;">PSU Cible :</span>
                    <select id="selRegPsuTarget"></select>
                    <span>Catégorie :</span>
                    <select id="selRegCategory">
                        <option value="ALL">Toutes les Catégories</option>
                        <option value="Startup & Input (VIN)">Startup &amp; Input (VIN)</option>
                        <option value="Output Voltage (VOUT)">Output Voltage (VOUT)</option>
                        <option value="Sequencing & Timings">Sequencing &amp; Timings</option>
                        <option value="Current & Power (IOUT)">Current &amp; Power (IOUT)</option>
                        <option value="Thermal & Cooling">Thermal &amp; Cooling</option>
                        <option value="Regulation & Timing">Regulation &amp; Timing</option>
                        <option value="Status & Diagnostics">Status &amp; Diagnostics</option>
                        <option value="Manufacturer Info">Manufacturer Info</option>
                        <option value="Configuration">Configuration</option>
                    </select>
                    <input type="text" id="txtRegSearch" placeholder="Rechercher (ex: READ_VIN, 0x88, VOUT)..." style="width:260px;">
                </div>
                <button class="btn btn-gray" id="btnRefreshRegs">Actualiser les Registres</button>
            </div>
            <div style="max-height:600px;overflow-y:auto;border:1px solid var(--border);border-radius:6px;">
                <table class="table">
                    <thead>
                        <tr>
                            <th style="width:240px;">NOM DU REGISTRE</th>
                            <th style="width:80px;">HEX</th>
                            <th style="width:200px;">CATÉGORIE</th>
                            <th>DESCRIPTION STANDARD PMBUS</th>
                            <th style="width:200px;">VALEUR DÉCODÉE</th>
                        </tr>
                    </thead>
                    <tbody id="tblRegistersBody">
                        <tr><td colspan="5" style="text-align:center;padding:20px;">Chargement des registres PMBus...</td></tr>
                    </tbody>
                </table>
            </div>
        </div>
    </div>

    <!-- VIEW 5: DATA LOGGER -->
    <div class="view-panel" id="viewLogger">
        <div class="hero-bar">
            <div>
                <strong style="font-size:14px;">Enregistreur de Télémétrie CSV Multi-PSU</strong>
                <p style="color:var(--muted);font-size:12px;margin-top:4px;" id="lblLoggerStatus">0 enregistrement(s) capturé(s)</p>
            </div>
            <div style="display:flex;gap:10px;">
                <button class="btn btn-green" id="btnToggleLogger">Démarrer l'Enregistrement</button>
                <button class="btn btn-gray" id="btnDownloadCsv">Télécharger le CSV</button>
            </div>
        </div>
    </div>

</div>

<!-- Tooltip box -->
<div id="tooltipBox" class="custom-tooltip">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;gap:8px;">
        <strong id="ttTitle" style="color:var(--cyan);font-family:monospace;font-size:12px;">FLAG</strong>
        <span id="ttBadge" style="font-size:9px;font-weight:700;padding:2px 6px;border-radius:4px;background:#1e293b;color:#94a3b8;">NORMAL (0)</span>
    </div>
    <div style="font-size:11px;color:#cbd5e1;margin-bottom:4px;" id="ttReg">STATUS REGISTER</div>
    <div style="font-size:11px;color:#94a3b8;" id="ttDesc">Description technique</div>
</div>

<div id="toastBox" class="toast"></div>

<script>
document.addEventListener("DOMContentLoaded", () => {
    let currentOverview = null;
    let selectedPsuAddr = "0x58";
    let allRegisters = [];
    let isLogging = false;

    // Scope History Buffer
    const MAX_POINTS = 60;
    const psuHistory = {};

    // SSE Stream
    let sseSource = null;
    let lastDataTime = Date.now();

    function showToast(msg) {
        const box = document.getElementById("toastBox");
        box.textContent = msg;
        box.classList.add("show");
        setTimeout(() => box.classList.remove("show"), 2500);
    }

    // Navigation Tabs
    const views = {
        btnNavOverview: "viewOverview",
        btnNavDeep: "viewDeep",
        btnNavScope: "viewScope",
        btnNavRegs: "viewRegs",
        btnNavLogger: "viewLogger"
    };

    Object.keys(views).forEach(btnId => {
        document.getElementById(btnId).addEventListener("click", () => {
            Object.keys(views).forEach(b => {
                document.getElementById(b).classList.remove("active");
                document.getElementById(views[b]).classList.remove("active");
            });
            document.getElementById(btnId).classList.add("active");
            document.getElementById(views[btnId]).classList.add("active");
            if (btnId === "btnNavRegs") loadRegisters();
            if (btnId === "btnNavScope") renderScope();
        });
    });

    // Initialisation des Bus I2C
    fetch("/api/buses")
        .then(r => r.json())
        .then(data => {
            const sel = document.getElementById("selI2CBus");
            sel.innerHTML = "";
            
            const optAll = document.createElement("option");
            optAll.value = "all";
            optAll.textContent = "Tous les bus (Multi-Bus Global)";
            sel.appendChild(optAll);

            data.buses.forEach(b => {
                const opt = document.createElement("option");
                opt.value = b;
                opt.textContent = `Bus #${b} (/dev/i2c-${b})`;
                if (b === data.current_bus) opt.selected = true;
                sel.appendChild(opt);
            });
        }).catch(() => {});

    document.getElementById("btnRescan").addEventListener("click", () => {
        const b = document.getElementById("selI2CBus").value;
        showToast(`Scan du bus I2C #${b}...`);
        fetch(`/api/scan?bus=${b}`)
            .then(r => r.json())
            .then(data => {
                updateOverviewUI(data);
                showToast(`Scan terminé : ${data.psu_count} PSU(s) trouvée(s)`);
            });
    });

    // Broadcast Actions
    document.getElementById("btnAllOn").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "power_all", state: true}) })
            .then(() => showToast("Commande d'allumage envoyée à toutes les PSU"));
    });

    document.getElementById("btnAllOff").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "power_all", state: false}) })
            .then(() => showToast("Commande d'extinction envoyée à toutes les PSU"));
    });

    document.getElementById("btnAllClear").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "clear_all_faults"}) })
            .then(() => showToast("CLEAR_FAULTS envoyé à toutes les PSU"));
    });

    // SSE Connection Watchdog
    function initSSE() {
        if (sseSource) sseSource.close();
        const badge = document.getElementById("badgeConnState");
        const txt = document.getElementById("txtConnState");

        if (sseSource) {
            sseSource.close();
        }
        sseSource = new EventSource("/api/stream");
        sseSource.onopen = () => {
            badge.className = "conn-badge live";
            txt.textContent = "LIVE";
        };
        sseSource.onmessage = (e) => {
            lastDataTime = Date.now();
            try {
                const data = JSON.parse(e.data);
                updateOverviewUI(data);
            } catch (err) {}
        };
        sseSource.onerror = () => {
            badge.className = "conn-badge reconnecting";
            txt.textContent = "RECONNECT";
        };
    }

    setInterval(() => {
        if (Date.now() - lastDataTime > 10000) {
            document.getElementById("badgeConnState").className = "conn-badge reconnecting";
            document.getElementById("txtConnState").textContent = "RECONNECT";
            lastDataTime = Date.now(); // Évite un reconnect en boucle immédiat
            initSSE();
        }
    }, 5000);

    initSSE();

    // Mise à jour de l'UI globale
    function updateOverviewUI(data) {
        if (!data || !data.psus || data.psus.length === 0) return;
        currentOverview = data;
        
        if (!data.psus.find(p => p.addr_hex === selectedPsuAddr)) {
            selectedPsuAddr = data.psus[0].addr_hex;
        }

        document.getElementById("kpiTotalPin").textContent = `${data.total_pin} W`;
        document.getElementById("kpiTotalPout").textContent = `${data.total_pout} W`;
        document.getElementById("kpiTotalIout").textContent = `${data.total_iout} A`;
        document.getElementById("kpiGlobalEff").textContent = `${data.global_efficiency} %`;
        document.getElementById("kpiGlobalState").textContent = data.global_power_state ? "ACTIVÉ (ON)" : "COUPE (OFF)";
        document.getElementById("kpiGlobalState").style.color = data.global_power_state ? "var(--green)" : "var(--red)";
        document.getElementById("kpiGlobalFaults").textContent = `${data.total_active_faults} DÉFAUT(S)`;
        document.getElementById("kpiGlobalFaults").style.color = data.total_active_faults > 0 ? "var(--red)" : "var(--green)";
        document.getElementById("lblDetectedCount").textContent = `${data.psu_count} PSU(s) active(s) ${data.is_mock ? '(SIMULATION)' : ''}`;

        // Mise à jour du Logger Status
        if (data.logger) {
            isLogging = data.logger.is_logging;
            document.getElementById("lblLoggerStatus").textContent = `${data.logger.records_count} enregistrement(s) capturé(s)`;
            const btnLog = document.getElementById("btnToggleLogger");
            btnLog.textContent = isLogging ? "Arrêter l'Enregistrement" : "Démarrer l'Enregistrement";
            btnLog.className = isLogging ? "btn btn-red" : "btn btn-green";
        }

        // Grille des PSU cards
        const grid = document.getElementById("gridPsuCards");
        grid.innerHTML = "";

        const pills = document.getElementById("boxPsuPills");
        pills.innerHTML = "";

        const selPsuTarget = document.getElementById("selRegPsuTarget");
        const prevSelectedTarget = selPsuTarget.value;
        selPsuTarget.innerHTML = "";

        let rebuildLegend = false;
        data.psus.forEach(psu => {
            if (!psuHistory[psu.addr_hex]) {
                psuHistory[psu.addr_hex] = { vin: [], vout: [], iin: [], iout: [], pin: [], pout: [], t1: [], checked: true };
                rebuildLegend = true;
            }
            const h = psuHistory[psu.addr_hex];
            h.vin.push(psu.vin); if (h.vin.length > MAX_POINTS) h.vin.shift();
            h.vout.push(psu.vout); if (h.vout.length > MAX_POINTS) h.vout.shift();
            h.iin.push(psu.iin); if (h.iin.length > MAX_POINTS) h.iin.shift();
            h.iout.push(psu.iout); if (h.iout.length > MAX_POINTS) h.iout.shift();
            h.pin.push(psu.pin); if (h.pin.length > MAX_POINTS) h.pin.shift();
            h.pout.push(psu.pout); if (h.pout.length > MAX_POINTS) h.pout.shift();
            h.t1.push(psu.temp1); if (h.t1.length > MAX_POINTS) h.t1.shift();

            // PSU Card
            const card = document.createElement("div");
            card.className = "psu-card";
            card.onclick = () => {
                selectedPsuAddr = psu.addr_hex;
                document.getElementById("btnNavDeep").click();
            };

            const stateBadge = psu.power_state ? '<span class="badge-state">ON</span>' : '<span class="badge-state off">OFF</span>';
            const faultBadge = psu.active_faults > 0 ? `<span class="badge-fault">${psu.active_faults} ALARME(S)</span>` : '';

            card.innerHTML = `
                <div class="psu-card-header">
                    <div>
                        <strong style="font-size:14px;color:var(--cyan);">PSU #${psu.slot_num}</strong>
                        <span style="font-family:monospace;font-size:11px;color:var(--muted);margin-left:6px;">[${psu.addr_hex}]</span>
                    </div>
                    <div style="display:flex;gap:6px;align-items:center;">
                        ${stateBadge}
                        ${faultBadge}
                    </div>
                </div>
                <div class="psu-metric-row"><span class="lbl">Tension Sortie (VOUT):</span><span class="val" style="color:var(--blue);">${psu.vout} V</span></div>
                <div class="psu-metric-row"><span class="lbl">Courant Charge (IOUT):</span><span class="val" style="color:var(--amber);">${psu.iout} A</span></div>
                <div class="psu-metric-row"><span class="lbl">Puissance Sortie (POUT):</span><span class="val" style="color:var(--purple);">${psu.pout} W</span></div>
                <div class="psu-metric-row"><span class="lbl">Tension Entrée (VIN):</span><span class="val">${psu.vin} V</span></div>
                <div class="psu-metric-row"><span class="lbl">Température Primaire:</span><span class="val" style="color:var(--red);">${psu.temp1} °C</span></div>
                <div class="psu-metric-row"><span class="lbl">Rendement Instantané:</span><span class="val" style="color:var(--green);">${psu.efficiency} %</span></div>
                <div style="margin-top:12px;display:flex;justify-content:flex-end;">
                    <span style="font-size:11px;color:var(--cyan);font-weight:700;">Inspecter PSU &rarr;</span>
                </div>
            `;
            grid.appendChild(card);

            // Pill
            const pill = document.createElement("button");
            pill.className = `psu-pill ${selectedPsuAddr === psu.addr_hex ? 'active' : ''}`;
            pill.textContent = `PSU #${psu.slot_num} [${psu.addr_hex}]`;
            pill.onclick = () => {
                selectedPsuAddr = psu.addr_hex;
                updateOverviewUI(currentOverview);
            };
            pills.appendChild(pill);

            // Select option
            const opt = document.createElement("option");
            opt.value = psu.addr_hex;
            opt.textContent = `PSU #${psu.slot_num} (${psu.addr_hex}) - ${psu.mfr.model}`;
            if (psu.addr_hex === prevSelectedTarget || (!prevSelectedTarget && psu.addr_hex === selectedPsuAddr)) opt.selected = true;
            selPsuTarget.appendChild(opt);
        });

        if (rebuildLegend) {
            const psuLegend = document.getElementById("chartPsuLegend");
            psuLegend.innerHTML = '<span style="font-size:11px;color:var(--muted);">PSU(s) affichée(s) :</span>';
            Object.keys(psuHistory).forEach(addr => {
                const lbl = document.createElement("label");
                lbl.className = "legend-chip";
                const chk = document.createElement("input");
                chk.type = "checkbox";
                chk.checked = psuHistory[addr].checked;
                chk.onchange = (e) => { psuHistory[addr].checked = e.target.checked; renderScope(); };
                
                const colorBox = document.createElement("div");
                colorBox.className = "legend-color";
                colorBox.style.background = "#fff";
                
                const spanTxt = document.createElement("span");
                spanTxt.textContent = addr;
                
                lbl.appendChild(chk);
                lbl.appendChild(colorBox);
                lbl.appendChild(spanTxt);
                psuLegend.appendChild(lbl);
            });
        }

        // Mise à jour de la vue Deep-Dive
        const curPsu = data.psus.find(p => p.addr_hex === selectedPsuAddr) || data.psus[0];
        if (curPsu) {
            document.getElementById("deepBadgePower").textContent = curPsu.power_state ? "STATUS: ON" : "STATUS: OFF";
            document.getElementById("deepBadgePower").className = curPsu.power_state ? "badge-state" : "badge-state off";

            document.getElementById("txtPsuModel").textContent = curPsu.mfr.model || "CRPS-3000W";
            document.getElementById("txtPsuMfr").textContent = `MFR: ${curPsu.mfr.id || "STANDARD_PWR"}`;
            document.getElementById("txtPsuSerial").textContent = `SN: ${curPsu.mfr.serial || "1001"}`;
            document.getElementById("txtPsuRev").textContent = curPsu.mfr.revision || "REV-1.0";
            document.getElementById("txtPsuPmbusRev").textContent = `PMBus: ${curPsu.mfr.pmbus_rev || "0x22"}`;

            document.getElementById("deepVin").textContent = `${curPsu.vin} V`;
            document.getElementById("deepVout").textContent = `${curPsu.vout} V`;
            document.getElementById("deepIin").textContent = `${curPsu.iin} A`;
            document.getElementById("deepIout").textContent = `${curPsu.iout} A`;
            document.getElementById("deepPin").textContent = `${curPsu.pin} W`;
            document.getElementById("deepPout").textContent = `${curPsu.pout} W`;
            document.getElementById("deepEff").textContent = `${curPsu.efficiency} %`;
            document.getElementById("deepTemps").textContent = `${curPsu.temp1} / ${curPsu.temp2} / ${curPsu.temp3} °C`;
            document.getElementById("deepFans").textContent = `${curPsu.fan1_rpm} / ${curPsu.fan2_rpm} RPM`;

            document.getElementById("lblDeepVoutCmd").textContent = `${curPsu.startup.vout_cmd || 12.00} V`;
            document.getElementById("lblDeepOpCode").textContent = curPsu.op_code === 0x80 ? "0x80 (ON)" : (curPsu.op_code === 0x00 ? "0x00 (OFF)" : `0x${curPsu.op_code.toString(16).toUpperCase()}`);

            // Remplissage des status strips
            renderStatusStrips(curPsu.status_strips);

            if (document.getElementById("viewScope").classList.contains("active")) {
                renderScope();
            }
        }
    }

    // Affichage des Status Strips PMBus
    function renderStatusStrips(strips) {
        const box = document.getElementById("boxDeepStatusStrips");
        box.innerHTML = "";
        if (!strips) return;

        strips.forEach(s => {
            const stripEl = document.createElement("div");
            stripEl.className = "status-strip-box";

            let chipsHtml = "";
            s.chips.forEach(c => {
                const cls = c.active ? "status-chip active-red" : "status-chip";
                chipsHtml += `<div class="${cls}" data-label="${c.label}" data-reg="${c.reg}" data-desc="${c.desc}" data-active="${c.active}">${c.label}</div>`;
            });

            stripEl.innerHTML = `
                <div class="status-strip-header">
                    <span>${s.title}</span>
                    <strong style="color:#fff;">${s.val_hex}</strong>
                </div>
                <div class="status-chips-wrap">${chipsHtml}</div>
            `;
            box.appendChild(stripEl);
        });

        // Hover tooltip events
        box.querySelectorAll(".status-chip").forEach(chip => {
            chip.addEventListener("mouseenter", (e) => {
                const tt = document.getElementById("tooltipBox");
                document.getElementById("ttTitle").textContent = chip.dataset.label;
                document.getElementById("ttReg").textContent = chip.dataset.reg;
                document.getElementById("ttDesc").textContent = chip.dataset.desc;

                const isActive = chip.dataset.active === "true";
                document.getElementById("ttBadge").textContent = isActive ? "ALARME ACTIVE (1)" : "NORMAL (0)";
                document.getElementById("ttBadge").style.background = isActive ? "#7f1d1d" : "#1e293b";
                document.getElementById("ttBadge").style.color = isActive ? "#fca5a5" : "#94a3b8";

                if (isActive) tt.classList.add("alert"); else tt.classList.remove("alert");

                tt.style.left = `${Math.min(window.innerWidth - 360, e.clientX + 15)}px`;
                tt.style.top = `${e.clientY + 15}px`;
                tt.classList.add("show");
            });
            chip.addEventListener("mouseleave", () => {
                document.getElementById("tooltipBox").classList.remove("show");
            });
        });
    }

    // Deep-Dive Controls
    window.setDeepVoutPreset = (val) => {
        document.getElementById("txtDeepSetVout").value = val;
        document.getElementById("btnDeepApplyVout").click();
    };

    document.getElementById("btnDeepApplyVout").addEventListener("click", () => {
        const v = parseFloat(document.getElementById("txtDeepSetVout").value);
        fetch("/api/action", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({action: "set_vout", addr: selectedPsuAddr, vout: v})
        }).then(() => showToast(`Consigne VOUT réglée à ${v}V sur ${selectedPsuAddr}`));
    });

    document.getElementById("btnDeepOpOn").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "set_operation", addr: selectedPsuAddr, op: 0x80}) })
            .then(() => showToast(`Power ON envoyé à ${selectedPsuAddr}`));
    });

    document.getElementById("btnDeepOpOff").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "set_operation", addr: selectedPsuAddr, op: 0x00}) })
            .then(() => showToast(`Power OFF envoyé à ${selectedPsuAddr}`));
    });

    document.getElementById("btnDeepOpMarginH").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "set_operation", addr: selectedPsuAddr, op: 0x98}) })
            .then(() => showToast(`Margin High envoyé à ${selectedPsuAddr}`));
    });

    document.getElementById("btnDeepOpMarginL").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "set_operation", addr: selectedPsuAddr, op: 0x94}) })
            .then(() => showToast(`Margin Low envoyé à ${selectedPsuAddr}`));
    });

    document.getElementById("btnDeepClearFaults").addEventListener("click", () => {
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "clear_faults", addr: selectedPsuAddr}) })
            .then(() => showToast(`CLEAR_FAULTS envoyé à ${selectedPsuAddr}`));
    });

    document.getElementById("btnDeepSaveStartup").addEventListener("click", () => {
        const von = parseFloat(document.getElementById("txtDeepVinOn").value);
        const voff = parseFloat(document.getElementById("txtDeepVinOff").value);
        const ton = parseFloat(document.getElementById("txtDeepTonRise").value);
        fetch("/api/action", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({action: "set_startup", addr: selectedPsuAddr, vin_on: von, vin_off: voff, ton_rise: ton})
        }).then(() => showToast(`Seuils de démarrage mis à jour sur ${selectedPsuAddr}`));
    });

    // Logger
    document.getElementById("btnToggleLogger").addEventListener("click", () => {
        const act = isLogging ? "logger_stop" : "logger_start";
        fetch("/api/action", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: act}) })
            .then(() => showToast(isLogging ? "Enregistrement arrêté" : "Enregistrement démarré"));
    });

    document.getElementById("btnDownloadCsv").addEventListener("click", () => {
        window.location.href = "/api/logger/export";
    });

    // PMBus Registers Table
    function loadRegisters() {
        const target = document.getElementById("selRegPsuTarget").value || selectedPsuAddr;
        fetch(`/api/registers?addr=${target}`)
            .then(r => r.json())
            .then(data => {
                allRegisters = data.registers || [];
                filterRegisters();
            });
    }

    document.getElementById("btnRefreshRegs").addEventListener("click", loadRegisters);
    document.getElementById("selRegPsuTarget").addEventListener("change", loadRegisters);
    document.getElementById("selRegCategory").addEventListener("change", filterRegisters);
    document.getElementById("txtRegSearch").addEventListener("input", filterRegisters);

    function filterRegisters() {
        const cat = document.getElementById("selRegCategory").value;
        const q = document.getElementById("txtRegSearch").value.toLowerCase().trim();
        const tbody = document.getElementById("tblRegistersBody");
        tbody.innerHTML = "";

        const filtered = allRegisters.filter(r => {
            const matchCat = (cat === "ALL" || r.cat === cat);
            const matchQ = (!q || r.name.toLowerCase().includes(q) || r.hex.toLowerCase().includes(q) || r.desc.toLowerCase().includes(q));
            return matchCat && matchQ;
        });

        if (filtered.length === 0) {
            tbody.innerHTML = `<tr><td colspan="5" style="text-align:center;padding:16px;color:var(--muted);">Aucun registre correspondant trouvé.</td></tr>`;
            return;
        }

        filtered.forEach(r => {
            const tr = document.createElement("tr");
            tr.innerHTML = `
                <td style="font-weight:700;color:var(--cyan);">${r.name}</td>
                <td>${r.hex}</td>
                <td style="color:var(--muted);">${r.cat}</td>
                <td>${r.desc}</td>
                <td style="font-weight:700;color:#fff;">${r.val}</td>
            `;
            tbody.appendChild(tr);
        });
    }

    // Oscilloscope Canvas Rendering
    function renderScope() {
        const canvas = document.getElementById("scopeCanvas");
        const ctx = canvas.getContext("2d");
        const rect = canvas.parentElement.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;

        const w = canvas.width;
        const h = canvas.height;

        // Background Grid
        ctx.fillStyle = "#060a12";
        ctx.fillRect(0, 0, w, h);

        ctx.strokeStyle = "#131d2e";
        ctx.lineWidth = 1;
        for (let x = 0; x < w; x += 40) {
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
        }
        for (let y = 0; y < h; y += 35) {
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
        }

        function drawTrace(arr, maxVal, color, isDashed) {
            if (!arr || arr.length < 2) return;
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            if (isDashed) {
                ctx.setLineDash([5, 5]);
            } else {
                ctx.setLineDash([]);
            }
            ctx.beginPath();
            const step = w / (MAX_POINTS - 1);
            arr.forEach((v, idx) => {
                const norm = Math.max(0, Math.min(1, v / maxVal));
                const y = h - (norm * (h - 20) + 10);
                const x = idx * step;
                if (idx === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            });
            ctx.stroke();
            ctx.setLineDash([]); // Reset
        }

        let psuIndex = 0;
        Object.keys(psuHistory).forEach(addr => {
            const hData = psuHistory[addr];
            if (!hData.checked) return; // PSU non sélectionnée pour l'affichage
            
            // Alterner trait plein et pointillé pour différencier les PSUs si elles se superposent
            const isDashed = (psuIndex % 2 !== 0);

            if (document.getElementById("chkVin").checked) drawTrace(hData.vin, 260, "#06b6d4", isDashed);
            if (document.getElementById("chkVout").checked) drawTrace(hData.vout, 15, "#38bdf8", isDashed);
            if (document.getElementById("chkIin").checked) drawTrace(hData.iin, 20, "#f59e0b", isDashed);
            if (document.getElementById("chkIout").checked) drawTrace(hData.iout, 150, "#f97316", isDashed);
            if (document.getElementById("chkPin").checked) drawTrace(hData.pin, 2500, "#818cf8", isDashed);
            if (document.getElementById("chkPout").checked) drawTrace(hData.pout, 2500, "#c084fc", isDashed);
            if (document.getElementById("chkT1").checked) drawTrace(hData.t1, 100, "#ef4444", isDashed);
            
            psuIndex++;
        });
    }

    window.addEventListener("resize", () => {
        if (document.getElementById("viewScope").classList.contains("active")) renderScope();
    });

    ["chkVin", "chkVout", "chkIin", "chkIout", "chkPin", "chkPout", "chkT1"].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.addEventListener("change", renderScope);
    });
});
</script>
</body>
</html>
"""

# ==============================================================================
# 8. LANCEUR PRINCIPAL
# ==============================================================================

def generate_ssl_cert(cert_path="server.pem"):
    if not os.path.exists(cert_path):
        print(f"[SSL] Génération d'un certificat HTTPS auto-signé ({cert_path})...")
        try:
            subprocess.run([
                "openssl", "req", "-x509", "-newkey", "rsa:2048",
                "-keyout", cert_path, "-out", cert_path,
                "-days", "3650", "-nodes", "-subj", "/CN=PMBus-GUI"
            ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[SSL] Certificat HTTPS généré avec succès.")
        except Exception as e:
            print(f"[SSL] ERREUR (openssl non installé ?) : {e}")

def main():
    parser = argparse.ArgumentParser(description="PMBus Multi-PSU Web GUI Server")
    parser.add_argument("--host", default="0.0.0.0", help="Adresse IP d'écoute (défaut: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Port Web HTTP (défaut: 8080)")
    parser.add_argument("--bus", type=int, default=2, help="Numéro de bus I2C (défaut: 2 pour BeagleBone Black)")
    parser.add_argument("--mock", action="store_true", help="Forcer le mode simulation (Mock)")
    args = parser.parse_args()

    print("=" * 75)
    print("  [PMBUS] Multi-PSU Monitor & Web GUI Server (BeagleBone Black)")
    print("=" * 75)
    print(f"  Bus I2C selectionne  : #{args.bus}")
    print(f"  Port d'ecoute        : {args.port}")
    print(f"  Mode Force Simul     : {'OUI' if args.mock else 'NON (Auto-Detection)'}")

    # Initialisation du gestionnaire Multi-PSU
    manager = MultiPSUManager(bus_id=args.bus, force_mock=args.mock)
    PMBusMultiApiHandler.manager = manager

    server = ThreadedHTTPServer((args.host, args.port), PMBusMultiApiHandler)

    # Configuration HTTPS (SSL)
    generate_ssl_cert("server.pem")
    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile="server.pem")
        server.socket = context.wrap_socket(server.socket, server_side=True)
    except Exception as e:
        print(f"[SSL] Attention: Impossible d'activer HTTPS : {e}")

    # Récupération de l'adresse IP locale pour affichage pratique
    local_ip = "127.0.0.1"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        pass

    print("-" * 75)
    print(f"  Dashboard Reseau IP  : https://{local_ip}:{args.port}")
    print("  (L'accès HTTP standard est désactivé au profit du chiffrement HTTPS)")
    print("=" * 75)
    print("  Appuyez sur Ctrl+C pour arreter le serveur.")
    print("=" * 75)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nArret du serveur Web PMBus Multi-PSU.")
        server.server_close()

if __name__ == "__main__":
    main()
