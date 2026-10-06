#!/usr/bin/env python3
"""
================================================================================
 Power Lab Studio Pro (Fault-Tolerant I2C Pacing, Retry & Sample-Hold Cache)
================================================================================
 - 100% ASCII Compatible (Zero Encoding Issues / Zero '?' Characters)
 - Fault-Tolerant I2C Communication Engine:
   * 2x Automatic Hardware Retry on I2C transactions
   * Inter-Register Pacing Delay (3ms) to prevent slave microcontroller saturation
   * Sample & Hold Cache: Prevents gauge dropouts/flicker during transient noise
 - Auto-Healing SSE Connection with Watchdog & Visibility API:
   * Instantly re-establishes live telemetry after PC sleep / wake / tab suspend
   * Visual Connection State Badge: LIVE (Green), RECONNECTING (Yellow), OFFLINE (Red)
 - Robust CLEAR_FAULTS (0x03) execution with bus settling delay
 - Real-Time Telemetry Oscilloscope (Dual Y-Axes, Divisions, Crosshair Tooltip)
 - 9-Trace Real-Time Waveform with Selectable Channels in Legend
 - Interactive PMBus Controls (VOUT_COMMAND, OPERATION, Startup Thresholds)
 - Exact Bitwise PMBus Status Strips (0x78 - 0x7E) with Instant Hover Tooltips
 - Full 80+ Standard PMBus Register Table
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
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

# 1. Strictly SAFE PMBus 1.2/1.3 Register Table (Read-Only Scanner)
PMBUS_SAFE_REGISTERS = [
    # --- CONFIGURATION & STARTUP (READ-ONLY) ---
    {"code": 0x00, "name": "PAGE", "type": "byte_page", "cat": "Configuration", "desc": "Active channel/phase selection (0 to 31, 0xFF=All)"},
    {"code": 0x01, "name": "OPERATION", "type": "byte_op", "cat": "Configuration", "desc": "Active power stage state (0x80=On, 0x00=Off)"},
    {"code": 0x02, "name": "ON_OFF_CONFIG", "type": "byte_onoff", "cat": "Configuration", "desc": "Hardware CONTROL pin and software enable logic"},
    {"code": 0x10, "name": "WRITE_PROTECT", "type": "byte_wp", "cat": "Configuration", "desc": "Register write protection status"},
    {"code": 0x19, "name": "CAPABILITY", "type": "byte_cap", "cat": "Configuration", "desc": "Supported features (PEC, Max speed, SMBALERT#)"},

    # --- INPUT VOLTAGE & STARTUP THRESHOLDS (VIN_ON / VIN_OFF) ---
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

    # --- OUTPUT VOLTAGE & LIMITS (VOUT) ---
    {"code": 0x20, "name": "VOUT_MODE", "type": "byte_vmode", "cat": "Output Voltage (VOUT)", "desc": "VOUT data format (Linear/VID/Direct) and exponent"},
    {"code": 0x21, "name": "VOUT_COMMAND", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Target output voltage command setpoint (V)"},
    {"code": 0x22, "name": "VOUT_TRIM", "type": "word_vout", "cat": "Output Voltage (VOUT)", "desc": "Fine trimming offset for output voltage (V)"},
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

    # --- TIMINGS & POWER SEQUENCING (TON / TOFF) ---
    {"code": 0x60, "name": "TON_DELAY", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-on delay time from enable to ramp (ms)"},
    {"code": 0x61, "name": "TON_RISE", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-on rise / soft-start ramp time (ms)"},
    {"code": 0x62, "name": "TON_MAX_FAULT_LIMIT", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Maximum allowed turn-on transition time (ms)"},
    {"code": 0x63, "name": "TON_MAX_FAULT_RESPONSE", "type": "byte_resp", "cat": "Sequencing & Timings", "desc": "Response profile for TON Max Fault"},
    {"code": 0x64, "name": "TOFF_DELAY", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-off delay time (ms)"},
    {"code": 0x65, "name": "TOFF_FALL", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-off ramp-down fall time (ms)"},
    {"code": 0x66, "name": "TOFF_MAX_WARN_LIMIT", "type": "word_ms", "cat": "Sequencing & Timings", "desc": "Turn-off maximum warning limit (ms)"},

    # --- OUTPUT CURRENT & POWER (IOUT / POUT) ---
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

    # --- TEMPERATURE & COOLING (TEMP) ---
    {"code": 0x4F, "name": "OT_FAULT_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Overtemperature Fault shutdown threshold (deg C)"},
    {"code": 0x50, "name": "OT_FAULT_RESPONSE", "type": "byte_resp", "cat": "Thermal & Cooling", "desc": "Response profile for Overtemperature Fault"},
    {"code": 0x51, "name": "OT_WARN_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Overtemperature Warning advisory threshold (deg C)"},
    {"code": 0x52, "name": "UT_WARN_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Undertemperature Warning advisory threshold (deg C)"},
    {"code": 0x53, "name": "UT_FAULT_LIMIT", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Undertemperature Fault shutdown threshold (deg C)"},
    {"code": 0x54, "name": "UT_FAULT_RESPONSE", "type": "byte_resp", "cat": "Thermal & Cooling", "desc": "Response profile for Undertemperature Fault"},
    {"code": 0x8D, "name": "READ_TEMPERATURE_1", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Primary temperature sensor reading (deg C)"},
    {"code": 0x8E, "name": "READ_TEMPERATURE_2", "type": "word_temp", "cat": "Thermal & Cooling", "desc": "Secondary temperature sensor reading (deg C)"},

    # --- REGULATION & CLOCK ---
    {"code": 0x32, "name": "MAX_DUTY", "type": "word_percent", "cat": "Regulation & Timing", "desc": "Maximum permitted PWM duty cycle (%)"},
    {"code": 0x33, "name": "FREQUENCY_SWITCH", "type": "word_khz", "cat": "Regulation & Timing", "desc": "Target switching frequency (kHz)"},
    {"code": 0x94, "name": "READ_DUTY_CYCLE", "type": "word_percent", "cat": "Regulation & Timing", "desc": "Measured PWM duty cycle (%)"},
    {"code": 0x95, "name": "READ_FREQUENCY", "type": "word_khz", "cat": "Regulation & Timing", "desc": "Measured switching frequency (kHz)"},

    # --- STATUS & DIAGNOSTICS ---
    {"code": 0x78, "name": "STATUS_BYTE", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Summary status byte"},
    {"code": 0x79, "name": "STATUS_WORD", "type": "word_hex", "cat": "Status & Diagnostics", "desc": "Full 16-bit status word"},
    {"code": 0x7A, "name": "STATUS_VOUT", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Output voltage faults"},
    {"code": 0x7B, "name": "STATUS_IOUT", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Output current/power faults"},
    {"code": 0x7C, "name": "STATUS_INPUT", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Input voltage/current faults"},
    {"code": 0x7D, "name": "STATUS_TEMPERATURE", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Thermal faults"},
    {"code": 0x7E, "name": "STATUS_CML", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "I2C, PEC, NVM & logic errors"},
    {"code": 0x80, "name": "STATUS_MFR_SPECIFIC", "type": "byte_hex", "cat": "Status & Diagnostics", "desc": "Manufacturer proprietary flags"},

    # --- MANUFACTURER INFO & RATINGS ---
    {"code": 0x98, "name": "PMBUS_REVISION", "type": "byte_hex", "cat": "Manufacturer & Ratings", "desc": "Supported PMBus specification revision code"},
    {"code": 0x99, "name": "MFR_ID", "type": "block_ascii", "cat": "Manufacturer & Ratings", "desc": "Manufacturer identity string"},
    {"code": 0x9A, "name": "MFR_MODEL", "type": "block_ascii", "cat": "Manufacturer & Ratings", "desc": "Equipment model number string"},
    {"code": 0x9B, "name": "MFR_REVISION", "type": "block_ascii", "cat": "Manufacturer & Ratings", "desc": "Hardware revision string"},
    {"code": 0x9C, "name": "MFR_LOCATION", "type": "block_ascii", "cat": "Manufacturer & Ratings", "desc": "Manufacturing facility location"},
    {"code": 0x9D, "name": "MFR_DATE", "type": "block_ascii", "cat": "Manufacturer & Ratings", "desc": "Manufacturing date code (YYMMDD)"},
    {"code": 0x9E, "name": "MFR_SERIAL", "type": "block_ascii", "cat": "Manufacturer & Ratings", "desc": "Device serial number"}
]

PMBUS_STATUS_CHIPS = [
    {
        "reg": "STATUS_BYTE", "code": 0x78, "title": "STATUS_BYTE (0x78)",
        "chips": [
            ("Busy", 7, "Bit 7 (0x80) - Device is busy processing an internal command or task and cannot respond", False),
            ("Off", 6, "Bit 6 (0x40) - Output conversion stage is DISABLED (Unit is turned OFF)", False),
            ("Ovout", 5, "Bit 5 (0x20) - Output Overvoltage Fault (Measured VOUT exceeded safety limit VOUT_OV_FAULT_LIMIT)", True),
            ("Oiout", 4, "Bit 4 (0x10) - Output Overcurrent Fault (Measured IOUT exceeded safety limit IOUT_OC_FAULT_LIMIT)", True),
            ("Uvin", 3, "Bit 3 (0x08) - Input Undervoltage Fault (Measured VIN fell below safety limit VIN_UV_FAULT_LIMIT)", True),
            ("Tmp", 2, "Bit 2 (0x04) - Temperature Fault or Warning (Overtemperature detected on power module)", True),
            ("Cml", 1, "Bit 1 (0x02) - Communication, Memory, PEC or Logic Fault active in STATUS_CML register (0x7E)", True),
            ("None", 0, "Bit 0 (0x01) - None of the above: Other hardware condition or sub-warning is asserted", False)
        ]
    },
    {
        "reg": "STATUS_WORD", "code": 0x79, "title": "STATUS_WORD (0x79)",
        "chips": [
            ("Vout", 15, "Bit 15 (0x8000) - Output voltage fault or warning condition is currently active in STATUS_VOUT (0x7A)", True),
            ("I/Pout", 14, "Bit 14 (0x4000) - Output current or power fault condition is active in STATUS_IOUT (0x7B)", True),
            ("IN", 13, "Bit 13 (0x2000) - Input voltage, current or power fault condition is active in STATUS_INPUT (0x7C)", True),
            ("Mfr", 12, "Bit 12 (0x1000) - Manufacturer specific custom diagnostic fault flag is asserted in STATUS_MFR (0x80)", False),
            ("PG", 11, "Bit 11 (0x0800) - Power Good signal is Negated / Inactive (#PG: Output voltage is out of regulation)", True),
            ("Fans", 10, "Bit 10 (0x0400) - Cooling fan speed fault or airflow restriction active in STATUS_FANS (0x81)", False),
            ("Oth", 9, "Bit 9 (0x0200) - Other internal hardware fault condition detected", False),
            ("Unk", 8, "Bit 8 (0x0100) - Unknown or unclassified hardware event flag", False),
            ("Busy", 7, "Bit 7 (0x0080) - Device is busy processing an internal command or task", False),
            ("Off", 6, "Bit 6 (0x0040) - Output power conversion stage is DISABLED (Unit is turned OFF)", False),
            ("Ovout", 5, "Bit 5 (0x0020) - Output Overvoltage Fault (VOUT > VOUT_OV_FAULT_LIMIT)", True),
            ("Oiout", 4, "Bit 4 (0x0010) - Output Overcurrent Fault (IOUT > IOUT_OC_FAULT_LIMIT)", True),
            ("Uvin", 3, "Bit 3 (0x0008) - Input Undervoltage Fault (VIN < VIN_UV_FAULT_LIMIT)", True),
            ("Tmp", 2, "Bit 2 (0x0004) - Temperature Fault or Warning on power stage", True),
            ("Cml", 1, "Bit 1 (0x0002) - Communication, Memory, PEC or Logic Fault active in STATUS_CML (0x7E)", True),
            ("None", 0, "Bit 0 (0x0001) - Other hardware condition asserted in status registers", False)
        ]
    },
    {
        "reg": "STATUS_VOUT", "code": 0x7A, "title": "STATUS_VOUT (0x7A)",
        "chips": [
            ("Ovout", 7, "Bit 7 (0x80) - Output Overvoltage Fault shutdown threshold exceeded (VOUT > VOUT_OV_FAULT_LIMIT)", True),
            ("Ov", 6, "Bit 6 (0x40) - Output Overvoltage Warning advisory threshold exceeded (VOUT > VOUT_OV_WARN_LIMIT)", False),
            ("Uvout", 5, "Bit 5 (0x20) - Output Undervoltage Warning advisory threshold reached (VOUT < VOUT_UV_WARN_LIMIT)", False),
            ("Uv", 4, "Bit 4 (0x10) - Output Undervoltage Fault shutdown threshold reached (VOUT < VOUT_UV_FAULT_LIMIT)", True),
            ("MaxVo", 3, "Bit 3 (0x08) - VOUT Max Warning: Target voltage setpoint was clamped to maximum safety limit VOUT_MAX", False),
            ("TonM", 2, "Bit 2 (0x04) - Turn-On Time-Over Fault: Output voltage failed to reach regulation within TON_MAX delay", True),
            ("ToffM", 1, "Bit 1 (0x02) - Turn-Off Time-Over Warning: Output voltage took longer than expected to ramp down", False),
            ("Track", 0, "Bit 0 (0x01) - Output Voltage Tracking Error between master and slave voltage rails", False)
        ]
    },
    {
        "reg": "STATUS_IOUT", "code": 0x7B, "title": "STATUS_IOUT (0x7B)",
        "chips": [
            ("Oiout", 7, "Bit 7 (0x80) - Output Overcurrent Fault shutdown threshold exceeded (IOUT > IOUT_OC_FAULT_LIMIT)", True),
            ("OioLv", 6, "Bit 6 (0x40) - Output Overcurrent Low-Voltage Fault shutdown (Overcurrent occurred with sagging voltage)", True),
            ("Oc", 5, "Bit 5 (0x20) - Output Overcurrent Warning advisory threshold reached (IOUT > IOUT_OC_WARN_LIMIT)", False),
            ("Uio", 4, "Bit 4 (0x10) - Output Undercurrent Fault shutdown threshold reached (IOUT < IOUT_UC_FAULT_LIMIT)", True),
            ("Cshr", 3, "Bit 3 (0x08) - Current Sharing Loop Fault: Unequal current sharing among parallel power stages", False),
            ("InPlm", 2, "Bit 2 (0x04) - In Power Limiting / Current Limiting active (Power stage is operating out of voltage regulation)", False),
            ("Opout", 1, "Bit 1 (0x02) - Output Overpower Fault shutdown threshold exceeded (POUT > POUT_OP_FAULT_LIMIT)", True),
            ("Op", 0, "Bit 0 (0x01) - Output Overpower Warning advisory threshold reached (POUT > POUT_OP_WARN_LIMIT)", False)
        ]
    },
    {
        "reg": "STATUS_INPUT", "code": 0x7C, "title": "STATUS_INPUT (0x7C)",
        "chips": [
            ("Ovin", 7, "Bit 7 (0x80) - Input Overvoltage Fault shutdown threshold exceeded (VIN > VIN_OV_FAULT_LIMIT)", True),
            ("Ov", 6, "Bit 6 (0x40) - Input Overvoltage Warning advisory threshold reached (VIN > VIN_OV_WARN_LIMIT)", False),
            ("Uv", 5, "Bit 5 (0x20) - Input Undervoltage Warning advisory threshold reached (VIN < VIN_UV_WARN_LIMIT)", False),
            ("Uvin", 4, "Bit 4 (0x10) - Input Undervoltage Fault shutdown threshold reached (VIN < VIN_UV_FAULT_LIMIT)", True),
            ("UnOff", 3, "Bit 3 (0x08) - Unit Turned Off Due to Low Input Voltage: Source voltage dropped below turn-off threshold VIN_OFF", False),
            ("Oiin", 2, "Bit 2 (0x04) - Input Overcurrent Fault shutdown threshold exceeded (IIN > IIN_OC_FAULT_LIMIT)", True),
            ("Oc", 1, "Bit 1 (0x02) - Input Overcurrent Warning advisory threshold reached (IIN > IIN_OC_WARN_LIMIT)", False),
            ("Opin", 0, "Bit 0 (0x01) - Input Overpower Warning advisory threshold reached (PIN > PIN_OP_WARN_LIMIT)", False)
        ]
    },
    {
        "reg": "STATUS_TEMPERATURE", "code": 0x7D, "title": "STATUS_TEMPERATURE (0x7D)",
        "chips": [
            ("OverTemp", 7, "Bit 7 (0x80) - Overtemperature Fault shutdown threshold exceeded (Temperature > OT_FAULT_LIMIT)", True),
            ("OtWarn", 6, "Bit 6 (0x40) - Overtemperature Warning advisory threshold reached (Temperature > OT_WARN_LIMIT)", False),
            ("UtWarn", 5, "Bit 5 (0x20) - Undertemperature Warning advisory threshold reached (Temperature < UT_WARN_LIMIT)", False),
            ("UnderTemp", 4, "Bit 4 (0x10) - Undertemperature Fault shutdown threshold reached (Temperature < UT_FAULT_LIMIT)", True),
            ("Sns1", 3, "Bit 3 (0x08) - Thermal Sensor 1 Hardware Fault (Open circuit, short circuit or invalid ADC readout)", False),
            ("Sns2", 2, "Bit 2 (0x04) - Thermal Sensor 2 Hardware Fault (Open circuit, short circuit or invalid ADC readout)", False),
            ("Sns3", 1, "Bit 1 (0x02) - Thermal Sensor 3 Hardware Fault (Open circuit, short circuit or invalid ADC readout)", False)
        ]
    },
    {
        "reg": "STATUS_CML", "code": 0x7E, "title": "STATUS_CML (0x7E)",
        "chips": [
            ("Command", 7, "Bit 7 (0x80) - Invalid or Unsupported PMBus Command Code received by the slave microcontroller", True),
            ("Data", 6, "Bit 6 (0x40) - Invalid or Out-of-Range Data Parameter byte received in PMBus transaction", True),
            ("PEC", 5, "Bit 5 (0x20) - Packet Error Check (PEC / CRC-8) Mismatch Failure detected on I2C frame", True),
            ("Mmory", 4, "Bit 4 (0x10) - Memory Corruption Fault: Checksum error in internal NVM, EEPROM or Flash memory", True),
            ("Proc", 3, "Bit 3 (0x08) - Processor or Logic Execution Fault in internal microcontroller sequencer", True),
            ("OthC", 1, "Bit 1 (0x02) - Other I2C / SMBus Frame Communication Fault (Bus lockup, timeout or unexpected stop)", True),
            ("OthML", 0, "Bit 0 (0x01) - Other Internal Memory or Logic Sequencer Error", False)
        ]
    }
]

def twos_complement(val: int, bits: int) -> int:
    if val is None: return 0
    return val - (1 << bits) if (val & (1 << (bits - 1))) else val

def linear11_decode(raw: int) -> float:
    if raw is None: return 0.0
    m = twos_complement(raw & 0x7FF, 11)
    e = twos_complement((raw >> 11) & 0x1F, 5)
    return m * (2.0 ** e)

def linear11_encode(val: float) -> int:
    if val is None or val == 0: return 0
    abs_v = abs(val)
    e = math.floor(math.log2(abs_v / 1023.0)) if abs_v > 0 else 0
    e = max(-16, min(15, e))
    m = round(val / (2.0 ** e))
    while (m > 1023 or m < -1024) and e < 15:
        e += 1
        m = round(val / (2.0 ** e))
    e_u = e & 0x1F
    m_u = m & 0x7FF
    return (e_u << 11) | m_u

def vout_decode(raw: int, mode_byte: int = 0x15) -> float:
    if raw is None: return 0.0
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
    if act == 0x00:
        return "Ignore fault"
    elif act == 0x01:
        return "Continue w/ Delay"
    elif act == 0x02:
        return "Disable 'til Cleared"
    elif act == 0x03:
        return f"Retry (Count={retry})" if retry else "Hiccup / Retry"
    return f"0x{resp_byte:02X}"

def format_decoded_value(code: int, rtype: str, raw: int, mode_byte: int = 0x15) -> str:
    if raw is None or raw == -1:
        return "n/a"
    if rtype == "byte_resp":
        return decode_fault_response(raw)
    elif rtype == "byte_vmode":
        exp = twos_complement(raw & 0x1F, 5)
        return f"Linear (exp= {exp})"
    elif rtype == "byte_op":
        if raw == 0x80: return "Immediate ON"
        elif raw == 0x00: return "Immediate OFF"
        elif raw == 0x98: return "Margin High"
        elif raw == 0x94: return "Margin Low"
        return f"0x{raw:02X}"
    elif rtype == "byte_onoff":
        return "Ignore all" if raw == 0x00 else f"Config (0x{raw:02X})"
    elif rtype == "byte_wp":
        return "DISABLE Other" if raw == 0x00 else (f"Enable ALL" if raw == 0x80 else f"0x{raw:02X}")
    elif rtype == "byte_cap":
        pec = "PEC " if (raw & 0x80) else ""
        spd = "400kHz " if ((raw >> 5) & 0x03) == 1 else "100kHz "
        alert = "SMBAL#" if (raw & 0x10) else ""
        return f"{pec}{spd}{alert}".strip() or f"0x{raw:02X}"
    elif rtype == "byte_page":
        return f"{raw:02X}h (All)" if raw == 0xFF else f"{raw:02X}h (Ch {raw})"
    elif rtype in ["byte", "byte_hex"]:
        return f"0x{raw:02X}"
    elif rtype == "word_hex":
        return f"0x{raw:04X}"
    elif rtype == "word_vout":
        val = vout_decode(raw, mode_byte)
        return f"{val:.3f} V"
    elif rtype == "word_volt":
        val = linear11_decode(raw)
        return f"{val:.3f} V"
    elif rtype == "word_curr":
        val = linear11_decode(raw)
        return f"{val:.2f} A"
    elif rtype == "word_power":
        val = linear11_decode(raw)
        return f"{val:.2f} W"
    elif rtype == "word_temp":
        val = linear11_decode(raw)
        return f"{val:.2f} C"
    elif rtype == "word_mohm":
        val = linear11_decode(raw)
        return f"{val:.3f} mOhm"
    elif rtype == "word_droop":
        val = linear11_decode(raw)
        return f"{val:.4f} mV/A"
    elif rtype == "word_v_ms":
        val = linear11_decode(raw)
        return f"{val:.4f} V/ms"
    elif rtype == "word_ms":
        val = linear11_decode(raw)
        return f"{val:.2f} ms"
    elif rtype == "word_percent":
        val = linear11_decode(raw)
        return f"{val:.1f} %"
    elif rtype == "word_khz":
        val = linear11_decode(raw)
        return f"{val:.1f} kHz"
    elif rtype == "word_rpm":
        val = linear11_decode(raw)
        return f"{val:.0f} RPM"
    elif rtype == "word_linear_ratio":
        val = linear11_decode(raw)
        return f"{val:.4f}"
    else:
        val = linear11_decode(raw)
        return f"{val:.2f}"

class DataLogger:
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

    def log(self, telem: dict):
        if not self.is_logging or not telem: return
        with self._lock:
            self.records.append({
                "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "VIN_V": telem.get("vin", 0),
                "VOUT_V": telem.get("vout", 0),
                "IIN_A": telem.get("iin", 0),
                "IOUT_A": telem.get("iout", 0),
                "PIN_W": telem.get("pin", 0),
                "POUT_W": telem.get("pout", 0),
                "Temp1_C": telem.get("temp1", 0),
                "Temp2_C": telem.get("temp2", 0)
            })
            if len(self.records) > 5000:
                self.records.pop(0)

    def get_csv(self) -> str:
        with self._lock:
            if not self.records:
                return "Timestamp,VIN_V,VOUT_V,IIN_A,IOUT_A,PIN_W,POUT_W,Temp1_C,Temp2_C\n"
            out = io.StringIO()
            w = csv.DictWriter(out, fieldnames=list(self.records[0].keys()))
            w.writeheader()
            w.writerows(self.records)
            return out.getvalue()

def list_user_i2c_buses() -> list:
    buses = []
    if os.path.exists('/dev'):
        for f in os.listdir('/dev'):
            if f.startswith('i2c-'):
                try:
                    num = int(f.split('-')[1])
                    if num != 0 and num not in buses:
                        buses.append(num)
                except Exception: pass
    return sorted(buses, reverse=True) if buses else [2]

class HardwareManager:
    def __init__(self, bus_id=2, addr=0x04):
        self.bus_id = bus_id
        self.addr = addr
        self.bus = None
        self.logger = DataLogger()
        self._lock = threading.Lock()
        
        # Sample & Hold Fallback Cache
        self._cache = {
            "vin": 54.0, "vout": 12.0, "iin": 0.0, "iout": 0.0,
            "pin": 0.0, "pout": 0.0, "eff": 0.0, "temp1": 25.0, "temp2": 25.0,
            "op": 0x80, "mode": 0x15, "sw": 0x0000,
            "vin_on": 40.0, "vin_off": 36.0, "pg_on": 10.8, "pg_off": 10.2,
            "vout_cmd": 12.0, "ton_rise": 10.0, "ton_delay": 0.0, "vout_rate": 1.0,
            "onoff_cfg": "0x00", "consecutive_errors": 0
        }
        self.connect(bus_id, addr)

    def connect(self, bus_id, addr):
        with self._lock:
            self.bus_id = 2 if bus_id == 0 else bus_id
            self.addr = addr
            if self.bus:
                try: self.bus.close()
                except Exception: pass
                self.bus = None
            try:
                from smbus2 import SMBus
                self.bus = SMBus(self.bus_id)
            except Exception: pass

    def clear_faults(self):
        """Send Byte PMBus command 0x03 to clear faults and settle."""
        with self._lock:
            try:
                if self.bus:
                    self.bus.write_byte(self.addr, 0x03)
                    time.sleep(0.15)
                    return True
            except Exception:
                try:
                    if self.bus:
                        self.bus.write_byte_data(self.addr, 0x03, 0x00)
                        time.sleep(0.15)
                        return True
                except Exception: pass
        return False

    def set_vout(self, target_v: float) -> bool:
        """Write VOUT_COMMAND (0x21) in Linear16 format."""
        with self._lock:
            try:
                if not self.bus: return False
                mode = self.read_byte_retry(0x20, self._cache["mode"])
                raw_word = vout_encode(target_v, mode)
                self.bus.write_word_data(self.addr, 0x21, raw_word)
                time.sleep(0.04)
                self._cache["vout_cmd"] = target_v
                return True
            except Exception:
                return False

    def set_operation(self, op_byte: int) -> bool:
        """Write OPERATION (0x01) byte command."""
        with self._lock:
            try:
                if not self.bus: return False
                self.bus.write_byte_data(self.addr, 0x01, op_byte & 0xFF)
                time.sleep(0.04)
                self._cache["op"] = op_byte
                return True
            except Exception:
                return False

    def set_startup_thresholds(self, vin_on=None, vin_off=None, ton_rise=None) -> bool:
        """Write VIN_ON (0x35), VIN_OFF (0x36), TON_RISE (0x61) Linear11 commands."""
        with self._lock:
            try:
                if not self.bus: return False
                if vin_on is not None:
                    self.bus.write_word_data(self.addr, 0x35, linear11_encode(vin_on))
                    time.sleep(0.02)
                    self._cache["vin_on"] = vin_on
                if vin_off is not None:
                    self.bus.write_word_data(self.addr, 0x36, linear11_encode(vin_off))
                    time.sleep(0.02)
                    self._cache["vin_off"] = vin_off
                if ton_rise is not None:
                    self.bus.write_word_data(self.addr, 0x61, linear11_encode(ton_rise))
                    time.sleep(0.02)
                    self._cache["ton_rise"] = ton_rise
                return True
            except Exception:
                return False

    def read_byte_retry(self, reg, default=0):
        if not self.bus: return default
        for attempt in range(2):
            try:
                val = self.bus.read_byte_data(self.addr, reg)
                time.sleep(0.003) # 3ms pacing delay
                return val
            except Exception:
                time.sleep(0.005)
        return default

    def read_word_retry(self, reg, default=0):
        if not self.bus: return default
        for attempt in range(2):
            try:
                val = self.bus.read_word_data(self.addr, reg)
                time.sleep(0.003) # 3ms pacing delay
                return val
            except Exception:
                time.sleep(0.005)
        return default

    def read_block_ascii(self, reg, default="n/a"):
        with self._lock:
            try:
                if not self.bus: return default
                data = self.bus.read_i2c_block_data(self.addr, reg, 16)
                time.sleep(0.005)
                if data:
                    length = data[0]
                    if 0 < length <= 15:
                        s = "".join([chr(c) for c in data[1:1+length] if 32 <= c <= 126])
                        if s.strip(): return s.strip()
                    s = "".join([chr(c) for c in data if 32 <= c <= 126])
                    if s.strip(): return s.strip()
            except Exception: pass
        return default

    def scan_bus_safe(self, bus_num=2) -> list:
        if bus_num == 0: bus_num = 2
        devices = []

        def worker():
            nonlocal devices
            try:
                from smbus2 import SMBus
                with SMBus(bus_num) as s:
                    for a in range(0x04, 0x78):
                        if a == 0x24 and bus_num == 0: continue
                        try:
                            s.read_byte(a)
                            name = "DC-DC Power Converter" if a == 0x04 else f"I2C Device (0x{a:02X})"
                            icon = "PWR" if a == 0x04 else "I2C"
                            if a in [0x58, 0x59, 0x5A, 0x5B]:
                                name = "CRPS Server Power Supply"
                                icon = "PSU"
                            elif a in [0x48, 0x49, 0x4A, 0x4B]:
                                name = "I2C Temperature Sensor"
                                icon = "TMP"
                            elif a in [0x50, 0x51, 0x52, 0x53]:
                                name = "EEPROM / FRU Memory"
                                icon = "ROM"
                            devices.append({"bus": bus_num, "addr_hex": f"0x{a:02X}", "name": name, "icon": icon})
                        except Exception: pass
            except Exception: pass

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        th.join(timeout=0.8)

        if not devices and bus_num == 2:
            devices.append({"bus": 2, "addr_hex": "0x04", "name": "DC-DC Power Converter", "icon": "PWR"})

        return devices

    def get_telemetry(self) -> dict:
        with self._lock:
            try:
                raw_mode = self.read_byte_retry(0x20, self._cache["mode"])
                if raw_mode: self._cache["mode"] = raw_mode
                mode = self._cache["mode"]

                # Read telemetry with retry & caching
                raw_vin = self.read_word_retry(0x88, None)
                if raw_vin is not None and raw_vin != 0:
                    self._cache["vin"] = linear11_decode(raw_vin)

                raw_vout = self.read_word_retry(0x8B, None)
                if raw_vout is not None:
                    self._cache["vout"] = vout_decode(raw_vout, mode)

                raw_iin = self.read_word_retry(0x89, None)
                if raw_iin is not None:
                    self._cache["iin"] = linear11_decode(raw_iin)

                raw_iout = self.read_word_retry(0x8C, None)
                if raw_iout is not None:
                    self._cache["iout"] = linear11_decode(raw_iout)

                raw_pout = self.read_word_retry(0x96, None)
                if raw_pout is not None and raw_pout != 0:
                    self._cache["pout"] = linear11_decode(raw_pout)
                else:
                    self._cache["pout"] = self._cache["vout"] * self._cache["iout"]

                raw_pin = self.read_word_retry(0x97, None)
                if raw_pin is not None and raw_pin != 0:
                    self._cache["pin"] = linear11_decode(raw_pin)
                else:
                    self._cache["pin"] = self._cache["vin"] * self._cache["iin"]

                raw_t1 = self.read_word_retry(0x8D, None)
                if raw_t1 is not None and raw_t1 != 0:
                    self._cache["temp1"] = linear11_decode(raw_t1)

                raw_t2 = self.read_word_retry(0x8E, None)
                if raw_t2 is not None and raw_t2 != 0:
                    self._cache["temp2"] = linear11_decode(raw_t2)

                raw_op = self.read_byte_retry(0x01, None)
                if raw_op is not None:
                    self._cache["op"] = raw_op

                pin = self._cache["pin"]
                pout = self._cache["pout"]
                eff = min(100.0, max(0.0, (pout / pin) * 100.0)) if pin > 5.0 and pout > 0 else 0.0

                # Read Key Startup & Turn-On Parameters (sample-hold cached)
                raw_vin_on = self.read_word_retry(0x35, None)
                if raw_vin_on is not None: self._cache["vin_on"] = linear11_decode(raw_vin_on)

                raw_vin_off = self.read_word_retry(0x36, None)
                if raw_vin_off is not None: self._cache["vin_off"] = linear11_decode(raw_vin_off)

                raw_pg_on = self.read_word_retry(0x5E, None)
                if raw_pg_on is not None: self._cache["pg_on"] = vout_decode(raw_pg_on, mode)

                raw_pg_off = self.read_word_retry(0x5F, None)
                if raw_pg_off is not None: self._cache["pg_off"] = vout_decode(raw_pg_off, mode)

                raw_vcmd = self.read_word_retry(0x21, None)
                if raw_vcmd is not None: self._cache["vout_cmd"] = vout_decode(raw_vcmd, mode)

                raw_ton = self.read_word_retry(0x61, None)
                if raw_ton is not None: self._cache["ton_rise"] = linear11_decode(raw_ton)

                # Read PMBus Status Registers
                status_vals = {
                    "STATUS_BYTE": self.read_byte_retry(0x78, 0x00),
                    "STATUS_WORD": self.read_word_retry(0x79, self._cache["sw"]),
                    "STATUS_VOUT": self.read_byte_retry(0x7A, 0x00),
                    "STATUS_IOUT": self.read_byte_retry(0x7B, 0x00),
                    "STATUS_INPUT": self.read_byte_retry(0x7C, 0x00),
                    "STATUS_TEMPERATURE": self.read_byte_retry(0x7D, 0x00),
                    "STATUS_CML": self.read_byte_retry(0x7E, 0x00)
                }
                if status_vals["STATUS_WORD"] is not None:
                    self._cache["sw"] = status_vals["STATUS_WORD"]

                # Build status chips decoding
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

                sw = self._cache["sw"]
                op = self._cache["op"]
                data = {
                    "timestamp": datetime.now().strftime("%H:%M:%S"),
                    "bus": self.bus_id,
                    "addr": f"0x{self.addr:02X}",
                    "vin": round(self._cache["vin"], 2),
                    "vout": round(self._cache["vout"], 3),
                    "iin": round(self._cache["iin"], 2),
                    "iout": round(self._cache["iout"], 2),
                    "pin": round(self._cache["pin"], 1),
                    "pout": round(self._cache["pout"], 1),
                    "eff": round(eff, 1),
                    "temp1": round(self._cache["temp1"], 2),
                    "temp2": round(self._cache["temp2"], 2),
                    "op_code": op,
                    "power_state": (op == 0x80) or (self._cache["vout"] > 1.0),
                    "startup": {
                        "vin_on": round(self._cache["vin_on"], 2),
                        "vin_off": round(self._cache["vin_off"], 2),
                        "pg_on": round(self._cache["pg_on"], 2),
                        "pg_off": round(self._cache["pg_off"], 2),
                        "vout_cmd": round(self._cache["vout_cmd"], 2),
                        "ton_rise": round(self._cache["ton_rise"], 2),
                        "ton_delay": round(self._cache["ton_delay"], 2),
                        "vout_rate": round(self._cache["vout_rate"], 4),
                        "onoff_cfg": self._cache["onoff_cfg"]
                    },
                    "status_word": f"0x{sw:04X}",
                    "status_strips": status_strips,
                    "active_faults_total": active_faults_total,
                    "critical_faults": critical_faults,
                    "logger": {
                        "is_logging": self.logger.is_logging,
                        "count": len(self.logger.records)
                    }
                }
                if self.logger.is_logging:
                    self.logger.log(data)
                return data
            except Exception as e:
                return {"bus": self.bus_id, "addr": f"0x{self.addr:02X}", "error": str(e)}

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
            except Exception: pass
            regs.append({"name": r["name"], "hex": f"0x{code:02X}", "cat": r["cat"], "desc": r["desc"], "val": val_str})
        return regs

HTML_APP = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Power Lab Studio Pro</title>
<style>
:root {
    --bg: #0b1120;
    --card: #151e2e;
    --card-hover: #1e293b;
    --input: #0f172a;
    --text: #f8fafc;
    --muted: #94a3b8;
    --blue: #38bdf8;
    --green: #22c55e;
    --red: #ef4444;
    --cyan: #06b6d4;
    --orange: #f97316;
    --yellow: #eab308;
    --purple: #c084fc;
    --indigo: #818cf8;
    --rose: #f43f5e;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, sans-serif; min-height: 100vh; display: flex; flex-direction: column; }
header { background: var(--card); padding: 12px 24px; display: flex; align-items: center; justify-content: space-between; border-bottom: 1px solid #1e293b; flex-wrap: wrap; gap: 10px; }
.nav-btn { background: none; border: 1px solid transparent; color: var(--muted); padding: 8px 16px; border-radius: 6px; font-weight: 700; cursor: pointer; transition: all 0.2s; display: inline-flex; align-items: center; gap: 8px; }
.nav-btn.active { background: #1e293b; color: #fff; border-color: #334155; }

.conn-badge { display: inline-flex; align-items: center; gap: 6px; padding: 4px 10px; border-radius: 20px; font-family: monospace; font-size: 11px; font-weight: 700; background: #0f172a; border: 1px solid #1e293b; color: #94a3b8; transition: all 0.3s; }
.conn-dot { width: 8px; height: 8px; border-radius: 50%; background: #64748b; }
.conn-badge.live { border-color: #16a34a; background: #14532d; color: #86efac; }
.conn-badge.live .conn-dot { background: #22c55e; box-shadow: 0 0 8px #22c55e; }
.conn-badge.reconnecting { border-color: #d97706; background: #78350f; color: #fde68a; }
.conn-badge.reconnecting .conn-dot { background: #f59e0b; animation: pulse 0.8s infinite; }
.conn-badge.offline { border-color: #dc2626; background: #7f1d1d; color: #fca5a5; }
.conn-badge.offline .conn-dot { background: #ef4444; }

.view { display: none; padding: 20px; max-width: 1560px; margin: 0 auto; width: 100%; }
.view.active { display: block; }
.hero { background: var(--card); padding: 18px 20px; border-radius: 8px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px; margin-bottom: 18px; border: 1px solid #1e293b; }
.btn { background: #2563eb; color: #fff; border: none; padding: 8px 16px; border-radius: 6px; font-weight: 700; cursor: pointer; display: inline-flex; align-items: center; gap: 6px; transition: all 0.2s; font-size: 13px; }
.btn:hover { filter: brightness(1.15); transform: translateY(-1px); }
.btn-red { background: #dc2626; }
.btn-green { background: var(--green); }
.btn-gray { background: #1e293b; border: 1px solid #334155; }
.btn-amber { background: #d97706; }

.badge-status { padding: 6px 12px; border-radius: 6px; font-family: monospace; font-size: 12px; font-weight: 700; background: #14532d; color: #86efac; border: 1px solid #16a34a; }
.badge-status.off { background: #7f1d1d; color: #fca5a5; border-color: #dc2626; }

.badge-fault-indicator { padding: 6px 12px; border-radius: 6px; font-family: monospace; font-size: 12px; font-weight: 700; background: #14532d; color: #86efac; border: 1px solid #16a34a; transition: all 0.3s; }
.badge-fault-indicator.alert { background: #991b1b; color: #fca5a5; border-color: #ef4444; animation: pulse 1s infinite; }

@keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }

.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 14px; }
.card { background: var(--card); padding: 18px; border-radius: 8px; cursor: pointer; border: 1px solid #1e293b; transition: all 0.2s; }
.card:hover { border-color: var(--blue); transform: translateY(-2px); background: var(--card-hover); }

.kpi-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; margin-bottom: 18px; }
.kpi { background: var(--card); padding: 14px; border-radius: 8px; border: 1px solid #1e293b; }
.kpi span { font-size: 11px; color: var(--muted); text-transform: uppercase; font-weight: 700; }
.kpi strong { font-family: monospace; font-size: 22px; display: block; margin-top: 4px; }

/* Control Cards */
.ctrl-card { background: var(--card); padding: 18px; border-radius: 8px; border: 1px solid #1e293b; }
.ctrl-card h4 { font-size: 13px; color: #f8fafc; margin-bottom: 12px; display: flex; align-items: center; justify-content: space-between; }

/* Startup Controls Card Grid */
.startup-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; }
.startup-item { background: #0f172a; padding: 12px 14px; border-radius: 6px; border: 1px solid #1e293b; }
.startup-item span { font-size: 11px; color: var(--muted); display: block; }
.startup-item strong { font-family: monospace; font-size: 17px; color: var(--cyan); display: block; margin-top: 2px; }

/* Studio Sub-Tabs */
.studio-tabs { display: flex; gap: 8px; margin-bottom: 16px; border-bottom: 1px solid #1e293b; padding-bottom: 8px; flex-wrap: wrap; }
.s-tab-btn { background: none; border: 1px solid transparent; color: var(--muted); padding: 8px 16px; border-radius: 6px; font-weight: 700; cursor: pointer; transition: all 0.2s; display: inline-flex; align-items: center; gap: 8px; font-size: 13px; }
.s-tab-btn.active { background: #1e293b; color: #fff; border-color: #334155; }
.s-tab-panel { display: none; }
.s-tab-panel.active { display: block; }

/* Oscilloscope Chart Styles with Canvas Container */
.chart-container { background: var(--card); padding: 18px; border-radius: 8px; border: 1px solid #1e293b; margin-bottom: 18px; position: relative; }
.chart-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; flex-wrap: wrap; gap: 10px; }
.chart-legend-grid { display: flex; gap: 10px; align-items: center; font-size: 11px; font-family: monospace; font-weight: 700; flex-wrap: wrap; }
.legend-chip { display: inline-flex; align-items: center; gap: 6px; background: #0f172a; border: 1px solid #1e293b; padding: 4px 8px; border-radius: 4px; cursor: pointer; user-select: none; transition: all 0.15s; }
.legend-chip.inactive { opacity: 0.35; }
.legend-chip input { cursor: pointer; accent-color: var(--blue); }
.legend-color { width: 10px; height: 10px; border-radius: 2px; }
.canvas-wrap { position: relative; width: 100%; height: 280px; background: #070c18; border-radius: 6px; border: 1px solid #1e293b; overflow: hidden; }
canvas { width: 100%; height: 100%; display: block; }

/* Floating Crosshair Readout Tooltip */
.chart-crosshair-tooltip {
    position: absolute;
    background: rgba(15, 23, 42, 0.95);
    border: 1px solid var(--blue);
    border-radius: 6px;
    padding: 8px 12px;
    font-size: 11px;
    font-family: monospace;
    color: #f8fafc;
    pointer-events: none;
    z-index: 20;
    box-shadow: 0 4px 20px rgba(0,0,0,0.8);
    display: none;
    line-height: 1.4;
    white-space: nowrap;
}

/* Status Registers Horizontal Strip Styles */
.status-strip-box { background: var(--card); border: 1px solid #1e293b; border-radius: 8px; padding: 12px 16px; margin-bottom: 12px; }
.status-strip-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
.status-chips-wrap { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; }
.status-chip { font-family: monospace; font-size: 11px; font-weight: 700; padding: 5px 10px; border-radius: 4px; background: #0f172a; border: 1px solid #1e293b; color: #64748b; transition: all 0.15s; user-select: none; cursor: help; }
.status-chip:hover { border-color: var(--blue); color: #fff; transform: translateY(-1px); }
.status-chip.active-red { background: #7f1d1d; color: #fecaca; border-color: #ef4444; box-shadow: 0 0 10px rgba(239, 68, 68, 0.6); animation: pulse 1s infinite; font-weight: 900; }

/* Instant Interactive Custom Tooltip */
.custom-tooltip {
    position: fixed;
    background: #0f172a;
    border: 1px solid var(--cyan);
    color: #f8fafc;
    padding: 10px 14px;
    border-radius: 8px;
    font-size: 12px;
    pointer-events: none;
    z-index: 10000;
    box-shadow: 0 10px 25px rgba(0,0,0,0.85);
    max-width: 340px;
    opacity: 0;
    transform: translateY(6px);
    transition: opacity 0.15s ease, transform 0.15s ease;
    display: none;
    line-height: 1.4;
}
.custom-tooltip.show { opacity: 1; transform: translateY(0); display: block; }
.custom-tooltip.alert { border-color: #ef4444; box-shadow: 0 0 20px rgba(239, 68, 68, 0.5); }

.table { width: 100%; border-collapse: collapse; font-family: monospace; font-size: 12px; }
.table th, .table td { padding: 8px 12px; border-bottom: 1px solid #1e293b; text-align: left; }
.table th { background: var(--input); color: var(--muted); position: sticky; top: 0; z-index: 10; }
select, input { background: var(--input); border: 1px solid #1e293b; color: #fff; padding: 6px 10px; border-radius: 4px; font-family: monospace; }
.icon-box { width: 36px; height: 36px; border-radius: 6px; background: #1e293b; display: flex; align-items: center; justify-content: center; font-weight: bold; color: var(--cyan); }

.toast { position: fixed; bottom: 20px; right: 20px; background: var(--card); border: 1px solid var(--blue); color: #fff; padding: 10px 18px; border-radius: 6px; font-size: 12px; font-weight: 600; opacity: 0; transform: translateY(10px); transition: all 0.2s ease; pointer-events: none; z-index: 999; }
.toast.show { opacity: 1; transform: translateY(0); }
</style>
</head>
<body>
<header>
    <div style="display:flex;align-items:center;gap:10px;">
        <div class="icon-box" style="background:linear-gradient(135deg,#38bdf8,#06b6d4);color:#fff;">PWR</div>
        <div>
            <h1 style="font-size:16px;">Power Lab Studio Pro</h1>
            <p style="font-size:11px;color:var(--muted)">PMBus Universal Power Management &amp; Full Diagnostics</p>
        </div>
    </div>
    <div style="display:flex;gap:12px;align-items:center;">
        <div id="badgeConnState" class="conn-badge live">
            <span class="conn-dot"></span>
            <span id="txtConnState">LIVE</span>
        </div>
        <div style="display:flex;gap:6px;">
            <button class="nav-btn active" id="btnNavHub">Hub &amp; Discovery</button>
            <button class="nav-btn" id="btnNavStudio">Live Studio</button>
        </div>
    </div>
</header>

<!-- VIEW 1: HUB -->
<div class="view active" id="viewHub">
    <div class="hero">
        <div>
            <h2>Hardware I2C Discovery Hub</h2>
            <p style="color:var(--muted);font-size:13px;margin-top:4px;">Select an I2C bus and click on any discovered card to control and monitor it.</p>
        </div>
        <div style="display:flex;gap:10px;align-items:center;">
            <span>I2C Bus:</span>
            <select id="selBus"></select>
            <button class="btn" id="btnScan">Scan Bus</button>
        </div>
    </div>
    <h3 style="margin-bottom:12px;font-size:14px;color:var(--muted);">DISCOVERED MODULES &amp; BOARDS:</h3>
    <div class="grid" id="gridDevices"></div>
</div>

<!-- VIEW 2: STUDIO (LIVE TARGET BOARD) -->
<div class="view" id="viewStudio">
    <!-- Top Target Bar -->
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:16px;background:var(--card);padding:12px 20px;border-radius:8px;border:1px solid #1e293b;flex-wrap:wrap;gap:10px;">
        <button class="btn btn-gray" id="btnBack">&larr; Back to Device List</button>
        <strong style="color:var(--cyan);font-family:monospace;font-size:14px;" id="lblTarget">TARGET: Bus #2 - 0x04</strong>
        <div style="display:flex;gap:10px;align-items:center;">
            <span class="badge-status" id="badgePower">STATUS: ENABLED (ON)</span>
            <span class="badge-fault-indicator" id="badgeFaultLive">0 FAULTS</span>
            <button class="btn btn-red" id="btnClearFaults">Clear Faults (0x03)</button>
        </div>
    </div>

    <!-- Live Telemetry KPI Gauges -->
    <div class="kpi-grid">
        <div class="kpi"><span>Input Voltage (VIN)</span><strong style="color:var(--cyan);" id="valVin">0.000 V</strong></div>
        <div class="kpi"><span>Output Voltage (VOUT)</span><strong style="color:var(--blue);" id="valVout">0.000 V</strong></div>
        <div class="kpi"><span>Input Current (IIN)</span><strong style="color:var(--yellow);" id="valIin">0.00 A</strong></div>
        <div class="kpi"><span>Output Current (IOUT)</span><strong style="color:var(--orange);" id="valIout">0.00 A</strong></div>
        <div class="kpi"><span>Input Power (PIN)</span><strong style="color:var(--indigo);" id="valPin">0.0 W</strong></div>
        <div class="kpi"><span>Output Power (POUT)</span><strong style="color:var(--purple);" id="valPout">0.0 W</strong></div>
        <div class="kpi"><span>Efficiency (Eff)</span><strong style="color:var(--green);" id="valEff">0.0 %</strong></div>
        <div class="kpi"><span>Temperatures (T1/T2)</span><strong style="color:var(--red);font-size:16px;" id="valTemp">-- C</strong></div>
    </div>

    <!-- REAL-TIME OSCILLOSCOPE / MULTI-TRACE TELEMETRY CHART -->
    <div class="chart-container">
        <div class="chart-header">
            <div>
                <strong style="font-size:13px;">Real-Time Telemetry Oscilloscope</strong>
                <span style="font-size:11px;color:var(--muted);margin-left:8px;">(Left Y-Axis: Volts | Right Y-Axis: Amps, Watts, Deg C)</span>
            </div>
            <div class="chart-legend-grid">
                <label class="legend-chip"><input type="checkbox" id="chkVin" checked><div class="legend-color" style="background:var(--cyan);"></div><span style="color:var(--cyan);">VIN (<span id="lgVin">0.0</span>V)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkVout" checked><div class="legend-color" style="background:var(--blue);"></div><span style="color:var(--blue);">VOUT (<span id="lgVout">0.0</span>V)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkIin" checked><div class="legend-color" style="background:var(--yellow);"></div><span style="color:var(--yellow);">IIN (<span id="lgIin">0.0</span>A)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkIout" checked><div class="legend-color" style="background:var(--orange);"></div><span style="color:var(--orange);">IOUT (<span id="lgIout">0.0</span>A)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkPin"><div class="legend-color" style="background:var(--indigo);"></div><span style="color:var(--indigo);">PIN (<span id="lgPin">0</span>W)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkPout"><div class="legend-color" style="background:var(--purple);"></div><span style="color:var(--purple);">POUT (<span id="lgPout">0</span>W)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkT1" checked><div class="legend-color" style="background:var(--red);"></div><span style="color:var(--red);">T1 (<span id="lgT1">0</span>C)</span></label>
                <label class="legend-chip"><input type="checkbox" id="chkT2"><div class="legend-color" style="background:var(--rose);"></div><span style="color:var(--rose);">T2 (<span id="lgT2">0</span>C)</span></label>
            </div>
        </div>
        <div class="canvas-wrap" id="canvasContainer">
            <canvas id="liveTelemetryCanvas"></canvas>
            <div id="chartCrosshairTooltip" class="chart-crosshair-tooltip"></div>
        </div>
    </div>

    <!-- Studio Internal Sub-Tabs Navigation -->
    <div class="studio-tabs">
        <button class="s-tab-btn active" id="tabBtnControls">
            Voltage &amp; Power Controls (I2C Settings)
        </button>
        <button class="s-tab-btn" id="tabBtnFaults">
            PMBus Bitwise Status Matrix (0x78 - 0x7E)
            <span class="status-chip" style="padding:2px 6px;font-size:10px;" id="badgeTabFaults">0 FAULT</span>
        </button>
        <button class="s-tab-btn" id="tabBtnRegs">
            Full PMBus Registers Table
            <span class="status-chip" style="padding:2px 6px;font-size:10px;">80+</span>
        </button>
        <button class="s-tab-btn" id="tabBtnLogger">Data Logger (CSV)</button>
    </div>

    <!-- SUB-PANEL 1: INTERACTIVE VOLTAGE & POWER CONTROLS -->
    <div class="s-tab-panel active" id="panelControls">
        <div style="display:grid;grid-template-columns:repeat(auto-fit, minmax(360px, 1fr));gap:16px;margin-bottom:16px;">
            <!-- CARD 1: VOUT TARGET SETPOINT ADJUSTMENT -->
            <div class="ctrl-card">
                <h4>
                    <span>Target Output Voltage (VOUT_COMMAND - 0x21)</span>
                    <strong style="color:var(--blue);font-family:monospace;" id="lblCurrentVoutCmd">12.00 V</strong>
                </h4>
                <p style="color:var(--muted);font-size:12px;margin-bottom:12px;">Adjust the regulated output voltage setpoint across the Linear16 range.</p>
                <div style="display:flex;gap:10px;align-items:center;margin-bottom:12px;">
                    <input type="number" id="txtSetVout" step="0.05" min="5.0" max="15.0" value="12.00" style="width:120px;font-size:14px;font-weight:700;">
                    <button class="btn" id="btnApplyVout">Apply VOUT (0x21)</button>
                </div>
                <div style="display:flex;gap:8px;flex-wrap:wrap;">
                    <button class="btn btn-gray" style="font-size:11px;padding:4px 8px;" onclick="quickSetVout(11.50)">Preset 11.50 V</button>
                    <button class="btn btn-gray" style="font-size:11px;padding:4px 8px;" onclick="quickSetVout(12.00)">Preset 12.00 V</button>
                    <button class="btn btn-gray" style="font-size:11px;padding:4px 8px;" onclick="quickSetVout(12.50)">Preset 12.50 V</button>
                </div>
            </div>

            <!-- CARD 2: MARGINING & POWER STAGE OPERATION -->
            <div class="ctrl-card">
                <h4>
                    <span>Power Stage &amp; Margining (OPERATION - 0x01)</span>
                    <strong style="color:var(--cyan);font-family:monospace;" id="lblOpState">0x80 (ON)</strong>
                </h4>
                <p style="color:var(--muted);font-size:12px;margin-bottom:12px;">Switch output regulation state or apply high/low margining offsets.</p>
                <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px;">
                    <button class="btn btn-green" id="btnOpOn">Power ON (0x80)</button>
                    <button class="btn btn-amber" id="btnOpMarginHigh">Margin High (0x98)</button>
                    <button class="btn btn-amber" id="btnOpMarginLow">Margin Low (0x94)</button>
                    <button class="btn btn-red" id="btnOpOff">Power OFF (0x00)</button>
                </div>
                <small style="color:var(--muted);font-size:11px;">* Power OFF will disable the 12V rail. Power ON re-enables regulation.</small>
            </div>

            <!-- CARD 3: STARTUP & INPUT VOLTAGE THRESHOLDS -->
            <div class="ctrl-card" style="grid-column: 1 / -1;">
                <h4>
                    <span>Startup Voltages &amp; Soft-Start (VIN_ON, VIN_OFF, TON_RISE)</span>
                    <button class="btn btn-gray" style="font-size:11px;padding:4px 8px;" id="btnApplyStartup">Save Thresholds</button>
                </h4>
                <div class="startup-grid" style="margin-top:10px;">
                    <div class="startup-item">
                        <span>Turn-On Threshold (VIN_ON - 0x35)</span>
                        <div style="display:flex;gap:6px;align-items:center;margin-top:4px;">
                            <input type="number" id="txtVinOn" step="0.5" style="width:100px;" value="40.0">
                            <span style="color:#fff;">V</span>
                        </div>
                    </div>
                    <div class="startup-item">
                        <span>Turn-Off Threshold (VIN_OFF - 0x36)</span>
                        <div style="display:flex;gap:6px;align-items:center;margin-top:4px;">
                            <input type="number" id="txtVinOff" step="0.5" style="width:100px;" value="36.0">
                            <span style="color:#fff;">V</span>
                        </div>
                    </div>
                    <div class="startup-item">
                        <span>Soft-Start Rise Time (TON_RISE - 0x61)</span>
                        <div style="display:flex;gap:6px;align-items:center;margin-top:4px;">
                            <input type="number" id="txtTonRise" step="1.0" style="width:100px;" value="10.0">
                            <span style="color:#fff;">ms</span>
                        </div>
                    </div>
                    <div class="startup-item">
                        <span>Power Good Assertion (0x5E / 0x5F)</span>
                        <strong style="font-size:14px;color:#fff;" id="lblPgThresh">ON: 10.8V / OFF: 10.2V</strong>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- SUB-PANEL 2: EXACT BITWISE PMBUS STATUS STRIPS -->
    <div class="s-tab-panel" id="panelFaults">
        <!-- Top Status Alert Banner -->
        <div id="bannerFaultStatus" style="padding:12px 18px;border-radius:8px;margin-bottom:16px;border:1px solid #16a34a;background:#14532d;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px;">
            <div>
                <h3 id="txtFaultTitle" style="color:#86efac;font-size:13px;">SYSTEM NOMINAL - NO FAULTS</h3>
                <p id="txtFaultSubtitle" style="color:#bbf7d0;font-size:11px;margin-top:2px;">Hover mouse over any status bit chip below to see its exact hardware description.</p>
            </div>
            <div style="display:flex;gap:10px;align-items:center;">
                <strong style="font-family:monospace;font-size:12px;color:#fff;" id="lblStatusWord">STATUS_WORD: 0x0000</strong>
                <button class="btn btn-red" id="btnClearFaultsPanel" style="font-size:12px;padding:6px 12px;">Clear Faults (0x03)</button>
            </div>
        </div>

        <h4 style="margin-bottom:12px;font-size:12px;color:var(--muted);text-transform:uppercase;">LIVE STATUS REGISTER BIT STRIPS (HOVER MOUSE FOR DETAILS) :</h4>
        <div id="boxStatusStrips">
            <!-- Dynamically populated bit strips -->
        </div>
    </div>

    <!-- SUB-PANEL 3: REGISTERS & LIMITS TABLE WITH SEARCH & FILTER -->
    <div class="s-tab-panel" id="panelRegs">
        <div style="background:var(--card);padding:16px;border-radius:8px;border:1px solid #1e293b;">
            <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;flex-wrap:wrap;gap:10px;">
                <div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap;">
                    <span>Category:</span>
                    <select id="selRegCat" style="max-width:260px;">
                        <option value="ALL">All PMBus Categories</option>
                        <option value="Startup & Input (VIN)">Startup &amp; Input Voltages (VIN)</option>
                        <option value="Output Voltage (VOUT)">Output Voltage &amp; Limits (VOUT)</option>
                        <option value="Sequencing & Timings">Sequencing &amp; Soft-Start (TON/TOFF)</option>
                        <option value="Current & Power (IOUT)">Output Current &amp; Power (IOUT)</option>
                        <option value="Thermal & Cooling">Thermal &amp; Cooling Fans</option>
                        <option value="Regulation & Timing">Regulation &amp; Switching Clock</option>
                        <option value="Status & Diagnostics">Status &amp; Diagnostics</option>
                        <option value="Manufacturer & Ratings">Manufacturer Info &amp; Ratings</option>
                        <option value="Configuration">Configuration</option>
                    </select>
                    <input type="text" id="txtRegSearch" placeholder="Search (e.g. VIN_ON, VIN_OFF, TON_RISE, 0x35)..." style="width:280px;">
                </div>
                <button class="btn btn-gray" id="btnRefreshRegs" style="font-size:12px;padding:6px 12px;">Refresh All Registers</button>
            </div>
            <div style="max-height:600px;overflow-y:auto;border:1px solid #1e293b;border-radius:6px;">
                <table class="table">
                    <thead>
                        <tr>
                            <th style="width:240px;">REGISTER NAME</th>
                            <th style="width:80px;">HEX</th>
                            <th style="width:190px;">CATEGORY</th>
                            <th>STANDARD PMBUS DESCRIPTION</th>
                            <th style="width:200px;">DECODED VALUE</th>
                        </tr>
                    </thead>
                    <tbody id="tblRegs">
                        <tr><td colspan="5" style="text-align:center;padding:20px;">Loading PMBus registers...</td></tr>
                    </tbody>
                </table>
            </div>
        </div>
    </div>

    <!-- SUB-PANEL 4: CSV LOGGER -->
    <div class="s-tab-panel" id="panelLogger">
        <div class="hero">
            <div>
                <strong>Continuous CSV Data Logger</strong>
                <p style="color:var(--muted);font-size:12px;margin-top:4px;" id="lblLog">0 data records captured for this session</p>
            </div>
            <div style="display:flex;gap:10px;">
                <button class="btn" id="btnLogger">Start Recording</button>
                <button class="btn btn-gray" id="btnDownloadCsv">Download CSV</button>
            </div>
        </div>
    </div>
</div>

<!-- Instant Interactive Floating Tooltip -->
<div id="tooltipBox" class="custom-tooltip">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;gap:8px;">
        <strong id="ttTitle" style="color:var(--cyan);font-family:monospace;font-size:12px;">FLAG</strong>
        <span id="ttBadge" style="font-size:9px;font-weight:700;padding:2px 6px;border-radius:4px;background:#1e293b;color:#94a3b8;">NORMAL (0)</span>
    </div>
    <div style="font-size:11px;color:#cbd5e1;margin-bottom:4px;" id="ttReg">STATUS REGISTER</div>
    <div style="font-size:11px;color:#94a3b8;" id="ttDesc">Technical Description</div>
</div>

<div id="toastBox" class="toast"></div>

<script>
document.addEventListener("DOMContentLoaded", () => {
    let currentBus = 2, currentAddr = "0x04";
    let allRegisters = [];

    // Telemetry History Buffer for Full Multi-Trace Chart
    const MAX_CHART_POINTS = 60;
    const histVin = [], histVout = [], histIin = [], histIout = [], histPin = [], histPout = [], histT1 = [], histT2 = [];
    const histTimestamps = [];

    // Auto-Healing SSE Connection & Watchdog Variables
    let sseSource = null;
    let lastDataTime = Date.now();
    let isReconnecting = false;
    let reconnectTimeout = null;

    const badgeConnState = document.getElementById("badgeConnState"), txtConnState = document.getElementById("txtConnState");

    const viewHub = document.getElementById("viewHub"), viewStudio = document.getElementById("viewStudio");
    const btnNavHub = document.getElementById("btnNavHub"), btnNavStudio = document.getElementById("btnNavStudio");
    const selBus = document.getElementById("selBus"), btnScan = document.getElementById("btnScan"), gridDevices = document.getElementById("gridDevices");
    const btnBack = document.getElementById("btnBack"), lblTarget = document.getElementById("lblTarget");
    const valVin = document.getElementById("valVin"), valVout = document.getElementById("valVout"), valIin = document.getElementById("valIin"), valIout = document.getElementById("valIout");
    const valPin = document.getElementById("valPin"), valPout = document.getElementById("valPout"), valEff = document.getElementById("valEff"), valTemp = document.getElementById("valTemp");
    const badgePower = document.getElementById("badgePower"), badgeFaultLive = document.getElementById("badgeFaultLive");
    const btnClearFaults = document.getElementById("btnClearFaults"), btnClearFaultsPanel = document.getElementById("btnClearFaultsPanel"), btnRefreshRegs = document.getElementById("btnRefreshRegs");
    const btnLogger = document.getElementById("btnLogger"), btnDownloadCsv = document.getElementById("btnDownloadCsv"), lblLog = document.getElementById("lblLog"), tblRegs = document.getElementById("tblRegs");

    // Interactive Controls Elements
    const txtSetVout = document.getElementById("txtSetVout"), btnApplyVout = document.getElementById("btnApplyVout"), lblCurrentVoutCmd = document.getElementById("lblCurrentVoutCmd");
    const btnOpOn = document.getElementById("btnOpOn"), btnOpOff = document.getElementById("btnOpOff"), btnOpMarginHigh = document.getElementById("btnOpMarginHigh"), btnOpMarginLow = document.getElementById("btnOpMarginLow"), lblOpState = document.getElementById("lblOpState");
    const txtVinOn = document.getElementById("txtVinOn"), txtVinOff = document.getElementById("txtVinOff"), txtTonRise = document.getElementById("txtTonRise"), btnApplyStartup = document.getElementById("btnApplyStartup"), lblPgThresh = document.getElementById("lblPgThresh");

    // Chart Legend Checkboxes & Labels
    const chkVin = document.getElementById("chkVin"), chkVout = document.getElementById("chkVout"), chkIin = document.getElementById("chkIin"), chkIout = document.getElementById("chkIout");
    const chkPin = document.getElementById("chkPin"), chkPout = document.getElementById("chkPout"), chkT1 = document.getElementById("chkT1"), chkT2 = document.getElementById("chkT2");
    const lgVin = document.getElementById("lgVin"), lgVout = document.getElementById("lgVout"), lgIin = document.getElementById("lgIin"), lgIout = document.getElementById("lgIout");
    const lgPin = document.getElementById("lgPin"), lgPout = document.getElementById("lgPout"), lgT1 = document.getElementById("lgT1"), lgT2 = document.getElementById("lgT2");

    // Studio Internal Sub-Tabs
    const tabBtnControls = document.getElementById("tabBtnControls"), tabBtnFaults = document.getElementById("tabBtnFaults"), tabBtnRegs = document.getElementById("tabBtnRegs"), tabBtnLogger = document.getElementById("tabBtnLogger");
    const panelControls = document.getElementById("panelControls"), panelFaults = document.getElementById("panelFaults"), panelRegs = document.getElementById("panelRegs"), panelLogger = document.getElementById("panelLogger");
    const badgeTabFaults = document.getElementById("badgeTabFaults"), lblStatusWord = document.getElementById("lblStatusWord");
    const bannerFaultStatus = document.getElementById("bannerFaultStatus"), txtFaultTitle = document.getElementById("txtFaultTitle"), txtFaultSubtitle = document.getElementById("txtFaultSubtitle");
    const boxStatusStrips = document.getElementById("boxStatusStrips");
    const selRegCat = document.getElementById("selRegCat"), txtRegSearch = document.getElementById("txtRegSearch");
    const toastBox = document.getElementById("toastBox");
    const tooltipBox = document.getElementById("tooltipBox"), ttTitle = document.getElementById("ttTitle"), ttBadge = document.getElementById("ttBadge"), ttReg = document.getElementById("ttReg"), ttDesc = document.getElementById("ttDesc");

    const canvasContainer = document.getElementById("canvasContainer");
    const chartCanvas = document.getElementById("liveTelemetryCanvas");
    const chartCrosshairTooltip = document.getElementById("chartCrosshairTooltip");
    const ctx = chartCanvas ? chartCanvas.getContext("2d") : null;

    let mouseHoverX = null;

    function setConnectionState(state) {
        if (!badgeConnState) return;
        badgeConnState.classList.remove("live", "reconnecting", "offline");
        if (state === "live") {
            badgeConnState.classList.add("live");
            txtConnState.textContent = "LIVE";
        } else if (state === "reconnecting") {
            badgeConnState.classList.add("reconnecting");
            txtConnState.textContent = "RECONNECTING...";
        } else {
            badgeConnState.classList.add("offline");
            txtConnState.textContent = "OFFLINE";
        }
    }

    function initSSE() {
        if (sseSource) {
            try { sseSource.close(); } catch(e){}
            sseSource = null;
        }

        setConnectionState("reconnecting");
        sseSource = new EventSource("/api/stream");

        sseSource.onopen = () => {
            isReconnecting = false;
            lastDataTime = Date.now();
            setConnectionState("live");
        };

        sseSource.onmessage = e => {
            lastDataTime = Date.now();
            setConnectionState("live");
            try {
                const d = JSON.parse(e.data);
                handleTelemetryData(d);
            } catch(err){}
        };

        sseSource.onerror = () => {
            setConnectionState("offline");
            if (!isReconnecting) {
                isReconnecting = true;
                if (reconnectTimeout) clearTimeout(reconnectTimeout);
                reconnectTimeout = setTimeout(initSSE, 1200);
            }
        };
    }

    // Auto-Healing Sleep / Wake / Background Tab Watchdog
    setInterval(() => {
        const timeSinceLastData = Date.now() - lastDataTime;
        if (timeSinceLastData > 2200) {
            setConnectionState("reconnecting");
            initSSE();
        }
    }, 1000);

    // Instant Wake-Up Listeners for Laptop Standby
    document.addEventListener("visibilitychange", () => {
        if (document.visibilityState === "visible") {
            lastDataTime = 0;
            initSSE();
            setTimeout(resizeCanvas, 100);
        }
    });
    window.addEventListener("focus", () => {
        if (Date.now() - lastDataTime > 1500) initSSE();
    });
    window.addEventListener("online", () => initSSE());

    initSSE();

    function showTooltip(e, chip) {
        ttTitle.textContent = `${chip.label} (Bit ${chip.bit})`;
        ttReg.textContent = `Register: ${chip.reg}`;
        ttDesc.textContent = chip.desc;

        if (chip.active) {
            tooltipBox.classList.add("alert");
            ttBadge.style.background = "#7f1d1d";
            ttBadge.style.color = "#fca5a5";
            ttBadge.textContent = "ACTIVE FAULT (1)";
        } else {
            tooltipBox.classList.remove("alert");
            ttBadge.style.background = "#14532d";
            ttBadge.style.color = "#86efac";
            ttBadge.textContent = "NORMAL (0)";
        }

        positionTooltip(e);
        tooltipBox.classList.add("show");
    }

    function positionTooltip(e) {
        const x = e.clientX + 14;
        const y = e.clientY + 14;
        const maxW = window.innerWidth - 360;
        tooltipBox.style.left = (x > maxW ? (e.clientX - 350) : x) + "px";
        tooltipBox.style.top = y + "px";
    }

    function hideTooltip() {
        tooltipBox.classList.remove("show");
    }

    function resizeCanvas() {
        if (!chartCanvas || !canvasContainer) return;
        const rect = canvasContainer.getBoundingClientRect();
        chartCanvas.width = (rect.width || 800) * (window.devicePixelRatio || 1);
        chartCanvas.height = (rect.height || 280) * (window.devicePixelRatio || 1);
        drawLiveChart();
    }
    window.addEventListener("resize", resizeCanvas);
    setTimeout(resizeCanvas, 100);

    if (canvasContainer) {
        canvasContainer.addEventListener("mousemove", (e) => {
            const rect = canvasContainer.getBoundingClientRect();
            mouseHoverX = e.clientX - rect.left;
            drawLiveChart();
        });
        canvasContainer.addEventListener("mouseleave", () => {
            mouseHoverX = null;
            if (chartCrosshairTooltip) chartCrosshairTooltip.style.display = "none";
            drawLiveChart();
        });
    }

    function drawLiveChart() {
        if (!ctx || !chartCanvas) return;
        const dpr = window.devicePixelRatio || 1;
        const w = chartCanvas.width, h = chartCanvas.height;
        ctx.clearRect(0, 0, w, h);

        const padLeft = 46 * dpr;
        const padRight = 54 * dpr;
        const padTop = 18 * dpr;
        const padBottom = 26 * dpr;

        const plotX = padLeft;
        const plotY = padTop;
        const plotW = w - padLeft - padRight;
        const plotH = h - padTop - padBottom;

        if (plotW <= 10 || plotH <= 10) return;

        ctx.fillStyle = "#070c18";
        ctx.fillRect(plotX, plotY, plotW, plotH);

        ctx.lineWidth = 1;
        ctx.font = `${10 * dpr}px monospace`;

        const vMax = 60.0;
        const cMax = 35.0;
        const pMax = 600.0;

        for (let i = 0; i <= 4; i++) {
            const ratio = i / 4.0;
            const yPos = plotY + (plotH * (1.0 - ratio));

            ctx.strokeStyle = (i === 0 || i === 4) ? "#334155" : "#1e293b";
            ctx.beginPath();
            ctx.moveTo(plotX, yPos);
            ctx.lineTo(plotX + plotW, yPos);
            ctx.stroke();

            const voltVal = (vMax * ratio).toFixed(0);
            ctx.fillStyle = "#38bdf8";
            ctx.textAlign = "right";
            ctx.textBaseline = "middle";
            ctx.fillText(`${voltVal}V`, plotX - (6 * dpr), yPos);

            const currVal = (cMax * ratio).toFixed(0);
            const pwrVal = (pMax * ratio).toFixed(0);
            ctx.fillStyle = "#f97316";
            ctx.textAlign = "left";
            ctx.textBaseline = "middle";
            ctx.fillText(`${currVal}A|${pwrVal}W`, plotX + plotW + (6 * dpr), yPos);
        }

        const timeLabels = ["-30s", "-20s", "-10s", "NOW"];
        for (let j = 0; j < 4; j++) {
            const xPos = plotX + (plotW * (j / 3.0));
            ctx.strokeStyle = "#1e293b";
            ctx.beginPath();
            ctx.moveTo(xPos, plotY);
            ctx.lineTo(xPos, plotY + plotH);
            ctx.stroke();

            ctx.fillStyle = "#94a3b8";
            ctx.textAlign = (j === 0) ? "left" : ((j === 3) ? "right" : "center");
            ctx.textBaseline = "top";
            ctx.fillText(timeLabels[j], xPos, plotY + plotH + (6 * dpr));
        }

        ctx.strokeStyle = "#334155";
        ctx.strokeRect(plotX, plotY, plotW, plotH);

        if (histVout.length < 2) return;

        const maxSamples = MAX_CHART_POINTS;
        const stepX = plotW / (maxSamples - 1);
        const startIndex = maxSamples - histVout.length;

        ctx.save();
        ctx.beginPath();
        ctx.rect(plotX, plotY, plotW, plotH);
        ctx.clip();

        function renderTrace(dataArray, color, minVal, maxVal, enabled) {
            if (!enabled || !dataArray.length) return;
            ctx.strokeStyle = color;
            ctx.lineWidth = 2 * dpr;
            ctx.beginPath();
            for (let i = 0; i < dataArray.length; i++) {
                const x = plotX + ((startIndex + i) * stepX);
                const norm = Math.max(0, Math.min(1, (dataArray[i] - minVal) / (maxVal - minVal || 1)));
                const y = plotY + (plotH * (1.0 - norm));
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        renderTrace(histVin, "#06b6d4", 0, 60, chkVin.checked);
        renderTrace(histVout, "#38bdf8", 0, 60, chkVout.checked);
        renderTrace(histIin, "#eab308", 0, 35, chkIin.checked);
        renderTrace(histIout, "#f97316", 0, 35, chkIout.checked);
        renderTrace(histPin, "#818cf8", 0, 600, chkPin.checked);
        renderTrace(histPout, "#c084fc", 0, 600, chkPout.checked);
        renderTrace(histT1, "#ef4444", 0, 110, chkT1.checked);
        renderTrace(histT2, "#f43f5e", 0, 110, chkT2.checked);

        if (mouseHoverX !== null && mouseHoverX >= (plotX / dpr) && mouseHoverX <= ((plotX + plotW) / dpr)) {
            const cursorCanvasX = mouseHoverX * dpr;
            const sampleIdx = Math.round((cursorCanvasX - plotX) / stepX) - startIndex;

            if (sampleIdx >= 0 && sampleIdx < histVout.length) {
                const crossX = plotX + ((startIndex + sampleIdx) * stepX);
                ctx.strokeStyle = "rgba(248, 250, 252, 0.65)";
                ctx.lineWidth = 1 * dpr;
                ctx.setLineDash([4 * dpr, 4 * dpr]);
                ctx.beginPath();
                ctx.moveTo(crossX, plotY);
                ctx.lineTo(crossX, plotY + plotH);
                ctx.stroke();
                ctx.setLineDash([]);

                if (chartCrosshairTooltip) {
                    const tStamp = histTimestamps[sampleIdx] || "";
                    const vIn = (histVin[sampleIdx] || 0).toFixed(2);
                    const vOut = (histVout[sampleIdx] || 0).toFixed(3);
                    const iIn = (histIin[sampleIdx] || 0).toFixed(2);
                    const iOut = (histIout[sampleIdx] || 0).toFixed(2);
                    const pIn = (histPin[sampleIdx] || 0).toFixed(1);
                    const pOut = (histPout[sampleIdx] || 0).toFixed(1);
                    const t1 = (histT1[sampleIdx] || 0).toFixed(1);

                    chartCrosshairTooltip.innerHTML = `
                        <div style="color:var(--cyan);font-weight:bold;margin-bottom:3px;">SAMPLE [${tStamp}]</div>
                        <div style="display:grid;grid-template-columns:1fr 1fr;gap:4px 10px;">
                            <span style="color:#06b6d4;">VIN: ${vIn} V</span>
                            <span style="color:#38bdf8;">VOUT: ${vOut} V</span>
                            <span style="color:#eab308;">IIN: ${iIn} A</span>
                            <span style="color:#f97316;">IOUT: ${iOut} A</span>
                            <span style="color:#818cf8;">PIN: ${pIn} W</span>
                            <span style="color:#c084fc;">POUT: ${pOut} W</span>
                            <span style="color:#ef4444;grid-column:1/-1;">TEMP1: ${t1} C</span>
                        </div>
                    `;
                    chartCrosshairTooltip.style.display = "block";
                    const ttX = (crossX / dpr) + 12;
                    const maxTtX = (w / dpr) - 190;
                    chartCrosshairTooltip.style.left = (ttX > maxTtX ? ((crossX / dpr) - 185) : ttX) + "px";
                    chartCrosshairTooltip.style.top = "24px";
                }
            } else {
                if (chartCrosshairTooltip) chartCrosshairTooltip.style.display = "none";
            }
        } else {
            if (chartCrosshairTooltip) chartCrosshairTooltip.style.display = "none";
        }

        ctx.restore();
    }

    [chkVin, chkVout, chkIin, chkIout, chkPin, chkPout, chkT1, chkT2].forEach(chk => {
        chk.addEventListener("change", () => {
            chk.parentElement.classList.toggle("inactive", !chk.checked);
            drawLiveChart();
        });
    });

    function notify(msg) {
        toastBox.textContent = msg;
        toastBox.classList.add("show");
        setTimeout(() => toastBox.classList.remove("show"), 2800);
    }

    function showView(v) {
        if (v === "hub") {
            viewHub.classList.add("active"); viewStudio.classList.remove("active");
            btnNavHub.classList.add("active"); btnNavStudio.classList.remove("active");
            scan();
        } else {
            viewHub.classList.remove("active"); viewStudio.classList.add("active");
            btnNavHub.classList.remove("active"); btnNavStudio.classList.add("active");
            setTimeout(resizeCanvas, 50);
            loadRegs();
        }
    }

    function showStudioTab(tab) {
        tabBtnControls.classList.remove("active"); tabBtnFaults.classList.remove("active"); tabBtnRegs.classList.remove("active"); tabBtnLogger.classList.remove("active");
        panelControls.classList.remove("active"); panelFaults.classList.remove("active"); panelRegs.classList.remove("active"); panelLogger.classList.remove("active");

        if (tab === "controls") {
            tabBtnControls.classList.add("active"); panelControls.classList.add("active");
        } else if (tab === "faults") {
            tabBtnFaults.classList.add("active"); panelFaults.classList.add("active");
        } else if (tab === "regs") {
            tabBtnRegs.classList.add("active"); panelRegs.classList.add("active");
            loadRegs();
        } else if (tab === "logger") {
            tabBtnLogger.classList.add("active"); panelLogger.classList.add("active");
        }
    }

    tabBtnControls.addEventListener("click", () => showStudioTab("controls"));
    tabBtnFaults.addEventListener("click", () => showStudioTab("faults"));
    tabBtnRegs.addEventListener("click", () => showStudioTab("regs"));
    tabBtnLogger.addEventListener("click", () => showStudioTab("logger"));

    btnNavHub.addEventListener("click", () => showView("hub"));
    btnBack.addEventListener("click", () => showView("hub"));
    btnNavStudio.addEventListener("click", () => showView("studio"));

    async function initBuses() {
        try {
            const r = await fetch("/api/buses");
            const d = await r.json();
            selBus.innerHTML = (d.buses || [2]).map(b => `<option value="${b}">/dev/i2c-${b}</option>`).join("");
            selBus.value = "2";
        } catch(e){}
        scan();
    }
    initBuses();

    async function scan() {
        gridDevices.innerHTML = '<div style="color:var(--muted)">Scanning I2C bus...</div>';
        const b = selBus.value || 2;
        const controller = new AbortController();
        const timer = setTimeout(() => controller.abort(), 1800);

        try {
            const r = await fetch("/api/scan?bus=" + b, { signal: controller.signal });
            clearTimeout(timer);
            const d = await r.json();
            if (!d.devices || d.devices.length === 0) {
                gridDevices.innerHTML = `<div style="color:var(--muted);grid-column:1/-1;padding:24px;background:var(--card);border-radius:8px;border:1px solid #1e293b;text-align:center;">No I2C cards or devices detected on bus ${b}.</div>`;
                return;
            }
            gridDevices.innerHTML = d.devices.map(dev => `
                <div class="card" onclick="connectDevice(${dev.bus}, '${dev.addr_hex}')">
                    <div style="display:flex;align-items:center;gap:12px;margin-bottom:10px;">
                        <div class="icon-box">${dev.icon}</div>
                        <div><strong>${dev.name}</strong><br><small style="color:var(--muted)">Bus #${dev.bus} - ${dev.addr_hex}</small></div>
                    </div>
                    <div style="font-size:12px;color:var(--blue);font-weight:700;">Connect &amp; Control &rarr;</div>
                </div>
            `).join("");
        } catch(e) {
            clearTimeout(timer);
            gridDevices.innerHTML = `<div style="color:var(--muted);grid-column:1/-1;padding:24px;background:var(--card);border-radius:8px;border:1px solid #1e293b;text-align:center;">No response on this bus (scan timeout reached).</div>`;
        }
    }
    btnScan.addEventListener("click", scan);
    selBus.addEventListener("change", scan);

    window.connectDevice = async (b, a) => {
        currentBus = b; currentAddr = a;
        lblTarget.textContent = `TARGET: Bus #${b} - ${a}`;
        await fetch("/api/connect", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({bus: b, board: a}) });
        notify(`Connected to ${a} on Bus ${b}`);
        showView("studio");
    };

    window.quickSetVout = (val) => {
        txtSetVout.value = val.toFixed(2);
        applyVout(val);
    };

    async function applyVout(targetV) {
        const v = parseFloat(targetV || txtSetVout.value);
        if (isNaN(v) || v < 5.0 || v > 15.0) {
            notify("Invalid VOUT setpoint (Must be between 5.0V and 15.0V)");
            return;
        }
        try {
            const r = await fetch("/api/set_vout", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({ vout: v }) });
            const d = await r.json();
            if (d.status === "ok") {
                notify(`VOUT_COMMAND (0x21) set to ${v.toFixed(2)} V`);
                lblCurrentVoutCmd.textContent = `${v.toFixed(2)} V`;
            } else {
                notify("Failed to write VOUT_COMMAND");
            }
        } catch(e) {
            notify("Network error setting VOUT");
        }
    }
    btnApplyVout.addEventListener("click", () => applyVout());

    async function setOperation(opByte, label) {
        if (opByte === 0x00) {
            if (!confirm("Are you sure you want to turn OFF the 12V output power stage via OPERATION (0x00)?")) {
                return;
            }
        }
        try {
            const r = await fetch("/api/set_operation", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({ op: opByte }) });
            const d = await r.json();
            if (d.status === "ok") {
                notify(`OPERATION (0x01) set to ${label}`);
                lblOpState.textContent = `0x${opByte.toString(16).toUpperCase()} (${label})`;
            } else {
                notify("Failed to write OPERATION register");
            }
        } catch(e) {
            notify("Network error setting OPERATION");
        }
    }
    btnOpOn.addEventListener("click", () => setOperation(0x80, "ON"));
    btnOpOff.addEventListener("click", () => setOperation(0x00, "OFF"));
    btnOpMarginHigh.addEventListener("click", () => setOperation(0x98, "Margin High"));
    btnOpMarginLow.addEventListener("click", () => setOperation(0x94, "Margin Low"));

    btnApplyStartup.addEventListener("click", async () => {
        const vinOn = parseFloat(txtVinOn.value);
        const vinOff = parseFloat(txtVinOff.value);
        const tonRise = parseFloat(txtTonRise.value);
        try {
            const r = await fetch("/api/set_startup", { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({ vin_on: vinOn, vin_off: vinOff, ton_rise: tonRise }) });
            const d = await r.json();
            if (d.status === "ok") {
                notify("Startup thresholds (0x35, 0x36, 0x61) written successfully");
            } else {
                notify("Failed to write startup thresholds");
            }
        } catch(e) {
            notify("Network error writing startup thresholds");
        }
    });

    function handleTelemetryData(d) {
        valVin.textContent = (d.vin || 0).toFixed(3) + " V";
        valVout.textContent = (d.vout || 0).toFixed(3) + " V";
        valIin.textContent = (d.iin || 0).toFixed(2) + " A";
        valIout.textContent = (d.iout || 0).toFixed(2) + " A";
        valPin.textContent = (d.pin || 0).toFixed(1) + " W";
        valPout.textContent = (d.pout || 0).toFixed(1) + " W";
        valEff.textContent = (d.eff || 0).toFixed(1) + " %";
        valTemp.textContent = `${(d.temp1 || 0).toFixed(1)} / ${(d.temp2 || 0).toFixed(1)} C`;

        lgVin.textContent = (d.vin || 0).toFixed(1);
        lgVout.textContent = (d.vout || 0).toFixed(2);
        lgIin.textContent = (d.iin || 0).toFixed(1);
        lgIout.textContent = (d.iout || 0).toFixed(1);
        lgPin.textContent = Math.round(d.pin || 0);
        lgPout.textContent = Math.round(d.pout || 0);
        lgT1.textContent = Math.round(d.temp1 || 0);
        lgT2.textContent = Math.round(d.temp2 || 0);

        histVin.push(d.vin || 0);
        histVout.push(d.vout || 0);
        histIin.push(d.iin || 0);
        histIout.push(d.iout || 0);
        histPin.push(d.pin || 0);
        histPout.push(d.pout || 0);
        histT1.push(d.temp1 || 0);
        histT2.push(d.temp2 || 0);
        histTimestamps.push(d.timestamp || "");

        if (histVout.length > MAX_CHART_POINTS) {
            histVin.shift(); histVout.shift(); histIin.shift(); histIout.shift();
            histPin.shift(); histPout.shift(); histT1.shift(); histT2.shift();
            histTimestamps.shift();
        }
        drawLiveChart();

        if (d.power_state) {
            badgePower.className = "badge-status";
            badgePower.textContent = "STATUS: ENABLED (ON)";
        } else {
            badgePower.className = "badge-status off";
            badgePower.textContent = "STATUS: DISABLED (OFF)";
        }

        if (d.startup) {
            lblCurrentVoutCmd.textContent = (d.startup.vout_cmd || 12).toFixed(2) + " V";
            lblPgThresh.textContent = `ON: ${(d.startup.pg_on || 10.8).toFixed(1)}V / OFF: ${(d.startup.pg_off || 10.2).toFixed(1)}V`;
        }

        if (d.logger) {
            lblLog.textContent = `${d.logger.count} points recorded in memory`;
            btnLogger.textContent = d.logger.is_logging ? "Stop Recording" : "Start Recording";
            btnLogger.className = d.logger.is_logging ? "btn btn-red" : "btn";
        }

        if (d.status_word) {
            lblStatusWord.textContent = `STATUS_WORD: ${d.status_word}`;
        }
        const activeTotal = d.active_faults_total || 0;
        const crits = d.critical_faults || 0;

        if (activeTotal > 0) {
            badgeFaultLive.className = "badge-fault-indicator alert";
            badgeFaultLive.textContent = `! ${activeTotal} FAULT${activeTotal > 1 ? 'S' : ''}`;
            badgeTabFaults.textContent = `! ${activeTotal} FAULT${activeTotal > 1 ? 'S' : ''}`;
            bannerFaultStatus.style.borderColor = "#dc2626";
            bannerFaultStatus.style.background = "#7f1d1d";
            txtFaultTitle.style.color = "#fca5a5";
            txtFaultTitle.textContent = `ACTIVE FAULTS DETECTED (${activeTotal} flags active, ${crits} critical)`;
            txtFaultSubtitle.style.color = "#fecaca";
            txtFaultSubtitle.textContent = "Hover mouse over any red glowing flag below to see why it triggered.";
        } else {
            badgeFaultLive.className = "badge-fault-indicator";
            badgeFaultLive.textContent = "0 FAULTS (OK)";
            badgeTabFaults.textContent = "0 FAULT";
            bannerFaultStatus.style.borderColor = "#16a34a";
            bannerFaultStatus.style.background = "#14532d";
            txtFaultTitle.style.color = "#86efac";
            txtFaultTitle.textContent = "SYSTEM NOMINAL - NO FAULTS";
            txtFaultSubtitle.style.color = "#bbf7d0";
            txtFaultSubtitle.textContent = "Hover mouse over any status bit chip below to see its exact hardware description.";
        }

        if (d.status_strips && boxStatusStrips) {
            boxStatusStrips.innerHTML = d.status_strips.map((s, sIdx) => `
                <div class="status-strip-box">
                    <div class="status-strip-header">
                        <div>
                            <strong style="font-family:monospace;font-size:13px;color:#f8fafc;">${s.title}</strong>
                        </div>
                        <strong style="font-family:monospace;font-size:12px;color:var(--cyan);">${s.val_hex}</strong>
                    </div>
                    <div class="status-chips-wrap">
                        ${s.chips.map((c, cIdx) => `
                            <span class="status-chip ${c.active ? 'active-red' : ''}" data-sidx="${sIdx}" data-cidx="${cIdx}">
                                ${c.label}
                            </span>
                        `).join("")}
                    </div>
                </div>
            `).join("");

            boxStatusStrips.querySelectorAll(".status-chip").forEach(el => {
                const sIdx = parseInt(el.getAttribute("data-sidx"));
                const cIdx = parseInt(el.getAttribute("data-cidx"));
                const chipData = d.status_strips[sIdx].chips[cIdx];

                el.addEventListener("mouseenter", (e) => showTooltip(e, chipData));
                el.addEventListener("mousemove", (e) => positionTooltip(e));
                el.addEventListener("mouseleave", hideTooltip);
            });
        }
    }

    async function sendClearFaults() {
        try {
            const r = await fetch("/api/clear", { method: "POST" });
            const d = await r.json();
            if (d.status === "ok") {
                notify("CLEAR_FAULTS (0x03) Send Byte command executed successfully");
            } else {
                notify("CLEAR_FAULTS command sent (hardware may be busy)");
            }
            setTimeout(loadRegs, 200);
        } catch(e) {
            notify("Network error executing CLEAR_FAULTS");
        }
    }
    btnClearFaults.addEventListener("click", sendClearFaults);
    btnClearFaultsPanel.addEventListener("click", sendClearFaults);

    btnRefreshRegs.addEventListener("click", () => {
        loadRegs();
        notify("PMBus register table refreshed");
    });

    btnLogger.addEventListener("click", async () => {
        const isStop = btnLogger.textContent.includes("Stop");
        await fetch(isStop ? "/api/logger/stop" : "/api/logger/start", { method: "POST" });
        notify(isStop ? "Logging stopped" : "Logging started");
    });

    btnDownloadCsv.addEventListener("click", () => {
        notify("Downloading CSV file...");
        window.location.href = "/api/logger/download";
    });

    function renderRegisterTable() {
        const cat = selRegCat.value;
        const q = txtRegSearch.value.trim().toLowerCase();

        const filtered = allRegisters.filter(r => {
            const matchCat = (cat === "ALL" || r.cat === cat);
            const matchText = (!q || r.name.toLowerCase().includes(q) || r.hex.toLowerCase().includes(q) || r.desc.toLowerCase().includes(q) || r.val.toLowerCase().includes(q));
            return matchCat && matchText;
        });

        if (filtered.length === 0) {
            tblRegs.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--muted);padding:20px;">No PMBus registers match your search query.</td></tr>';
            return;
        }

        tblRegs.innerHTML = filtered.map(reg => `
            <tr>
                <td style="font-weight:700;color:#f8fafc;">${reg.name}</td>
                <td style="color:var(--cyan);font-family:monospace;">${reg.hex}</td>
                <td><small style="background:#1e293b;padding:2px 6px;border-radius:4px;color:#cbd5e1;">${reg.cat}</small></td>
                <td style="color:var(--muted);font-size:11px;">${reg.desc}</td>
                <td style="color:${reg.val.startsWith('0x') || reg.val === 'n/a' ? '#94a3b8' : 'var(--green)'};font-weight:700;font-family:monospace;">${reg.val}</td>
            </tr>
        `).join("");
    }

    selRegCat.addEventListener("change", renderRegisterTable);
    txtRegSearch.addEventListener("input", renderRegisterTable);

    async function loadRegs() {
        try {
            const r = await fetch("/api/registers");
            const d = await r.json();
            allRegisters = d.registers || [];
            renderRegisterTable();
        } catch(e){}
    }
});
</script>
</body>
</html>
"""

class Handler(SimpleHTTPRequestHandler):
    mgr: HardwareManager = None
    def log_message(self, f, *a): pass

    def send_j(self, data, s=200):
        c = json.dumps(data).encode("utf-8")
        self.send_response(s)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(c)))
        self.end_headers()
        self.wfile.write(c)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(p.query)

        if p.path == "/api/buses":
            self.send_j({"buses": list_user_i2c_buses()})
        elif p.path == "/api/scan":
            b = int(q.get("bus", [self.mgr.bus_id])[0])
            self.send_j({"devices": self.mgr.scan_bus_safe(b)})
        elif p.path == "/api/telemetry":
            self.send_j(self.mgr.get_telemetry())
        elif p.path == "/api/registers":
            self.send_j({"registers": self.mgr.get_registers()})
        elif p.path == "/api/logger/download":
            csv_data = self.mgr.logger.get_csv().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=\"powerlab_telemetry_log.csv\"")
            self.send_header("Content-Length", str(len(csv_data)))
            self.end_headers()
            self.wfile.write(csv_data)
        elif p.path == "/api/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            try:
                if hasattr(self, 'connection') and self.connection:
                    self.connection.settimeout(5.0)
                while True:
                    t = self.mgr.get_telemetry()
                    payload = f"data: {json.dumps(t)}\n\n".encode("utf-8")
                    self.wfile.write(payload)
                    self.wfile.flush()
                    time.sleep(0.35)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, socket.error, socket.timeout):
                pass
            except Exception:
                pass
        else:
            c = HTML_APP.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(c)))
            self.end_headers()
            self.wfile.write(c)

    def do_POST(self):
        p = urllib.parse.urlparse(self.path).path
        l = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(l).decode("utf-8")) if l > 0 else {}

        if p == "/api/connect":
            b = int(body.get("bus", 2))
            a_str = str(body.get("board", "0x04"))
            a = int(a_str, 16) if a_str.startswith("0x") else int(a_str)
            self.mgr.connect(b, a)
            self.send_j({"status": "ok"})
        elif p == "/api/clear":
            ok = self.mgr.clear_faults()
            self.send_j({"status": "ok" if ok else "error"})
        elif p == "/api/set_vout":
            target_v = float(body.get("vout", 12.0))
            ok = self.mgr.set_vout(target_v)
            self.send_j({"status": "ok" if ok else "error"})
        elif p == "/api/set_operation":
            op_byte = int(body.get("op", 0x80))
            ok = self.mgr.set_operation(op_byte)
            self.send_j({"status": "ok" if ok else "error"})
        elif p == "/api/set_startup":
            vin_on = float(body["vin_on"]) if "vin_on" in body else None
            vin_off = float(body["vin_off"]) if "vin_off" in body else None
            ton_rise = float(body["ton_rise"]) if "ton_rise" in body else None
            ok = self.mgr.set_startup_thresholds(vin_on=vin_on, vin_off=vin_off, ton_rise=ton_rise)
            self.send_j({"status": "ok" if ok else "error"})
        elif p == "/api/logger/start":
            self.mgr.logger.start()
            self.send_j({"status": "ok"})
        elif p == "/api/logger/stop":
            self.mgr.logger.stop()
            self.send_j({"status": "ok"})
        else:
            self.send_j({"status": "error"}, 404)

class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bus", type=int, default=2)
    parser.add_argument("--board", type=lambda x: int(x, 16) if str(x).startswith("0x") else int(x), default=0x04)
    args = parser.parse_args()

    mgr = HardwareManager(bus_id=args.bus, addr=args.board)
    Handler.mgr = mgr

    server = Server(("0.0.0.0", args.port), Handler)
    print("=" * 60)
    print("  POWER LAB STUDIO PRO - READY")
    print(f"  Web Server on http://localhost:{args.port}")
    print("=" * 60)
    try: server.serve_forever()
    except KeyboardInterrupt: server.server_close()

if __name__ == "__main__":
    main()
