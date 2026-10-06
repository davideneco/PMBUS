#!/usr/bin/env python3
"""
PMBus Hardware Driver & Hardware Abstraction Layer
Supporte à la fois le matériel réel (via smbus2) et un mode Simulation / Mock avancé.
"""

import os
import sys
import time
import math
import random
from datetime import datetime

# ==============================================================================
# CONSTANTES PMBUS ET MATÉRIELLES PMBUS
# ==============================================================================
PMBUS_NUMBER_OF_PAGES = 10
PMBUS_NUMBER_OF_MODULES = 3
PMBUS_UNLOCK_CODE = [0xAA, 0x55, 0xA5, 0x5A]

LTC4286_SENSE_FULL_SCALE              = 0.032
LTC4286_SOURCE_FULL_SCALE             = 102.4
TWO_POWER_FIFTEEN                     = 2.0 ** 15.0
LTC4286_I_CONSTANT_FOR_ADC_CONVERSION = 0.032
LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS  = -273
R_SHUNT_HS                            = 0.0003

# Registres PMBus Standard
PAGE                                = 0x00
OPERATION                           = 0x01
ON_OFF_CONFIG                       = 0x02
CLEAR_FAULTS                        = 0x03
WRITE_PROTECT                       = 0x10
RESTORE_DEFAULT_ALL                 = 0x12
STORE_USER_ALL                      = 0x15
RESTORE_USER_ALL                    = 0x16
CAPABILITY                          = 0x19
QUERY                               = 0x1A

VOUT_MODE                           = 0x20
VOUT_COMMAND                        = 0x21
VOUT_MAX                            = 0x24
VOUT_MARGIN_HIGH                    = 0x25
VOUT_MARGIN_LOW                     = 0x26
VOUT_TRANSITION_RATE                = 0x27
VOUT_DROOP                          = 0x28

VOUT_OV_FAULT_LIMIT                 = 0x40
VOUT_OV_FAULT_RESPONSE              = 0x41
VOUT_OV_WARN_LIMIT                  = 0x42
VOUT_UV_WARN_LIMIT                  = 0x43
VOUT_UV_FAULT_LIMIT                 = 0x44
VOUT_UV_FAULT_RESPONSE              = 0x45
IOUT_OC_FAULT_LIMIT                 = 0x46
IOUT_OC_FAULT_RESPONSE              = 0x47
IOUT_OC_LV_FAULT_LIMIT              = 0x48
IOUT_OC_LV_FAULT_RESPONSE           = 0x49
IOUT_OC_WARN_LIMIT                  = 0x4A
IOUT_UC_FAULT_LIMIT                 = 0x4B
IOUT_UC_FAULT_RESPONSE              = 0x4C

OT_FAULT_LIMIT                      = 0x4F
OT_FAULT_RESPONSE                   = 0x50
OT_WARN_LIMIT                       = 0x51
UT_WARN_LIMIT                       = 0x52
UT_FAULT_LIMIT                      = 0x53
UT_FAULT_RESPONSE                   = 0x54
VIN_OV_FAULT_LIMIT                  = 0x55
VIN_OV_FAULT_RESPONSE               = 0x56
VIN_OV_WARN_LIMIT                   = 0x57
VIN_UV_WARN_LIMIT                   = 0x58
VIN_UV_FAULT_LIMIT                  = 0x59
VIN_UV_FAULT_RESPONSE               = 0x5A
IIN_OC_FAULT_LIMIT                  = 0x5B
IIN_OC_FAULT_RESPONSE               = 0x5C
IIN_OC_WARN_LIMIT                   = 0x5D
POWER_GOOD_ON                       = 0x5E
POWER_GOOD_OFF                      = 0x5F
TON_DELAY                           = 0x60
TON_RISE                            = 0x61
TON_MAX_FAULT_LIMIT                 = 0x62
TON_MAX_FAULT_RESPONSE              = 0x63
TOFF_DELAY                          = 0x64
TOFF_FALL                           = 0x65
TOFF_MAX_WARN_LIMIT                 = 0x66

POUT_OP_FAULT_LIMIT                 = 0x68
POUT_OP_FAULT_RESPONSE              = 0x69
POUT_OP_WARN_LIMIT                  = 0x6A
PIN_OP_WARN_LIMIT                   = 0x6B

STATUS_BYTE                         = 0x78
STATUS_WORD                         = 0x79
STATUS_VOUT                         = 0x7A
STATUS_IOUT                         = 0x7B
STATUS_INPUT                        = 0x7C
STATUS_TEMPERATURE                  = 0x7D
STATUS_CML                          = 0x7E
STATUS_OTHER                        = 0x7F
STATUS_MFR_SPECIFIC                 = 0x80

READ_VIN                            = 0x88
READ_IIN                            = 0x89
READ_VOUT                           = 0x8B
READ_IOUT                           = 0x8C
READ_TEMPERATURE_1                  = 0x8D
READ_TEMPERATURE_2                  = 0x8E
READ_TEMPERATURE_3                  = 0x8F
READ_POUT                           = 0x96
READ_PIN                            = 0x97

PMBUS_REVISION                      = 0x98
MFR_ID                              = 0x99
MFR_MODEL                           = 0x9A
MFR_REVISION                        = 0x9B
MFR_LOCATION                        = 0x9C
MFR_DATE                            = 0x9D
MFR_SERIAL                          = 0x9E

MFR_VIN_MIN                         = 0xA0
MFR_VIN_MAX                         = 0xA1
MFR_IIN_MAX                         = 0xA2
MFR_PIN_MAX                         = 0xA3
MFR_VOUT_MIN                        = 0xA4
MFR_VOUT_MAX                        = 0xA5
MFR_IOUT_MAX                        = 0xA6
MFR_POUT_MAX                        = 0xA7
MFR_TAMBIENT_MAX                    = 0xA8
MFR_TAMBIENT_MIN                    = 0xA9
MFR_EFFICIENCY_LL                   = 0xAA
MFR_EFFICIENCY_HL                   = 0xAB

MFR_MAX_TEMP_1                      = 0xC0
MFR_MAX_TEMP_2                      = 0xC1
MFR_MAX_TEMP_3                      = 0xC2

MFR_RESPONSE_UNIT_CFG               = 0xD2
MFR_PSU_REBOOT                      = 0xD3
MFR_HW_COMPATIBILITY                = 0xD4
MFR_FWUPLOAD_CAPABILITY             = 0xD5
MFR_FWUPLOAD_MODE                   = 0xD6
MFR_FWUPLOAD                        = 0xD7
MFR_FWUPLOAD_STATUS                 = 0xD8
MFR_FWVERSION_NUMBER                = 0xD9
MFR_PSU_FW_CRC16_READ               = 0xDA
MFR_PSU_POWER_CYCLE                 = 0xDB
MFR_BRICK_CONTROL                   = 0xDC
MFR_VIN_PG_THRESH                   = 0xDD
MFR_PMBUS_ISHARE_THRESHOLD          = 0xDE
MFR_POWER_ENABLED                   = 0xE0
MFR_POWER_GOOD                      = 0xE1
MFR_BRICK_FAILURE                   = 0xE2
MFR_FAULTS_ORING                    = 0xE3
MFR_PARAMS_DIFF                     = 0xE4
MFR_BUS_STATUS                      = 0xE5
MFR_PMBUS_ALERT                     = 0xE6
MFR_BMC_SIGNALS                     = 0xE7
MFR_PMBUS_ADDR                      = 0xE8
MFR_CAL_DATA                        = 0xE9
MFR_AUTO_ON                         = 0xEA
MFR_ENABLE_DIAG                     = 0xEB
MFR_DEBUG_STATUS                    = 0xEC
MFR_SYSTEM_STATUS1                  = 0xF0
MFR_SYSTEM_STATUS2                  = 0xF1
MFR_CONFIG1                         = 0xF2
MFR_ADDED_DROOP_DURING_RAMP         = 0xFC

# Listes d'options
items_PAGE = ["00h (All)", "01h (Module 1)", "02h (Module 2)", "03h (Module 3)", "04h (Reserved)", "05h (HS 48V Aux)", "06h (HS 1)", "07h (HS 2)", "08h (HS 3)"]
items_OPERATION = ["Immediate OFF", "Soft OFF", "ON (nominal)", "ON Margin LOW Ign", "ON Margin LOW Flts", "ON Margin HIGH Ign", "ON Margin HIGH Flts"]
items_ON_OFF_CONFIG = ["Ignore all", "PMBus", "Remote Control", "PMBus & RC"]
items_WRITE_PROTECT = ["DISABLE Other", "En Operation/Page", "En OP/PG/OFC/V_CMD", "Enable ALL"]
items_RESPONSE = ["Ignore fault", "Continue 2_Retry 4", "Continue 3_Retry 64", "Shutdown NoRetry", "Shutdown 3_Retry 64", "Shutdown Endless", "Shutdown Endless 16", "Disable NoRetry", "Disable 3_Retry 64"]
items_Current_RESPONSE = ["Ignore flt CstCr", "CdCstCr 2_Retry 4", "CdCstCr 3_Retry 64", "DlCstCr NoRetry", "DlCstCr 3_Retry 64", "DlCstCr Endless", "DlCstCr Endless 16", "Shutdown NoRetry", "Shutdown 3_Retry 64"]
items_POWER_ENABLED = ["All bricks OFF", "All bricks ON", "Only brick 1 ON", "Only brick 2 ON", "Only brick 3 ON", "Bricks 1 & 3 ON", "Bricks 1 & 2 ON", "Bricks 2 & 3 ON"]
items_BRICK_CONTROL = ["1 + 1", "2 + 1"]
items_AUTO_ON = ["Auto ON activated", "NO auto on"]
items_NO_YES = ["No", "Yes"]
hex_RESPONSE = [0x00, 0x52, 0x5E, 0x80, 0x9E, 0xB8, 0xBC, 0xC0, 0xDE]

# ==============================================================================
# FONCTIONS DE CONVERSION MATHÉMATIQUE PMBUS
# ==============================================================================
def twos_complement(value: int, width: int) -> int:
    """Calcul du complément à deux pour une largeur de bits donnée."""
    if value & (1 << (width - 1)):
        return value - (1 << width)
    return value

def linear11_decode(raw: int) -> float:
    """Décodage du format PMBus Linear11 (5 bits exposant, 11 bits mantisse signés)."""
    mantissa = raw & 0x7FF
    exponent = (raw >> 11) & 0x1F
    m = twos_complement(mantissa, 11)
    e = twos_complement(exponent, 5)
    return m * (2.0 ** e)

def linear11_encode(value: float) -> int:
    """Encodage d'un flottant vers le format PMBus Linear11."""
    if value == 0:
        return 0
    e = -16
    while e < 16:
        m = int(round(value / (2.0 ** e)))
        if -1024 <= m <= 1023:
            raw_e = e & 0x1F
            raw_m = m & 0x7FF
            return (raw_e << 11) | raw_m
        e += 1
    return 0

def vout_decode(raw: int, mode: int = 0x14) -> float:
    """Décodage de la tension selon le mode VOUT_MODE (généralement Linear avec exp signé)."""
    exp = twos_complement(mode & 0x1F, 5)
    return raw * (2.0 ** exp)

def vout_encode(val: float, mode: int = 0x14) -> int:
    """Encodage de la consigne de tension selon le mode VOUT_MODE."""
    exp = twos_complement(mode & 0x1F, 5)
    raw = int(round(val / (2.0 ** exp)))
    return max(0, min(0xFFFF, raw))


def detect_i2c_buses() -> list:
    """Détecte tous les bus I2C disponibles sur Linux (/dev/i2c-*) ou par test SMBus."""
    buses = []
    if os.path.exists('/dev'):
        for f in os.listdir('/dev'):
            if f.startswith('i2c-'):
                try:
                    num = int(f.split('-')[1])
                    if num not in buses:
                        buses.append(num)
                except Exception:
                    pass
    try:
        from smbus2 import SMBus
        for b in range(6):
            if b not in buses:
                try:
                    s = SMBus(b)
                    s.close()
                    buses.append(b)
                except Exception:
                    pass
    except Exception:
        pass
    if not buses:
        buses = [0, 1, 2]
    return sorted(buses)

def scan_i2c_addresses(bus_id: int) -> list:
    """Scanne toutes les adresses 7-bit (0x03 à 0x77) sur le bus spécifié."""
    found = []
    try:
        from smbus2 import SMBus
        bus = SMBus(bus_id)
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                found.append(f"0x{addr:02X}")
            except Exception:
                pass
        bus.close()
    except Exception:
        if bus_id == 2: found = ["0x1A", "0x59", "0x5A"]
        elif bus_id == 1: found = ["0x50", "0x51"]
        elif bus_id == 0: found = ["0x24", "0x50"]
    return found

class PMBusDriver:
    """
    Pilote de communication avec la carte d'alimentation PMBus.
    Gère le bus I2C réel (smbus2) ou fournit un simulateur temps réel complet (Mock).
    """

    def __init__(self, bus_id: int = 2, board_addr: int = 0x59, sensor_addr: int = 0x1A, force_mock: bool = False):
        self.bus_id = bus_id
        self.board_addr = board_addr
        self.sensor_addr = sensor_addr
        self.force_mock = force_mock
        self.is_mock = False
        self.bus = None
        self.active_page = 1  # Page 1 par défaut (Module 1)
        self.logs = []
        
        # État simulé pour le mode Mock
        self._sim_state = {
            "power_on": True,
            "vout_target": 12.0,
            "vout_max": 14.0,
            "vout_margin_high": 12.5,
            "vout_margin_low": 11.5,
            "vin_ov_limit": 59.0,
            "vin_uv_limit": 36.0,
            "iout_oc_limit": 200.0,
            "ot_limit": 90.0,
            "ton_delay": 10.0,
            "toff_delay": 10.0,
            "operation": 2, # ON nominal
            "on_off_config": 3,
            "write_protect": 0,
            "auto_on": 0,
            "diag_mode": 0,
            "brick_ctrl": 1, # 2+1
            "brick_power": 1, # All bricks ON
            "status_word": 0x0000,
            "status_vout": 0x00,
            "status_iout": 0x00,
            "status_input": 0x00,
            "status_temp": 0x00,
            "status_cml": 0x00,
            "reboot_timer": 0,
            "noise_seed": random.random() * 100
        }

        self._init_hardware()

    def _init_hardware(self):
        """Tente d'initialiser smbus2 sur le bus réel, sinon bascule proprement en simulation."""
        if self.force_mock:
            self.is_mock = True
            self._log_event("SYS", f"Mode simulation forcé (Mock activé pour l'adresse 0x{self.board_addr:02X})")
            return

        try:
            from smbus2 import SMBus
            self.bus = SMBus(self.bus_id)
            # Test d'accès léger
            self.bus.read_byte(self.board_addr)
            self.is_mock = False
            self._log_event("I2C", f"Connecté au bus I2C #{self.bus_id} (Carte 0x{self.board_addr:02X})")
        except Exception as e:
            self.is_mock = True
            self.bus = None
            self._log_event("SYS", f"Matériel non détecté ({e}). Bascule automatique en mode Simulation (Mock).")

    def _log_event(self, category: str, message: str):
        entry = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "category": category,
            "message": message
        }
        self.logs.append(entry)
        if len(self.logs) > 100:
            self.logs.pop(0)

    # --------------------------------------------------------------------------
    # ACCÈS BAS NIVEAU I2C / PMBUS
    # --------------------------------------------------------------------------
    def read_byte(self, reg: int) -> int:
        if self.is_mock:
            return self._mock_read_byte(reg)
        return self.bus.read_byte_data(self.board_addr, reg)

    def write_byte(self, reg: int, val: int):
        if self.is_mock:
            self._mock_write_byte(reg, val)
            return
        self.bus.write_byte_data(self.board_addr, reg, val)

    def read_word(self, reg: int) -> int:
        if self.is_mock:
            return self._mock_read_word(reg)
        return self.bus.read_word_data(self.board_addr, reg)

    def write_word(self, reg: int, val: int):
        if self.is_mock:
            self._mock_write_word(reg, val)
            return
        self.bus.write_word_data(self.board_addr, reg, val)

    def read_block(self, reg: int, length: int = 16) -> list:
        if self.is_mock:
            return [0] * length
        return self.bus.read_i2c_block_data(self.board_addr, reg, length)

    def write_block(self, reg: int, data: list):
        if self.is_mock:
            return
        self.bus.write_i2c_block_data(self.board_addr, reg, data)

    # --------------------------------------------------------------------------
    # MOCK ENGINE (SIMULATEUR DE COMPORTEMENT PHYSIQUE)
    # --------------------------------------------------------------------------
    def _mock_read_byte(self, reg: int) -> int:
        if reg == PAGE: return self.active_page
        if reg == OPERATION: return self._sim_state["operation"]
        if reg == ON_OFF_CONFIG: return self._sim_state["on_off_config"]
        if reg == WRITE_PROTECT: return self._sim_state["write_protect"]
        if reg == STATUS_BYTE: return self._sim_state["status_word"] & 0xFF
        if reg == STATUS_VOUT: return self._sim_state["status_vout"]
        if reg == STATUS_IOUT: return self._sim_state["status_iout"]
        if reg == STATUS_INPUT: return self._sim_state["status_input"]
        if reg == STATUS_TEMPERATURE: return self._sim_state["status_temp"]
        if reg == STATUS_CML: return self._sim_state["status_cml"]
        if reg == MFR_BRICK_CONTROL: return self._sim_state["brick_ctrl"]
        if reg == MFR_POWER_ENABLED: return self._sim_state["brick_power"]
        if reg == MFR_AUTO_ON: return self._sim_state["auto_on"]
        if reg == MFR_ENABLE_DIAG: return self._sim_state["diag_mode"]
        if reg == VOUT_MODE: return 0x14 # Linear mode (-12)
        return 0x00

    def _mock_write_byte(self, reg: int, val: int):
        if reg == PAGE: self.active_page = val
        elif reg == OPERATION: self._sim_state["operation"] = val
        elif reg == ON_OFF_CONFIG: self._sim_state["on_off_config"] = val
        elif reg == WRITE_PROTECT: self._sim_state["write_protect"] = val
        elif reg == MFR_BRICK_CONTROL: self._sim_state["brick_ctrl"] = val
        elif reg == MFR_POWER_ENABLED: self._sim_state["brick_power"] = val
        elif reg == MFR_AUTO_ON: self._sim_state["auto_on"] = val
        elif reg == MFR_ENABLE_DIAG: self._sim_state["diag_mode"] = val

    def _mock_read_word(self, reg: int) -> int:
        t = time.time() + self._sim_state["noise_seed"]
        noise = math.sin(t * 1.5) * 0.05 + (random.random() - 0.5) * 0.02

        if reg == STATUS_WORD:
            return self._sim_state["status_word"]
        if reg == VOUT_COMMAND:
            return vout_encode(self._sim_state["vout_target"])
        if reg == VOUT_MAX:
            return vout_encode(self._sim_state["vout_max"])
        if reg == VOUT_MARGIN_HIGH:
            return vout_encode(self._sim_state["vout_margin_high"])
        if reg == VOUT_MARGIN_LOW:
            return vout_encode(self._sim_state["vout_margin_low"])

        # Mesures télémétriques simulées
        if reg == READ_VIN:
            vin = 48.0 + math.sin(t * 0.2) * 1.2 + noise
            return linear11_encode(vin)
        if reg == READ_VOUT:
            if not self._sim_state["power_on"] or self._sim_state["operation"] in [0, 1]:
                vout = 0.05 + random.random() * 0.02
            else:
                vout = self._sim_state["vout_target"] + noise * 0.5
            return vout_encode(vout)
        if reg == READ_IOUT:
            if not self._sim_state["power_on"] or self._sim_state["operation"] in [0, 1]:
                iout = 0.0
            else:
                # Simule un courant de charge variable entre 40A et 140A
                iout = 85.0 + math.sin(t * 0.4) * 35.0 + (random.random() - 0.5) * 2.0
            return linear11_encode(iout)
        if reg == READ_IIN:
            if not self._sim_state["power_on"] or self._sim_state["operation"] in [0, 1]:
                iin = 0.15
            else:
                # Calcul basé sur la puissance de sortie et ~95% de rendement
                vout = self._sim_state["vout_target"]
                iout = 85.0 + math.sin(t * 0.4) * 35.0
                pin = (vout * iout) / 0.955 + 15.0
                iin = pin / 48.0
            return linear11_encode(iin)
        if reg == READ_TEMPERATURE_1:
            t1 = 44.5 + math.sin(t * 0.1) * 3.0 + random.random() * 0.5
            return linear11_encode(t1)
        if reg == READ_TEMPERATURE_2:
            t2 = 48.2 + math.sin(t * 0.08) * 4.0 + random.random() * 0.5
            return linear11_encode(t2)
        if reg == READ_TEMPERATURE_3:
            t3 = 41.0 + math.sin(t * 0.05) * 2.0 + random.random() * 0.5
            return linear11_encode(t3)
        if reg == READ_POUT:
            vout = self._sim_state["vout_target"]
            iout = 85.0 + math.sin(t * 0.4) * 35.0
            pout = vout * iout
            return linear11_encode(pout)
        if reg == READ_PIN:
            vout = self._sim_state["vout_target"]
            iout = 85.0 + math.sin(t * 0.4) * 35.0
            pin = (vout * iout) / 0.955 + 15.0
            return linear11_encode(pin)

        return 0x0000

    def _mock_write_word(self, reg: int, val: int):
        if reg == VOUT_COMMAND:
            self._sim_state["vout_target"] = vout_decode(val)
        elif reg == VOUT_MAX:
            self._sim_state["vout_max"] = vout_decode(val)
        elif reg == VOUT_MARGIN_HIGH:
            self._sim_state["vout_margin_high"] = vout_decode(val)
        elif reg == VOUT_MARGIN_LOW:
            self._sim_state["vout_margin_low"] = vout_decode(val)

    # --------------------------------------------------------------------------
    # API HAUT NIVEAU POUR L'APPLICATION WEB
    # --------------------------------------------------------------------------
    def set_page(self, page_num: int):
        """Bascule la page PMBus active."""
        try:
            self.write_byte(PAGE, page_num)
            self.active_page = page_num
            self._log_event("CMD", f"Bascule vers la PAGE {page_num} ({items_PAGE[min(page_num, len(items_PAGE)-1)]})")
            return {"status": "ok", "page": page_num}
        except Exception as e:
            self._log_event("ERR", f"Échec de changement de page: {e}")
            return {"status": "error", "error": str(e)}

    def clear_faults(self):
        """Réinitialise tous les drapeaux d'alarme/défaut PMBus."""
        try:
            if not self.is_mock:
                self.bus.write_byte(self.board_addr, CLEAR_FAULTS)
            else:
                self._sim_state["status_word"] = 0
                self._sim_state["status_vout"] = 0
                self._sim_state["status_iout"] = 0
                self._sim_state["status_input"] = 0
                self._sim_state["status_temp"] = 0
                self._sim_state["status_cml"] = 0
            self._log_event("CMD", "CLEAR_FAULTS envoyé (Alarmes réinitialisées)")
            return {"status": "ok", "message": "Faults cleared"}
        except Exception as e:
            self._log_event("ERR", f"Erreur CLEAR_FAULTS: {e}")
            return {"status": "error", "error": str(e)}

    def toggle_power(self, state: bool = None):
        """Active ou coupe la sortie de puissance de l'alimentation."""
        try:
            current_op = self.read_byte(OPERATION)
            if state is None:
                # Toggle
                new_op = 0x00 if current_op in [0x80, 0x88, 0x98, 0xA8] else 0x80
            else:
                new_op = 0x80 if state else 0x00

            self.write_byte(OPERATION, new_op)
            if self.is_mock:
                self._sim_state["power_on"] = (new_op == 0x80)
                self._sim_state["operation"] = 2 if new_op == 0x80 else 0

            status_str = "ACTIVÉ (ON)" if new_op == 0x80 else "COUPE (OFF)"
            self._log_event("CMD", f"Sortie puissance: {status_str}")
            return {"status": "ok", "power": (new_op == 0x80)}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def reboot_psu(self, delay_s: int = 5):
        """Envoie la séquence sécurisée de déverrouillage et de redémarrage de la PSU."""
        try:
            if not self.is_mock:
                # PMBUS_UNLOCK_CODE = [0xAA, 0x55, 0xA5, 0x5A]
                data = PMBUS_UNLOCK_CODE + [delay_s]
                self.write_block(MFR_PSU_REBOOT, data)
            else:
                self._sim_state["reboot_timer"] = delay_s
            self._log_event("PWR", f"Commande MFR_PSU_REBOOT envoyée (Délai: {delay_s}s)")
            return {"status": "ok", "message": f"PSU Reboot scheduled in {delay_s}s"}
        except Exception as e:
            self._log_event("ERR", f"Erreur Reboot PSU: {e}")
            return {"status": "error", "error": str(e)}

    def power_cycle(self, delay_s: int = 10):
        """Envoie la commande de cycle d'alimentation (Power Cycle)."""
        try:
            self.write_byte(MFR_PSU_POWER_CYCLE, delay_s)
            self._log_event("PWR", f"Commande MFR_PSU_POWER_CYCLE envoyée ({delay_s}s)")
            return {"status": "ok", "message": f"Power cycle scheduled ({delay_s}s)"}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def write_pmbus_param(self, name: str, value: float):
        """Écrit une valeur dans un registre PMBus selon son nom logique."""
        try:
            if name == "VOUT_COMMAND":
                raw = vout_encode(float(value))
                self.write_word(VOUT_COMMAND, raw)
                self._log_event("SET", f"VOUT_COMMAND réglé à {value} V")
            elif name == "VOUT_MAX":
                raw = vout_encode(float(value))
                self.write_word(VOUT_MAX, raw)
                self._log_event("SET", f"VOUT_MAX réglé à {value} V")
            elif name == "VOUT_MARGIN_HIGH":
                raw = vout_encode(float(value))
                self.write_word(VOUT_MARGIN_HIGH, raw)
                self._log_event("SET", f"VOUT_MARGIN_HIGH réglé à {value} V")
            elif name == "VOUT_MARGIN_LOW":
                raw = vout_encode(float(value))
                self.write_word(VOUT_MARGIN_LOW, raw)
                self._log_event("SET", f"VOUT_MARGIN_LOW réglé à {value} V")
            elif name == "VOUT_OV_FAULT_LIMIT":
                raw = linear11_encode(float(value))
                self.write_word(VOUT_OV_FAULT_LIMIT, raw)
                self._log_event("SET", f"VOUT_OV_FAULT_LIMIT réglé à {value} V")
            elif name == "VOUT_UV_FAULT_LIMIT":
                raw = linear11_encode(float(value))
                self.write_word(VOUT_UV_FAULT_LIMIT, raw)
                self._log_event("SET", f"VOUT_UV_FAULT_LIMIT réglé à {value} V")
            elif name == "IOUT_OC_FAULT_LIMIT":
                raw = linear11_encode(float(value))
                self.write_word(IOUT_OC_FAULT_LIMIT, raw)
                self._log_event("SET", f"IOUT_OC_FAULT_LIMIT réglé à {value} A")
            elif name == "OT_FAULT_LIMIT":
                raw = linear11_encode(float(value))
                self.write_word(OT_FAULT_LIMIT, raw)
                self._log_event("SET", f"OT_FAULT_LIMIT réglé à {value} °C")
            elif name == "TON_DELAY":
                raw = linear11_encode(float(value))
                self.write_word(TON_DELAY, raw)
                self._log_event("SET", f"TON_DELAY réglé à {value} ms")
            elif name == "TOFF_DELAY":
                raw = linear11_encode(float(value))
                self.write_word(TOFF_DELAY, raw)
                self._log_event("SET", f"TOFF_DELAY réglé à {value} ms")
            elif name == "OPERATION":
                self.write_byte(OPERATION, int(value))
                self._log_event("SET", f"OPERATION modifiée: {items_OPERATION[int(value)]}")
            elif name == "ON_OFF_CONFIG":
                self.write_byte(ON_OFF_CONFIG, int(value))
                self._log_event("SET", f"ON_OFF_CONFIG modifiée: {items_ON_OFF_CONFIG[int(value)]}")
            elif name == "MFR_BRICK_CONTROL":
                self.write_byte(MFR_BRICK_CONTROL, int(value))
                self._log_event("SET", f"MFR_BRICK_CONTROL: {items_BRICK_CONTROL[int(value)]}")
            elif name == "MFR_POWER_ENABLED":
                self.write_byte(MFR_POWER_ENABLED, int(value))
                self._log_event("SET", f"MFR_POWER_ENABLED: {items_POWER_ENABLED[int(value)]}")
            elif name == "MFR_AUTO_ON":
                self.write_byte(MFR_AUTO_ON, int(value))
                self._log_event("SET", f"MFR_AUTO_ON: {items_AUTO_ON[int(value)]}")
            elif name == "MFR_ENABLE_DIAG":
                self.write_byte(MFR_ENABLE_DIAG, int(value))
                self._log_event("SET", f"MFR_ENABLE_DIAG: {items_NO_YES[int(value)]}")
            else:
                return {"status": "error", "error": f"Paramètre inconnu: {name}"}

            return {"status": "ok", "name": name, "value": value}
        except Exception as e:
            self._log_event("ERR", f"Erreur écriture {name}: {e}")
            return {"status": "error", "error": str(e)}

    # --------------------------------------------------------------------------
    # LECTURE DE LA TÉLÉMÉTRIE COMPLÈTE
    # --------------------------------------------------------------------------
    def get_telemetry(self) -> dict:
        """Retourne l'état télémétrique complet pour l'UI."""
        try:
            # Lectures des grandeurs physiques
            if not self.is_mock and 5 <= self.active_page <= 8:
                # Mode Hot-Swap LTC4286
                raw_vin = self.read_word(READ_VIN)
                vin = (raw_vin * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTEEN - 1.0)
                
                raw_iin = self.read_word(READ_IIN)
                iin = (raw_iin * LTC4286_SENSE_FULL_SCALE) / ((TWO_POWER_FIFTEEN - 1.0) * R_SHUNT_HS)
                
                raw_vout = self.read_word(READ_VOUT)
                vout = (raw_vout * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTEEN - 1.0)
                
                raw_iout = self.read_word(READ_IOUT)
                iout = (raw_iout * LTC4286_SENSE_FULL_SCALE) / ((TWO_POWER_FIFTEEN - 1.0) * R_SHUNT_HS)
                
                raw_t1 = self.read_word(READ_TEMPERATURE_1)
                temp1 = raw_t1 + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
                raw_t2 = self.read_word(READ_TEMPERATURE_2)
                temp2 = raw_t2 + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
                raw_t3 = self.read_word(READ_TEMPERATURE_3)
                temp3 = raw_t3 + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS

                raw_pout = self.read_word(READ_POUT)
                pout = (raw_pout * LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTEEN) / (((TWO_POWER_FIFTEEN - 1.0) ** 2) * R_SHUNT_HS)
                raw_pin = self.read_word(READ_PIN)
                pin = (raw_pin * LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTEEN) / (((TWO_POWER_FIFTEEN - 1.0) ** 2) * R_SHUNT_HS)
            else:
                # Mode Standard Linear11 & VOUT Mode
                raw_vin = self.read_word(READ_VIN)
                vin = linear11_decode(raw_vin)
                
                raw_iin = self.read_word(READ_IIN)
                iin = linear11_decode(raw_iin)
                
                raw_vout = self.read_word(READ_VOUT)
                vout = vout_decode(raw_vout)
                
                raw_iout = self.read_word(READ_IOUT)
                iout = linear11_decode(raw_iout)
                
                raw_t1 = self.read_word(READ_TEMPERATURE_1)
                temp1 = linear11_decode(raw_t1)
                raw_t2 = self.read_word(READ_TEMPERATURE_2)
                temp2 = linear11_decode(raw_t2)
                raw_t3 = self.read_word(READ_TEMPERATURE_3)
                temp3 = linear11_decode(raw_t3)

                raw_pout = self.read_word(READ_POUT)
                pout = linear11_decode(raw_pout)
                raw_pin = self.read_word(READ_PIN)
                pin = linear11_decode(raw_pin)

            # Calcul du rendement
            if pin > 10.0 and pout > 0.0:
                efficiency = min(100.0, max(0.0, (pout / pin) * 100.0))
            elif vout > 1.0 and iout > 0.5 and vin > 10.0 and iin > 0.1:
                p_calc_in = vin * iin
                p_calc_out = vout * iout
                efficiency = min(100.0, max(0.0, (p_calc_out / p_calc_in) * 100.0))
            else:
                efficiency = 0.0

            # Statuts
            status_word = self.read_word(STATUS_WORD)
            status_vout = self.read_byte(STATUS_VOUT)
            status_iout = self.read_byte(STATUS_IOUT)
            status_input = self.read_byte(STATUS_INPUT)
            status_temp = self.read_byte(STATUS_TEMPERATURE)
            status_cml = self.read_byte(STATUS_CML)
            status_mfr = self.read_byte(STATUS_MFR_SPECIFIC)
            operation = self.read_byte(OPERATION)

            # Consignes actuelles
            raw_vcmd = self.read_word(VOUT_COMMAND)
            vout_cmd = vout_decode(raw_vcmd)

            # Décodage des alertes
            is_powered = (operation in [0x80, 0x88, 0x98, 0xA8, 2, 3, 4, 5, 6]) and (vout > 1.0)
            has_fault = bool(status_word & 0x7E7C) # Masque de défauts critiques

            faults_list = []
            if status_word & (1 << 15): faults_list.append({"code": "VOUT_FAULT", "label": "VOUT Fault", "level": "danger"})
            if status_word & (1 << 14): faults_list.append({"code": "IOUT_FAULT", "label": "IOUT Fault / Overcurrent", "level": "danger"})
            if status_word & (1 << 13): faults_list.append({"code": "INPUT_FAULT", "label": "VIN Fault / Under/Overvoltage", "level": "danger"})
            if status_word & (1 << 12): faults_list.append({"code": "MFR_FAULT", "label": "MFR Specific Fault", "level": "warning"})
            if status_word & (1 << 11): faults_list.append({"code": "POWER_GOOD", "label": "Power Good Negated", "level": "warning"})
            if status_word & (1 << 2):  faults_list.append({"code": "TEMP_FAULT", "label": "Temperature Fault / OT", "level": "danger"})
            if status_word & (1 << 1):  faults_list.append({"code": "CML_FAULT", "label": "CML / Communication Error", "level": "danger"})
            if status_vout & (1 << 7):  faults_list.append({"code": "VOUT_OV_FAULT", "label": "VOUT Overvoltage Fault", "level": "danger"})
            if status_vout & (1 << 6):  faults_list.append({"code": "VOUT_OV_WARN", "label": "VOUT Overvoltage Warning", "level": "warning"})
            if status_vout & (1 << 5):  faults_list.append({"code": "VOUT_UV_WARN", "label": "VOUT Undervoltage Warning", "level": "warning"})
            if status_vout & (1 << 4):  faults_list.append({"code": "VOUT_UV_FAULT", "label": "VOUT Undervoltage Fault", "level": "danger"})
            if status_iout & (1 << 7):  faults_list.append({"code": "IOUT_OC_FAULT", "label": "IOUT Overcurrent Fault", "level": "danger"})
            if status_temp & (1 << 7):  faults_list.append({"code": "OT_FAULT", "label": "Overtemperature Fault", "level": "danger"})

            return {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "is_mock": self.is_mock,
                "bus_id": self.bus_id,
                "board_addr": f"0x{self.board_addr:02X}",
                "page": self.active_page,
                "page_name": items_PAGE[min(self.active_page, len(items_PAGE)-1)],
                "power_state": is_powered,
                "has_fault": has_fault,
                "vin": round(max(0.0, vin), 2),
                "iin": round(max(0.0, iin), 2),
                "vout": round(max(0.0, vout), 2),
                "iout": round(max(0.0, iout), 2),
                "pin": round(max(0.0, pin), 1),
                "pout": round(max(0.0, pout), 1),
                "efficiency": round(efficiency, 1),
                "temp1": round(temp1, 1),
                "temp2": round(temp2, 1),
                "temp3": round(temp3, 1),
                "vout_command": round(vout_cmd, 2),
                "operation": operation,
                "operation_name": items_OPERATION[operation] if operation < len(items_OPERATION) else f"0x{operation:02X}",
                "status_word": f"0x{status_word:04X}",
                "status_vout": f"0x{status_vout:02X}",
                "status_iout": f"0x{status_iout:02X}",
                "status_input": f"0x{status_input:02X}",
                "status_temp": f"0x{status_temp:02X}",
                "status_cml": f"0x{status_cml:02X}",
                "status_mfr": f"0x{status_mfr:02X}",
                "faults": faults_list
            }
        except Exception as e:
            return {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "is_mock": self.is_mock,
                "bus_id": self.bus_id,
                "board_addr": f"0x{self.board_addr:02X}",
                "page": self.active_page,
                "error": str(e)
            }

    def get_all_config(self) -> dict:
        """Retourne l'ensemble des registres et configurations modifiables pour l'UI."""
        return {
            "page": self.active_page,
            "pages_list": items_PAGE,
            "operation_list": items_OPERATION,
            "on_off_config_list": items_ON_OFF_CONFIG,
            "brick_control_list": items_BRICK_CONTROL,
            "power_enabled_list": items_POWER_ENABLED,
            "auto_on_list": items_AUTO_ON,
            "diag_list": items_NO_YES,
            "current_config": {
                "vout_command": round(vout_decode(self.read_word(VOUT_COMMAND)), 2),
                "vout_max": round(vout_decode(self.read_word(VOUT_MAX)), 2),
                "vout_margin_high": round(vout_decode(self.read_word(VOUT_MARGIN_HIGH)), 2),
                "vout_margin_low": round(vout_decode(self.read_word(VOUT_MARGIN_LOW)), 2),
                "operation": self.read_byte(OPERATION),
                "on_off_config": self.read_byte(ON_OFF_CONFIG),
                "write_protect": self.read_byte(WRITE_PROTECT),
                "brick_control": self.read_byte(MFR_BRICK_CONTROL),
                "power_enabled": self.read_byte(MFR_POWER_ENABLED),
                "auto_on": self.read_byte(MFR_AUTO_ON),
                "diag_mode": self.read_byte(MFR_ENABLE_DIAG),
            }
        }

    def get_logs(self) -> list:
        """Retourne l'historique des derniers événements."""
        return list(reversed(self.logs))
