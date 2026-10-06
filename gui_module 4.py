#!/usr/bin/env python3

# IMP
import argparse
import curses
import errno
import csv
from datetime import datetime
import os
import re
from   smbus2 import SMBus
import sys
import time
import threading

# OAS
DEV_NUMBER_OF_PAGES = 10
DEV_NUMBER_OF_MODULES = 3

DEV_UNLOCK_CODE = [0xAA, 0x55, 0xA5, 0x5A] # for MFR_PSU_REBOOT for example

LTC4286_SENSE_FULL_SCALE                = 0.032
LTC4286_SOURCE_FULL_SCALE               = 102.4
TWO_POWER_FIFTHTEEN                     = 2.0 ** 15.0
LTC4286_I_CONSTANT_FOR_ADC_CONVERSION   = 0.032
LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS    = -273
R_SHUNT_HS                              = 0.0003

######################################################               ELEMENTS SELECTION    ##########################################################

MENU_SELECTION_OFFSET = 100
PMBUS_WRITE_1_SELECTION_OFFSET = 200
PMBUS_WRITE_2_SELECTION_OFFSET = 300

######################################################                 COLOR DEFINE        ##########################################################

# Colors
F_RED_B_BLACK       = 1
F_GREEN_B_BLACK     = 2
F_YELLOW_B_BLACK    = 3
F_BLUE_B_BLACK      = 4
F_MAGENTA_B_BLACK   = 5
F_CYAN_B_BLACK      = 6
F_WHITE_B_BLACK     = 7
F_BLACK_B_RED       = 8
F_BLACK_B_GREEN     = 9
F_BLACK_B_YELLOW    = 10
F_BLACK_B_WHITE     = 11

######################################################      X Y W H POSITION DEFINE        ##########################################################

# Positionning and size of UI elements
W_COLUMN_1      = 27
W_COLUMN_2      = 46
W_COLUMN_3      = 45
W_COLUMN_4      = 45
W_COLUMN_5      = 65

H_COLUMN_1      = 58
H_COLUMN_2      = H_COLUMN_1 
H_COLUMN_3      = H_COLUMN_1
H_COLUMN_4      = H_COLUMN_1
H_COLUMN_5      = H_COLUMN_1

X_COLUMN_1      = 0
X_COLUMN_2      = W_COLUMN_1
X_COLUMN_3      = W_COLUMN_1 + W_COLUMN_2 
X_COLUMN_4      = W_COLUMN_1 + W_COLUMN_2 +  W_COLUMN_3
X_COLUMN_5      = W_COLUMN_1 + W_COLUMN_2 +  W_COLUMN_3 + W_COLUMN_4

## Title        
X_TITLE         = X_COLUMN_1
Y_TITLE         = 0
W_TITLE         = W_COLUMN_1
H_TITLE         = 9

## Menu
TITLE_MENU          = "Menu"
X_MENU              = X_COLUMN_1
Y_MENU              = Y_TITLE + H_TITLE
W_MENU              = W_COLUMN_1
H_MENU              = 3

## Helper
TITLE_HELPER        = "Helper"
X_HELPER            = X_COLUMN_1
Y_HELPER            = Y_MENU + H_MENU
W_HELPER            = W_COLUMN_1
H_HELPER            = 6

## Efficiency
TITLE_EFFICIENCY    = "Efficiency"
X_EFFICIENCY        = X_COLUMN_1
Y_EFFICIENCY        = Y_HELPER + H_HELPER
W_EFFICIENCY        = W_COLUMN_1
H_EFFICIENCY        = 4

## PMBus write commands
TITLE_PMBUS_WRITE_1 = "PMBus WRITE commands 1/2"
X_PMBUS_WRITE_1     = X_COLUMN_2
Y_PMBUS_WRITE_1     = 0
W_PMBUS_WRITE_1     = W_COLUMN_2
H_PMBUS_WRITE_1     = H_COLUMN_2

## PMBus write commands
TITLE_PMBUS_WRITE_2 = "PMBus WRITE commands 2/2"
X_PMBUS_WRITE_2     = X_COLUMN_3
Y_PMBUS_WRITE_2     = 0
W_PMBUS_WRITE_2     = W_COLUMN_3
H_PMBUS_WRITE_2     = H_COLUMN_3

## PMBus read commands n1
TITLE_PMBUS_READ_1  = "PMBus READ commands 1/2"
X_PMBUS_READ_1      = X_COLUMN_4
Y_PMBUS_READ_1      = 0
W_PMBUS_READ_1      = W_COLUMN_4
H_PMBUS_READ_1      = H_COLUMN_4

## PMBus read commands n2
TITLE_PMBUS_READ_2  = "PMBus READ commands 2/2"
X_PMBUS_READ_2      = X_COLUMN_5
Y_PMBUS_READ_2      = 0
W_PMBUS_READ_2      = W_COLUMN_5
H_PMBUS_READ_2      = H_COLUMN_5

## Total dimensions
W_TOTAL         = W_COLUMN_1 + W_COLUMN_2 + W_COLUMN_3 + W_COLUMN_4 + W_COLUMN_5
H_TOTAL         = H_COLUMN_1

######################################################      PMBUS REGISTER        ##########################################################

# PMBus Registers
PAGE                                = 0x00
OPERATION                           = 0x01
ON_OFF_CONFIG                       = 0x02
CLEAR_FAULTS                        = 0x03
RESTORE_DEFAULT_ALL                 = 0x12
STORE_USER_ALL                      = 0x15
RESTORE_USER_ALL                    = 0x16
WRITE_PROTECT                       = 0x10
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

MFR_BRICK_CONTROL                   = 0xE0
MFR_VIN_PG_THRESH                   = 0xE1

MFR_ISHARE_THRESHOLD          = 0xE2
MFR_POWER_ENABLED                   = 0xE3
MFR_POWER_GOOD                      = 0xE4
MFR_BRICK_FAILURE                   = 0xE5
MFR_FAULTS_ORING                    = 0xE6
MFR_PARAMS_DIFF                     = 0xE7
MFR_BUS_STATUS                      = 0xE8
MFR_PMBUS_ALERT                     = 0xE9
MFR_BMC_SIGNALS                     = 0xEA
MFR_PMBUS_ADDR                      = 0xEB
MFR_CAL_DATA                        = 0xEC
MFR_AUTO_ON                         = 0xED
MFR_ENABLE_DIAG                     = 0xEE
MFR_DEBUG_STATUS                    = 0xEF

## Modules specifics PMBus commands
MFR_ADDED_DROOP_DURING_RAMP         = 0xFC

## Hotswaps specifics PMBus commands
MFR_SYSTEM_STATUS1                  = 0xE0
MFR_SYSTEM_STATUS2                  = 0xE1
MFR_CONFIG1                         = 0xF2

FIRST_PMBUS_READ_2                  = STATUS_BYTE

######################################################      PMBUS ATTRIBUTS        ##########################################################

# format of PMbus commands
FORMAT_LINEAR       = 0b000,
FORMAT_SINT16       = 0b001,
FORMAT_IEEE_FP      = 0b010,
FORMAT_DIRECT       = 0b011,
FORMAT_UINT8        = 0b100,
FORMAT_VID          = 0b101,
FORMAT_MFR_SPECIFIC = 0b110,
FORMAT_OTHER        = 0b111,

# type of PMBus data
TYPE_NONE           = 0,
TYPE_ZERO           = 1,
TYPE_BYTE           = 2,
TYPE_WORD           = 3,
TYPE_BLOCK          = 4,

# which slaves manage this PMBus command
SLAVE_ALL           = 8,
SLAVE_MCU_HSAux     = 7,
SLAVE_MCU_HS        = 6,
SLAVE_MCU_Mx        = 5,
SLAVE_MCU           = 4,
SLAVE_HS_Mx         = 3,
SLAVE_HS            = 2,
SLAVE_Mx            = 1,

#WRITE_PROTECT values
WP_DISABLE_OTHER                            = 0b10000000 # Disable all writes except to the WRITE_PROTECT command
WP_EN_WP_OP_PG                              = 0b01000000 # Disable all writes except to the WRITE_PROTECT, OPERATION and PAGE commands
WP_EN_WP_OP_PG_OFC_VOUTCMD                  = 0b00100000 # Disable all writes except to the WRITE_PROTECT, OPERATION, PAGE, ON_OFF_CONFIG and VOUT_COMMAND commands
WP_EN_ALL                                   = 0b00000000 # Enable writes to all commands

#OPERATION values
OPERATION_IMMEDIATE_OFF                     = 0b00000000
OPERATION_SOFT_OFF                          = 0b01000000
OPERATION_ON_MARGIN_OFF                     = 0b10000000
OPERATION_ON_MARGIN_LOW_IGNORE_FAULT        = 0b10010100
OPERATION_ON_MARGIN_LOW_ACT_ON_FAULT        = 0b10011000
OPERATION_ON_MARGIN_HIGH_IGNORE_FAULT       = 0b10100100
OPERATION_ON_MARGIN_HIGH_ACT_ON_FAULT       = 0b10101000
OPERATION_NONE                              = 0b11111111

#ON_OFF_CONFIG values
ON_OFF_CONFIG_IGNORE_ALL                    = 0x02
ON_OFF_CONFIG_PMBUS_ONLY                    = 0x1B
ON_OFF_CONFIG_RC_ONLY                       = 0x17
ON_OFF_CONFIG_PMBUS_RC                      = 0x1F

#MFR_RESPONSE_UNIT_CFG values
MFR_RESPONSE_UNIT_CFG_1MS                   = 0b00000000
MFR_RESPONSE_UNIT_CFG_10MS                  = 0b01010101
MFR_RESPONSE_UNIT_CFG_100MS                 = 0b10101010
MFR_RESPONSE_UNIT_CFG_1S                    = 0b11111111
#MFR_RESPONSE_UNIT_CFG bits
MFR_RESPONSE_UNIT_CFG_VOUT                  = 6
MFR_RESPONSE_UNIT_CFG_VIN                   = 4
MFR_RESPONSE_UNIT_CFG_IOUT                  = 2
MFR_RESPONSE_UNIT_CFG_TEMP                  = 0

#MFR_PSU_POWER_CYCLE
POWER_CYCLE_05S                             = 0x05
POWER_CYCLE_10S                             = 0x0A
POWER_CYCLE_20S                             = 0x14

#MFR_BRICK_CONTROL values
BRICK_CONTROL_ONE_PLUS_ONE                  = 0b010      # Command to choose 1+1 bricks configuration
BRICK_CONTROL_TWO_PLUS_ONE                  = 0b011      # Command to choose 2+1 bricks configuration

#MFR_VIN_PG_THRESH factors
MFR_VIN_PG_THRESH_GAIN      = 0.125
MFR_VIN_PG_THRESH_OFFSET    = 40.000

MFR_CAL_DATA_CAL1 = [0, 0, 0, 0, 0, 0] # for MFR_CAL_DATA
MFR_CAL_DATA_CAL2 = [1.2, 2.3, 3.4, -9.8, -7.8, -6.5] # for MFR_CAL_DATA
MFR_CAL_DATA_CAL3 = [12.3, -10.1, -8.7, 9.5, -1.1, 1.1] # for MFR_CAL_DATA

#MFR_CONFIG1 values as advised by ltc datasheet
# MFR_CONFIG_1_ILIM_06       = 0b0100010101110010
# MFR_CONFIG_1_ILIM_08       = 0b0100110101110010
# MFR_CONFIG_1_ILIM_10       = 0b0101010101110010
# MFR_CONFIG_1_ILIM_12       = 0b0101110101110010
# MFR_CONFIG_1_ILIM_14       = 0b0110010101110010
# MFR_CONFIG_1_ILIM_18       = 0b0111010101110010
# MFR_CONFIG_1_ILIM_16       = 0b0110110101110010
# MFR_CONFIG_1_ILIM_20       = 0b0111110101110010
MFR_CONFIG_1_ILIM_06       = 0b0000010101110010
MFR_CONFIG_1_ILIM_07       = 0b0000100101110010
MFR_CONFIG_1_ILIM_08       = 0b0000110101110010
MFR_CONFIG_1_ILIM_09       = 0b0001000101110010
MFR_CONFIG_1_ILIM_10       = 0b0001010101110010
MFR_CONFIG_1_ILIM_11       = 0b0001100101110010
MFR_CONFIG_1_ILIM_12       = 0b0001110101110010
MFR_CONFIG_1_ILIM_13       = 0b0010000101110010
MFR_CONFIG_1_ILIM_14       = 0b0010010101110010
MFR_CONFIG_1_ILIM_15       = 0b0010100101110010
MFR_CONFIG_1_ILIM_16       = 0b0010110101110010
MFR_CONFIG_1_ILIM_17       = 0b0011000101110010
MFR_CONFIG_1_ILIM_18       = 0b0011010101110010
MFR_CONFIG_1_ILIM_19       = 0b0011100101110010
MFR_CONFIG_1_ILIM_20       = 0b0011110101110010

######################################################      STRING DEFINE        ##########################################################


AppTitle = "PMBus test bench"

logo = [
    " ___   ___ ____  ___ ___ ",
    "|__ \\ / __|  _ \\/ __|_ _|",
    "  _) | |  | |_) \\__ \\| | ",
    " / _/| |__|  _ < __) | | ",
    "|____|\\___|_| \\_\\___/___|",
]

######################################################      PMBUS COMMANDS        ##########################################################
items_PAGE                          = ["00h (All)", "01h (Module 1)", "02h (Module 2)", "03h (Module 3)", "RESERVED", "05h (HS 48V Aux)", "06h (HS 1)", "07h (HS 2)", "08h (HS 3)"]                                               
items_OPERATION                     = ["Immediate OFF", "Soft OFF", "ON (nominal)", "ON Margin LOW Ign", "ON Margin LOW Flts", "ON Margin HIGH Ign", "ON Margin HIGH Flts",]                                                                                                                                                                                                                       
items_ON_OFF_CONFIG                 = ["Ignore all", "PMBus", "Remote Control", "PMBus & RC"]
items_WRITE_PROTECT                 = ["DISABLE Other", "En Operation/Page", "En OP/PG/OFC/V_CMD", "Enable ALL"]                                                                                                                                                                                                                                                                                                         
items_QUERY                         = ["OPERATION", "VOUT_MODE", "VOUT_COMMAND", "VOUT_OV_FAULT_LIMIT", "OT_FAULT_RESPONSE", "TON_DELAY", "STATUS_BYTE", "STATUS_WORD", "READ_VIN", "MFR_EFFICIENCY_LL", "MFR_PSU_POWER_CYCLE", "MFR_FWVERSION_NUMBER", "MFR_BRICK_CONTROL"]
items_VOUT_COMMAND                  = [6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.28, 11.5, 12, 12.5, 12.72, 13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17]
items_VOUT_MAX                      = items_VOUT_COMMAND                                                                                                                                                                               
items_VOUT_MARGIN_HIGH              = items_VOUT_COMMAND                                                                                                                                                                               
items_VOUT_MARGIN_LOW               = items_VOUT_COMMAND                                                                                                                                                                               
items_VOUT_TRANSITION_RATE          = [0.0001, 0.001, 0.005, 0.01, 0.1, 1] 
items_VOUT_DROOP                    = [0.1, 0.5, 1, 2, 10, 50] 
items_VOUT_OV_FAULT_LIMIT           = [10, 11, 12, 12.72, 13, 14, 15.6, 16, 17, 39, 40, 41, 45, 48, 50, 555, 59, 60, 61]  
items_VOUT_OV_WARN_LIMIT            = items_VOUT_OV_FAULT_LIMIT   
items_VOUT_UV_WARN_LIMIT            = [0, 5, 10, 11, 11.28, 12, 13, 38, 39, 40, 41, 48, 50, 55, 59, 60, 61]    
items_VOUT_UV_FAULT_LIMIT           = items_VOUT_UV_WARN_LIMIT  
items_IOUT_OC_FAULT_LIMIT           = [0.5, 1, 5, 10, 48, 60, 100, 105, 110, 160, 200, 205, 210, 220, 250]
items_IOUT_OC_LV_FAULT_LIMIT        = [1, 3, 5, 7, 9, 10, 11]
items_IOUT_OC_WARN_LIMIT            = items_IOUT_OC_FAULT_LIMIT
items_IOUT_UC_FAULT_LIMIT           = [-70, -50, -35, -20, -5, -1]
items_OT_FAULT_LIMIT                = [40, 50, 60, 70, 80, 90, 100, 110, 120]     
items_OT_WARN_LIMIT                 = items_OT_FAULT_LIMIT
items_UT_WARN_LIMIT                 = [-70, -50, -40, -10, 0, 20, 40]
items_UT_FAULT_LIMIT                = items_UT_WARN_LIMIT
items_VIN_OV_FAULT_LIMIT            = [39, 40, 41, 45, 50, 55, 59, 60, 61, 70, 80, 85, 90, 95, 100] 
items_VIN_OV_WARN_LIMIT             = items_VIN_OV_FAULT_LIMIT 
items_VIN_UV_WARN_LIMIT             = [31, 33, 35, 37, 39, 40, 41, 43, 45]
items_VIN_UV_FAULT_LIMIT            = items_VIN_UV_WARN_LIMIT   
items_IIN_OC_FAULT_LIMIT            = [0.1, 0.2, 10, 30,31, 32, 40, 50, 60, 61, 62, 63, 70, 100]
items_IIN_OC_WARN_LIMIT             = items_IIN_OC_FAULT_LIMIT
items_POWER_GOOD_ON                 = [8, 9, 10, 10.25, 10.5, 10.75, 11, 11.25, 11.28, 11.5, 12, 12.25, 12.5, 13, 14]  
items_POWER_GOOD_OFF                = items_POWER_GOOD_ON
items_TON_DELAY                     = [0, 5, 10, 15, 20, 25, 40, 63.8, 80, 140, 200, 255, 400, 655]          
items_TON_RISE                      = items_TON_DELAY            
items_TON_MAX_FAULT_LIMIT           = [20, 40, 63.8, 80, 140, 200, 255, 400, 655]        
items_RESPONSE                      = ["Ignore fault", "Continue 2_Retry 4", "Continue 3_Retry 64", "Shutdown NoRetry", "Shutdown 3_Retry 64","Shutdown Endless", "Shutdown Endless 16", "Disable NoRetry", "Disable 3_Retry 64"] #[0x00, 0x52, 0x5E, 0x80, 0x9E, 0xB8, 0xBC, 0xC0, 0xDE]  
items_Current_RESPONSE              = ["Ignore flt CstCr", "CdCstCr 2_Retry 4", "CdCstCr 3_Retry 64", "DlCstCr NoRetry", "DlCstCr 3_Retry 64", "DlCstCr Endless", "DlCstCr Endless 16", "Shutdown NoRetry", "Shutdown 3_Retry 64"] #[0x00, 0x52, 0x5E, 0x80, 0x9E, 0xB8, 0xBC, 0xC0, 0xDE] 
items_TOFF_DELAY                    = items_TON_DELAY                                                                                                                                                                      
items_TOFF_FALL                     = items_TON_DELAY                                                                                                                                                                                                                                                                                                                                                                     
items_TOFF_MAX_WARN_LIMIT           = items_TON_MAX_FAULT_LIMIT
items_POUT_OP_FAULT_LIMIT           = [20, 30, 40, 50, 75, 100, 500, 1000, 1200, 1300, 2000, 2050, 2100, 2150, 2350, 2400, 2500, 2600, 3000, 4092, 5000]
items_POUT_OP_WARN_LIMIT            = items_POUT_OP_FAULT_LIMIT
items_PIN_OP_WARN_LIMIT             = items_POUT_OP_FAULT_LIMIT
items_MFR_PSU_REBOOT                = [5, 10, 20, 30, 40, 50, 60]
items_MFR_RESPONSE_UNIT_CFG         = ["n * 1ms/u", "n * 10ms/u", "n * 100ms/u", "n * 1s/u"]                                                                                                                                                                                                                       
items_NO_YES                        = ["No", "Yes"]
items_AUTO_ON                       = ["Auto ON activated", "NO auto on"]                                                                                                                                                                                                                                                                                                                                                                             
items_MFR_PSU_POWER_CYCLE           = [POWER_CYCLE_05S, POWER_CYCLE_10S, POWER_CYCLE_20S, 30]                                                                                                                                                                             
items_MFR_BRICK_CONTROL             = ["1 + 1", "2 + 1"]                                                                                                                                                                              
items_MFR_VIN_PG_THRESH             = [40, 45, 48, 49, 50, 51, 52, 55, 60, 65]    
items_MFR_ISHARE_THRESHOLD    = [0, 0.5, 1, 5, 10, 50, 100, 500, 1000]
items_MFR_POWER_ENABLED             = ["All bricks OFF", "All bricks ON", "Only brick 1 ON", "Only brick 2 ON", "Only brick 3 ON", "Bricks 1 & 3 ON", "Bricks 1 & 2 ON", "Bricks 2 & 3 ON" ]                                                                                                                                                                                                                                                                                                                
items_MFR_PMBUS_ADDR                = [hex(0x59), hex(0x5A), hex(0x5B)]                                                                                                                                                                    
items_MFR_CAL_DATA                  = ["Cal 1", "Cal 2", "Cal 3"]                                                                                                                                                                 
items_MFR_ENABLE_DIAG               = ["No", "Yes"]                                                                                                                                                                                                                                                                                                                                                                                  
items_MFR_CONFIG1                   = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
hex_RESPONSE                        = [0x00, 0x52, 0x5E, 0x80, 0x9E, 0xB8, 0xBC, 0xC0, 0xDE] 

###################################################### PRINT CALLBACK FOR READ INTERPRETATION ##########################################################

def print_PAGE(self, line, pos, raw):
    string = items_PAGE[raw]
    self._win.addstr(line, pos, string, self._board._color)

def print_OPERATION(self, line, pos, raw):
        if not (5 <= self._board._page <= 8): # not hotswap PAGE
            operation = get_operation(raw)
            if operation == OPERATION_IMMEDIATE_OFF:
                string = items_OPERATION[0]
                self._win.addstr(line, pos, string, curses.color_pair(F_RED_B_BLACK))
            elif operation == OPERATION_SOFT_OFF:
                string = items_OPERATION[1]
                self._win.addstr(line, pos, string, curses.color_pair(F_RED_B_BLACK))
            elif operation == OPERATION_ON_MARGIN_OFF:
                string = items_OPERATION[2]
                self._win.addstr(line, pos, string, curses.color_pair(F_GREEN_B_BLACK))
            elif operation == OPERATION_ON_MARGIN_LOW_IGNORE_FAULT:
                string = items_OPERATION[3]
                self._win.addstr(line, pos, string, curses.color_pair(F_YELLOW_B_BLACK))
            elif operation == OPERATION_ON_MARGIN_LOW_ACT_ON_FAULT:
                string = items_OPERATION[4]
                self._win.addstr(line, pos, string, curses.color_pair(F_YELLOW_B_BLACK))
            elif operation == OPERATION_ON_MARGIN_HIGH_IGNORE_FAULT:
                string = items_OPERATION[5]
                self._win.addstr(line, pos, string, curses.color_pair(F_YELLOW_B_BLACK))
            elif operation == OPERATION_ON_MARGIN_HIGH_ACT_ON_FAULT:
                string = items_OPERATION[6]
                self._win.addstr(line, pos, string, curses.color_pair(F_YELLOW_B_BLACK))
        else:
            if raw == 0b10000000:
                self._win.addstr(line, pos, "HS FET cmded ON", curses.color_pair(F_GREEN_B_BLACK))
            elif raw == 0b00000000:
                self._win.addstr(line, pos, "HS FET cmded OFF", curses.color_pair(F_RED_B_BLACK))

def print_ON_OFF_CONFIG(self, line, pos, raw):
    if CHECK_BIT(raw, 4): # Bit 4 : Powerup Operation    : 0 - Always enable | 1 - Enable pin or PMBus
        if CHECK_BIT(raw, 3): # Bit 3 : PMBus Enable Mode    : 0 - Ignore PMBus  | 1 - Use PMBus
            self._win.addstr(line, pos+0, "PMBus", curses.color_pair(F_GREEN_B_BLACK))
        else:
            self._win.addstr(line, pos+0, "PMBus", curses.color_pair(F_RED_B_BLACK))
        if CHECK_BIT(raw, 2): # Bit 2 : PMBus Enable Mode    : 0 - Ignore pin    | 1 - Use pin
            self._win.addstr(line, pos+6, "RC", curses.color_pair(F_GREEN_B_BLACK))
        else:
            self._win.addstr(line, pos+6, "RC", curses.color_pair(F_RED_B_BLACK))         
        if CHECK_BIT(raw, 1): # Bit 1 : Enable Pin Polarity  : 0 - Active High   | 1 - Active Low
            self._win.addstr(line, pos+9, "Low", curses.color_pair(F_GREEN_B_BLACK))
        else:
            self._win.addstr(line, pos+9, "High", curses.color_pair(F_YELLOW_B_BLACK))         
        if CHECK_BIT(raw, 0): # Bit 0 : Pin Disable Action   : 0 - Soft Off      | 1 - Immediate Off
            self._win.addstr(line, pos+14, "Imm", curses.color_pair(F_GREEN_B_BLACK))
        else:
            self._win.addstr(line, pos+14, "Soft", curses.color_pair(F_YELLOW_B_BLACK))    
    else:
        self._win.addstr(line, pos, "Ignore all", curses.color_pair(F_YELLOW_B_BLACK))     

def print_WRITE_PROTECT(self, line, pos, raw):
    if raw == WP_DISABLE_OTHER:
        string = items_WRITE_PROTECT[0]
        self._win.addstr(line, pos, string, curses.color_pair(F_RED_B_BLACK))
    elif raw == WP_EN_WP_OP_PG:
        string = items_WRITE_PROTECT[1]
        self._win.addstr(line, pos, string, curses.color_pair(F_YELLOW_B_BLACK))
    elif raw == WP_EN_WP_OP_PG_OFC_VOUTCMD:
        string = items_WRITE_PROTECT[2]
        self._win.addstr(line, pos, string, curses.color_pair(F_YELLOW_B_BLACK))                            
    elif raw == WP_EN_ALL:
        string = items_WRITE_PROTECT[3]
        self._win.addstr(line, pos, string, curses.color_pair(F_GREEN_B_BLACK)) 

# CAPABILITY command data byte format :
# bit 7 : PEC support
# bits 6:5 : Max supported bus speed | 00 : 100kHz | 01 400 kHz
# bit 4 : SMBALERT# | 1 : The device does have a SMBALERT# pin and does support the SMBus Alert Response protocol protocol
# bits 3:0 : Reserved
def print_CAPABILITY(self, line, pos, raw):
    if CHECK_BIT(raw, 7):
        self._win.addstr(line, pos, "PEC", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos, "PEC", curses.color_pair(F_RED_B_BLACK))
    
    if not CHECK_BIT(raw, 6) and not CHECK_BIT(raw, 5) :
        self._win.addstr(line, pos + 4, "100kHz", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 6) and CHECK_BIT(raw, 5):
        self._win.addstr(line, pos + 4, "400kHz", curses.color_pair(F_WHITE_B_BLACK))
    else:
        self._win.addstr(line, pos + 4, "1MHz  ", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 4):
        self._win.addstr(line, pos + 11, "SMbAL#", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos + 11, "SMbAL#", curses.color_pair(F_RED_B_BLACK))
                                 
def print_QUERY(self, line, pos, raw):
    if CHECK_BIT(raw, 7):
        self._win.addstr(line, pos, "Cmd", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos, "Cmd", curses.color_pair(F_RED_B_BLACK))

    if CHECK_BIT(raw, 6):
        self._win.addstr(line, pos+3, " Wr", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+3, " Wr", curses.color_pair(F_RED_B_BLACK))

    if CHECK_BIT(raw, 5):
        self._win.addstr(line, pos+6, " Rd", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+6, " Rd", curses.color_pair(F_RED_B_BLACK)) 

    format = (raw >> 2) & 0b111
    if format == 0b000:
        self._win.addstr(line, pos+9, " Linear", curses.color_pair(F_YELLOW_B_BLACK))
    elif format == 0b001:
        self._win.addstr(line, pos+9, " 16bSig", curses.color_pair(F_YELLOW_B_BLACK))
    elif format == 0b011:
        self._win.addstr(line, pos+9, " IEEE_FP", curses.color_pair(F_YELLOW_B_BLACK))        
    elif format == 0b011:
        self._win.addstr(line, pos+9, " Direct", curses.color_pair(F_YELLOW_B_BLACK))
    elif format == 0b100:
        self._win.addstr(line, pos+9, " 8bUnsig", curses.color_pair(F_YELLOW_B_BLACK))
    elif format == 0b101:
        self._win.addstr(line, pos+9, " VID", curses.color_pair(F_YELLOW_B_BLACK))
    elif format == 0b110:
        self._win.addstr(line, pos+9, " MFR_Spec", curses.color_pair(F_YELLOW_B_BLACK))
    elif format == 0b111:
        self._win.addstr(line, pos+9, " Blocks", curses.color_pair(F_YELLOW_B_BLACK))

def print_VOUT_MODE(self, line, pos, raw):
    format = (raw >> 5) & 0b111
    if format == 0b000:
        exp = twos_complement(raw & 0x1F, 5)
        self._win.addstr(line, pos, "Linear (exp= " + str(exp) + ")", curses.color_pair(F_WHITE_B_BLACK))
    elif format == 0b001:
        self._win.addstr(line, pos, "VID (" + format(raw & 0b11111, '#05b') + ")", curses.color_pair(F_WHITE_B_BLACK))
    elif format == 0b011:
        self._win.addstr(line, pos, "Direct", curses.color_pair(F_WHITE_B_BLACK))     
                                 
def print_VOUT_COMMAND(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                              
def print_VOUT_MAX(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VOUT_MARGIN_HIGH(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VOUT_MARGIN_LOW(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                                           
def print_VOUT_TRANSITION_RATE(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.4f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7,  "V/ms", self._board._color)

def print_VOUT_DROOP(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.4f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7,  "mV/A", self._board._color)

def print_VOUT_OV_FAULT_LIMIT(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                                           
def print_VOUT_OV_WARN_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        mode = self._board.read_byte(VOUT_MODE)
        val = vout_decode(raw, mode)
    else :
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VIN_UV_WARN_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VOUT_UV_WARN_LIMIT(self, line, pos, raw): 
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        mode = self._board.read_byte(VOUT_MODE)
        val = vout_decode(raw, mode)
    else :
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                                           
def print_VOUT_UV_FAULT_LIMIT(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)                                         

def print_IOUT_OC_FAULT_LIMIT(self, line, pos, raw):  
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color) 

def print_IOUT_OC_LV_FAULT_LIMIT(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color) 

def print_IOUT_OC_WARN_LIMIT(self, line, pos, raw): 
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw  * LTC4286_SENSE_FULL_SCALE) / ((TWO_POWER_FIFTHTEEN - 1) * R_SHUNT_HS)) # Hotswap_DirectDecode_Current
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color) 

def print_IOUT_UC_FAULT_LIMIT(self, line, pos, raw): 
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color) 

def print_OT_FAULT_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = raw + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "C", self._board._color) 

def print_OT_WARN_LIMIT(self, line, pos, raw):   
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = raw + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "C", self._board._color) 

def print_UT_WARN_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = raw + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "C", self._board._color) 

def print_UT_FAULT_LIMIT(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "C", self._board._color)       

def print_VIN_OV_FAULT_LIMIT(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VIN_OV_WARN_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VIN_UV_WARN_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_VIN_UV_FAULT_LIMIT(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)    

def print_IIN_OC_FAULT_LIMIT(self, line, pos, raw): 
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color) 

def print_IIN_OC_WARN_LIMIT(self, line, pos, raw): 
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color) 

def print_POWER_GOOD_ON(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color) 

def print_POWER_GOOD_OFF(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color) 

def print_TON_DELAY(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "ms", self._board._color)

def print_TON_RISE(self, line, pos, raw):   
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "ms", self._board._color)

def print_TON_MAX_FAULT_LIMIT(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "ms", self._board._color)    

def print_TOFF_DELAY(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "ms", self._board._color)
                                            
def print_TOFF_FALL(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "ms", self._board._color)

def print_TOFF_MAX_WARN_LIMIT(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "ms", self._board._color)

def print_POUT_OP_FAULT_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw * LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTHTEEN) / (((TWO_POWER_FIFTHTEEN - 1.0) ** 2) * R_SHUNT_HS))
    val = "{:.1f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)  

def print_POUT_OP_WARN_LIMIT(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.1f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)

def print_PIN_OP_WARN_LIMIT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw * LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTHTEEN) / (((TWO_POWER_FIFTHTEEN - 1.0) ** 2) * R_SHUNT_HS))
    val = "{:.1f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)  

def print_RESPONSE(self, line, pos, raw):
    #self._win.addstr(line, pos, "0x{:02X}".format(raw), self._board._color)
    if ((raw  >> 6) & 0b11) == 0b00:
        # The PMBus device continues operation without interruption.
        self._win.addstr(line, pos, "Ignore fault", curses.color_pair(F_YELLOW_B_BLACK))    
    
    elif ((raw  >> 6) & 0b11) == 0b01:
        # The PMBus device continues operation for the delay time specified by bits [2:0] and the delay time unit 
        # specified for that particular fault. If the fault condition is still present at the end of the delay time,
        # the unit responds as programmed in the Retry Setting (bits [5:3]).
        self._win.addstr(line, pos, "Continue", curses.color_pair(F_CYAN_B_BLACK))

        if ((raw  >> 3) & 0b111) == 0b000:
            # A zero value for the Retry Setting means that the unit does not attempt to restart. The output
            # remains disabled until the fault is cleared (Section 10.7).
            self._win.addstr(line, pos+9, "NoRetry", curses.color_pair(F_RED_B_BLACK))    

        elif ((raw  >> 3) & 0b111) == 0b111:
            # The PMBus device attempts to restart continuously, without limitation, until it is commanded
            # OFF (by the CONTROL pin or OPERATION command or both), bias power is removed, or another
            # fault condition causes the unit to shut down.  
            self._win.addstr(line, pos+9, "Endless", curses.color_pair(F_YELLOW_B_BLACK))  

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))
            
        else:
            # The PMBus device attempts to restart the number of times set by these bits. The minimum number is
            # 1 and the maximum number is 6. If the device fails to restart (the fault condition is no longer present and
            # the device is delivering power to the output and operating as programmed) in the allowed
            # number of retries, it disables the output and remains off until the fault is cleared as described in Section
            # 10.7. The time between the start of each attempt to restart is set by the value in bits [2:0] along with the
            # delay time unit specified for that particular fault.
            self._win.addstr(line, pos+9, str((raw  >> 3) & 0b111) + "_Retry", curses.color_pair(F_CYAN_B_BLACK))    

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

    elif ((raw  >> 6) & 0b11) == 0b10:
        # The device shuts down (disables the output) and responds according to the retry setting in bits [5:3].
        self._win.addstr(line, pos, "Disable", curses.color_pair(F_MAGENTA_B_BLACK))

        if ((raw  >> 3) & 0b111) == 0b000:
            # A zero value for the Retry Setting means that the unit does not attempt to restart. The output
            # remains disabled until the fault is cleared (Section 10.7).
            self._win.addstr(line, pos+9, "NoRetry", curses.color_pair(F_RED_B_BLACK))    

        elif ((raw  >> 3) & 0b111) == 0b111:
            # The PMBus device attempts to restart continuously, without limitation, until it is commanded
            # OFF (by the CONTROL pin or OPERATION command or both), bias power is removed, or another
            # fault condition causes the unit to shut down.  
            self._win.addstr(line, pos+9, "Endless", curses.color_pair(F_YELLOW_B_BLACK))

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

        else:
            # The PMBus device attempts to restart the number of times set by these bits. The minimum number is
            # 1 and the maximum number is 6. If the device fails to restart (the fault condition is no longer present and
            # the device is delivering power to the output and operating as programmed) in the allowed
            # number of retries, it disables the output and remains off until the fault is cleared as described in Section
            # 10.7. The time between the start of each attempt to restart is set by the value in bits [2:0] along with the
            # delay time unit specified for that particular fault.
            self._win.addstr(line, pos+9, str((raw  >> 3) & 0b111) + "_Retry", curses.color_pair(F_CYAN_B_BLACK))    

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

    elif ((raw  >> 6) & 0b11) == 0b11:
        # The device’s output is disabled while the fault is present.
        # Operation resumes and the output is enabled
        # when the fault condition no longer exists.
        self._win.addstr(line, pos, "Disable 'til Cleard", curses.color_pair(F_RED_B_BLACK))    

def print_Current_RESPONSE(self, line, pos, raw):
    #self._win.addstr(line, pos, "0x{:02X}".format(raw), self._board._color)
    if ((raw  >> 6) & 0b11) == 0b00:
        # The PMBus device continues to operate indefinitely while maintaining the output current at the value set by
        # IOUT_OC_FAULT_LIMIT (Section 15.8) without regard to the output voltage (known as constant-current or brickwall limiting)
        self._win.addstr(line, pos, "Ignore flt cst curr", curses.color_pair(F_YELLOW_B_BLACK))    
    
    elif ((raw  >> 6) & 0b11) == 0b01:
        # The PMBus device continues to operate indefinitely while maintaining the output current at the value set by
        # IOUT_OC_FAULT_LIMIT as long as the output voltage remains above the minimum value specified by IOUT_OC_LV_FAULT_LIMIT.
        # If the output voltage is pulled down to less than that value, then the PMBus device shuts down and responds according
        # to the Retry setting in bits [5:3].
        self._win.addstr(line, pos, "CdCstCr", curses.color_pair(F_CYAN_B_BLACK))

        if ((raw  >> 3) & 0b111) == 0b000:
            # A zero value for the Retry Setting means that the unit does not attempt to restart. The output
            # remains disabled until the fault is cleared (Section 10.7).
            self._win.addstr(line, pos+9, "NoRetry", curses.color_pair(F_RED_B_BLACK))    

        elif ((raw  >> 3) & 0b111) == 0b111:
            # The PMBus device attempts to restart continuously, without limitation, until it is commanded
            # OFF (by the CONTROL pin or OPERATION command or both), bias power is removed, or another
            # fault condition causes the unit to shut down.  
            self._win.addstr(line, pos+9, "Endless", curses.color_pair(F_YELLOW_B_BLACK))  

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

        else:
            # The PMBus device attempts to restart the number of times set by these bits. The minimum number is
            # 1 and the maximum number is 6. If the device fails to restart (the fault condition is no longer present and
            # the device is delivering power to the output and operating as programmed) in the allowed
            # number of retries, it disables the output and remains off until the fault is cleared as described in Section
            # 10.7. The time between the start of each attempt to restart is set by the value in bits [2:0] along with the
            # delay time unit specified for that particular fault.
            self._win.addstr(line, pos+9, str((raw  >> 3) & 0b111) + "_Retry", curses.color_pair(F_CYAN_B_BLACK))    

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

    elif ((raw  >> 6) & 0b11) == 0b10:
        # The PMBus device continues to operate, maintaining the output current at the value set by IOUT_OC_FAULT_LIMIT
        # without regard to the output voltage, for the delay time set by bits [2:0] and the delay time units for
        # specified in the IOUT_OC_FAULT_RESPONSE. If the device is still operating in current limiting at the end
        # of the delay time, the device responds as programmed by the Retry Setting in bits [5:3].
        self._win.addstr(line, pos, "DlCstCr", curses.color_pair(F_RED_B_BLACK))

        if ((raw  >> 3) & 0b111) == 0b000:
            # A zero value for the Retry Setting means that the unit does not attempt to restart. The output
            # remains disabled until the fault is cleared (Section 10.7).
            self._win.addstr(line, pos+9, "NoRetry", curses.color_pair(F_RED_B_BLACK))

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.                
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

        elif ((raw  >> 3) & 0b111) == 0b111:
            # The PMBus device attempts to restart continuously, without limitation, until it is commanded
            # OFF (by the CONTROL pin or OPERATION command or both), bias power is removed, or another
            # fault condition causes the unit to shut down.  
            self._win.addstr(line, pos+9, "Endless", curses.color_pair(F_YELLOW_B_BLACK))

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

        else:
            # The PMBus device attempts to restart the number of times set by these bits. The minimum number is
            # 1 and the maximum number is 6. If the device fails to restart (the fault condition is no longer present and
            # the device is delivering power to the output and operating as programmed) in the allowed
            # number of retries, it disables the output and remains off until the fault is cleared as described in Section
            # 10.7. The time between the start of each attempt to restart is set by the value in bits [2:0] along with the
            # delay time unit specified for that particular fault.
            self._win.addstr(line, pos+9, str((raw  >> 3) & 0b111) + "_Retry", curses.color_pair(F_CYAN_B_BLACK))    

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

    elif ((raw  >> 6) & 0b11) == 0b11:
        # The PMBus device shuts down and responds as programmed by the Retry Setting in bits [5:3].
        self._win.addstr(line, pos, "Disable", curses.color_pair(F_MAGENTA_B_BLACK))    

        if ((raw  >> 3) & 0b111) == 0b000:
            # A zero value for the Retry Setting means that the unit does not attempt to restart. The output
            # remains disabled until the fault is cleared (Section 10.7).
            self._win.addstr(line, pos+9, "NoRetry", curses.color_pair(F_RED_B_BLACK))

        elif ((raw  >> 3) & 0b111) == 0b111:
            # The PMBus device attempts to restart continuously, without limitation, until it is commanded
            # OFF (by the CONTROL pin or OPERATION command or both), bias power is removed, or another
            # fault condition causes the unit to shut down.  
            self._win.addstr(line, pos+9, "Endless", curses.color_pair(F_YELLOW_B_BLACK))

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

        else:
            # The PMBus device attempts to restart the number of times set by these bits. The minimum number is
            # 1 and the maximum number is 6. If the device fails to restart (the fault condition is no longer present and
            # the device is delivering power to the output and operating as programmed) in the allowed
            # number of retries, it disables the output and remains off until the fault is cleared as described in Section
            # 10.7. The time between the start of each attempt to restart is set by the value in bits [2:0] along with the
            # delay time unit specified for that particular fault.
            self._win.addstr(line, pos+9, str((raw  >> 3) & 0b111) + "_Retry", curses.color_pair(F_CYAN_B_BLACK))    

            # The number of delay time units, which vary depending on the type of fault.
            # This delay time is used for either the amount of time a unit is to continue
            # operating after a fault is detected or for the amount of time between attempts to restart.
            self._win.addstr(line, pos + 17, str(2 **(raw & 0b111)), curses.color_pair(F_WHITE_B_BLACK))

def print_STATUS_BYTE(self, line, pos, raw):

    if CHECK_BIT(raw, 7): # BUSY
        if (1 <= self._board._page <= 3): # modules PAGE
            self._win.addstr(line, pos, "Busy", curses.color_pair(F_WHITE_B_BLACK))
        else:
            self._win.addstr(line, pos, "Busy", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Busy", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # OFF
        self._win.addstr(line, pos+5, "Off", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+5, "Off", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # VOUT_OV_FAULT
        self._win.addstr(line, pos+9, "Ovout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+9, "Ovout", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # IOUT_OC_FAULT
        self._win.addstr(line, pos+15, "Oiout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+15, "Oiout", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # VIN_UV_FAULT
        self._win.addstr(line, pos+21, "Uvin", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+21, "Uvin", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 2): # TEMPERATURE
        self._win.addstr(line, pos+26, "Tmp", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+26, "Tmp", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # CML
        self._win.addstr(line, pos+30, "Cml", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+30, "Cml", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # NONE OF THE ABOVE
        if not (5 <= self._board._page <= 8): # not hotswap PAGE
            self._win.addstr(line, pos+34, "None", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+34, "None", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+34, "None", curses.color_pair(F_WHITE_B_BLACK))
                               
def print_STATUS_WORD(self, line, pos, raw):
    if CHECK_BIT(raw, 15): # VOUT
        self._win.addstr(line, pos, "Vout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Vout", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 14): # IOUT/POUT
        self._win.addstr(line, pos+5, "I/Pout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+5, "I/Pout", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 13): # INPUT
        self._win.addstr(line, pos+12, "IN", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+12, "IN", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 12): # MFR_SPECIFIC
        self._win.addstr(line, pos+15, "Sp", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+15, "Sp", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 11): # POWER_GOOD#
        self._win.addstr(line, pos+18, "PG", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+18, "PG", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 10): # FANS
        if (1 <= self._board._page <= 3): # modules PAGE
            self._win.addstr(line, pos+21, "F", curses.color_pair(F_WHITE_B_BLACK))
        else :
            self._win.addstr(line, pos+21, "F", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+21, "F", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 9): # OTHER
        if (1 <= self._board._page <= 3): # modules PAGE
            self._win.addstr(line, pos+23, "Oth", curses.color_pair(F_WHITE_B_BLACK))
        else :
            self._win.addstr(line, pos+23, "Oth", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+23, "Oth", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 8):  # UNKNOWN
        if (1 <= self._board._page <= 3): # modules PAGE
            self._win.addstr(line, pos+27, "Nk", curses.color_pair(F_WHITE_B_BLACK))
        elif (5 <= self._board._page <= 8): # hotswap PAGE:
            self._win.addstr(line, pos+27, "Nk", curses.color_pair(F_YELLOW_B_BLACK))
        else :
            self._win.addstr(line, pos+27, "Nk", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+27, "Nk", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 7): # BUSY
        if (1 <= self._board._page <= 3): # modules PAGE
            self._win.addstr(line, pos+30, "B", curses.color_pair(F_WHITE_B_BLACK))
        else:
            self._win.addstr(line, pos+30, "B", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+30, "B", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # OFF
        self._win.addstr(line, pos+31, "O", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+31, "O", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # VOUT_OV_FAULT
        self._win.addstr(line, pos+32, "V", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+32, "V", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # IOUT_OC_FAULT
        self._win.addstr(line, pos+33, "I", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+33, "I", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # VIN_UV_FAULT
        self._win.addstr(line, pos+34, "U", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+34, "U", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 2): # TEMPERATURE
        self._win.addstr(line, pos+35, "T", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+35, "T", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # CML
        self._win.addstr(line, pos+36, "C", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+36, "C", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # NONE OF THE ABOVE
        if not (5 <= self._board._page <= 8): # not hotswap PAGE
            self._win.addstr(line, pos+37, "N", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+37, "N", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+37, "N", curses.color_pair(F_WHITE_B_BLACK))
                               
def print_STATUS_VOUT(self, line, pos, raw):

    if CHECK_BIT(raw, 7): # VOUT_OV_FAULT
        self._win.addstr(line, pos, "Ovout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Ovout", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # VOUT_OV_WARNING
        self._win.addstr(line, pos+6, "Ov", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+6, "Ov", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # VOUT_UV_FAULT
        self._win.addstr(line, pos+9, "Uvout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+9, "Uvout", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # VOUT_UV_WARNING
        self._win.addstr(line, pos+15, "Uv", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+15, "Uv", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # VOUT_MAX_WARNING
        self._win.addstr(line, pos+18, "MaxVo", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+18, "MaxVo", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 2): # TON_MAX_FAULT
        self._win.addstr(line, pos+24, "TonM", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+24, "TonM", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # TOFF_MAX_WARNING
        self._win.addstr(line, pos+29, "ToffM", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+29, "ToffM", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # OUT Tracing Error (MFR)
        self._win.addstr(line, pos+35, "Mfr", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+35, "Mfr", curses.color_pair(F_WHITE_B_BLACK))
                               
def print_STATUS_IOUT(self, line, pos, raw):
    if CHECK_BIT(raw, 7): # IOUT_OC_FAULT
        self._win.addstr(line, pos, "Oiout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Oiout", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # IOUT_OC_LV_FAULT
        self._win.addstr(line, pos+6, "OioLv", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+6, "OioLv", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # IOUT_OC_WARNING
        self._win.addstr(line, pos+12, "Oc", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+12, "Oc", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # IOUT_UC_FAULT
        self._win.addstr(line, pos+15, "Uio", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+15, "Uio", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # CURRENT_SHARE_FAULT
        self._win.addstr(line, pos+19, "Cshr", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+19, "Cshr", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 2): # IN_POWER_LIMITING_MODE
        self._win.addstr(line, pos+24, "InPlm", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+24, "InPlm", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # POUT_OP_FAULT
        self._win.addstr(line, pos+30, "Opout", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+30, "Opout", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # POUT_OP_WARNING
        self._win.addstr(line, pos+36, "Op", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+36, "Op", curses.color_pair(F_WHITE_B_BLACK))
                               
def print_STATUS_INPUT(self, line, pos, raw):

    if CHECK_BIT(raw, 7): # VIN_OV_FAULT
        self._win.addstr(line, pos, "Ovin", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Ovin", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # VIN_OV_WARNING
        self._win.addstr(line, pos+5, "Ov", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+5, "Ov", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # VIN_UV_WARNING
        self._win.addstr(line, pos+8, "Uv", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+8, "Uv", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # VIN_UV_FAULT
        self._win.addstr(line, pos+11, "Uvin", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+11, "Uvin", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # UNIT_OFF_INSUFFICIENT_VOLTAGE
        self._win.addstr(line, pos+16, "UnOffInsV", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+16, "UnOffInsV", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 2): # IIN_OC_FAULT
        self._win.addstr(line, pos+26, "Oiin", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+26, "Oiin", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # IIN_OC_WARNING
        self._win.addstr(line, pos+31, "Oc", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+31, "Oc", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # PIN_OP_WARNING
        self._win.addstr(line, pos+34, "Opin", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+34, "Opin", curses.color_pair(F_WHITE_B_BLACK))
                              
def print_STATUS_TEMPERATURE(self, line, pos, raw):
    if CHECK_BIT(raw, 7): # OT_FAULT
        self._win.addstr(line, pos, "OverTemp", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "OverTemp", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # OT_WARNING
        self._win.addstr(line, pos+9, "OverTemp", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+9, "OverTemp", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # UT_WARNING
        self._win.addstr(line, pos+19, "UnderTemp", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+19, "UnderTemp", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # UT_FAULT
        self._win.addstr(line, pos+29, "UnderTemp", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+29, "UnderTemp", curses.color_pair(F_WHITE_B_BLACK))
                        
def print_STATUS_CML(self, line, pos, raw):
    if CHECK_BIT(raw, 7): # INVALID_CMD
        self._win.addstr(line, pos, "Command", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Command", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # INVALID_DATA
        self._win.addstr(line, pos+8, "Data", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+8, "Data", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # PEC
        self._win.addstr(line, pos+13, "PEC", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+13, "PEC", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # MEMORY_FAULT
        if not (5 <= self._board._page <= 8): # not hotswap PAGE
            self._win.addstr(line, pos+17, "Mmory", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+17, "Mmory", curses.color_pair(F_WHITE_B_BLACK)) # RESERVED for HS
    else:
        self._win.addstr(line, pos+17, "Mmory", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # PROCESSOR_FAULT
        if  (self._board._page == 0): # MCU PAGE            
            self._win.addstr(line, pos+23, "Proc", curses.color_pair(F_BLACK_B_RED))
        else :
            self._win.addstr(line, pos+23, "Proc", curses.color_pair(F_WHITE_B_BLACK))
    else:
        self._win.addstr(line, pos+23, "Proc", curses.color_pair(F_WHITE_B_BLACK))
    #if CHECK_BIT(raw, 2): # RESERVED  
    if CHECK_BIT(raw, 1): # OTHER_COM_FAULT
        self._win.addstr(line, pos+28, "OthC", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+28, "OthC", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # OTHER_MEMORY_OR_LOGIC_FAULT
        if not (5 <= self._board._page <= 8): # not hotswap PAGE
            self._win.addstr(line, pos+33, "OthML", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+33, "OthML", curses.color_pair(F_WHITE_B_BLACK)) # RESERVED for HS        
    else:
        self._win.addstr(line, pos+33, "OthML", curses.color_pair(F_WHITE_B_BLACK))
                                
def print_STATUS_OTHER(self, line, pos, raw): # bits [7:4] : RSVD | bits [3:1] : Brick [3:1] OR-ing Fault | bit 0 : RSVD
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        if CHECK_BIT(raw, 3): # ORING_3_FAULT
            self._win.addstr(line, pos, "OR-ing 3", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos, "OR-ing 3", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 2): # ORING_2_FAULT
            self._win.addstr(line, pos+15, "OR-ing 2", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+15, "OR-ing 2", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 1): # ORING_1_FAULT
            self._win.addstr(line, pos+30, "OR-ing 1", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+30, "OR-ing 1", curses.color_pair(F_WHITE_B_BLACK))     
    else :
        if CHECK_BIT(raw, 0): # FIRST_ALERT
            self._win.addstr(line, pos, "FIRST_ALERT", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos, "FIRST_ALERT", curses.color_pair(F_WHITE_B_BLACK))   
                              
def print_STATUS_MFR_SPECIFIC(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        if CHECK_BIT(raw, 7): # HSC_FET_FAULT_48V_OUTPUT
            self._win.addstr(line, pos, "FET48v", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos, "FET48v", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 6): # HSC_FET_FAULT3
            self._win.addstr(line, pos+7, "FET3", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+7, "FET3", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 5): # HSC_FET_FAULT2
            self._win.addstr(line, pos+12, "FET2", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+12, "FET2", curses.color_pair(F_WHITE_B_BLACK))     
        if CHECK_BIT(raw, 4): # HSC_FET_FAULT1
            self._win.addstr(line, pos+17, "FET1", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+17, "FET1", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 3): # 48V_OUTPUT_FAULT
            self._win.addstr(line, pos+22, "HS48", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+22, "HS48", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 2): # BRICK3_FAULT
            self._win.addstr(line, pos+27, "Br3", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+27, "Br3", curses.color_pair(F_WHITE_B_BLACK))     
        if CHECK_BIT(raw, 1): # BRICK2_FAULT
            self._win.addstr(line, pos+31, "Br2", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+31, "Br2", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 0):  # BRICK1_FAULT
            self._win.addstr(line, pos+35, "Br1", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+35, "Br1", curses.color_pair(F_WHITE_B_BLACK))
    else :
        if CHECK_BIT(raw, 7): # EN_CHANGED
            self._win.addstr(line, pos, "EN_CHG", curses.color_pair(F_YELLOW_B_BLACK))
        else:
            self._win.addstr(line, pos, "EN_CHG", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 6): # Thermal Shutdown
            self._win.addstr(line, pos+7, "THSTDWN", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+7, "THSTDWN", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 5): # Vdd UV Low Limit
            self._win.addstr(line, pos+15, "VDDUVLO", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+15, "VDDUVLO", curses.color_pair(F_WHITE_B_BLACK))     
        if CHECK_BIT(raw, 4): # PIN_OP2_FAULT
            self._win.addstr(line, pos+23, "OP2", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+23, "OP2", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 3): # 48V_OUTPUT_FAULT
            self._win.addstr(line, pos+27, "OP1", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+27, "OP1", curses.color_pair(F_WHITE_B_BLACK))
        if CHECK_BIT(raw, 2): # BRICK3_FAULT
            self._win.addstr(line, pos+31, "FET_BAD", curses.color_pair(F_BLACK_B_RED))
        else:
            self._win.addstr(line, pos+31, "FET_BAD", curses.color_pair(F_WHITE_B_BLACK))     
                       
def print_READ_VIN(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else:
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))
    val = "{:.3f}".format(val)
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/vin_{self._board._timestamp}.csv"

    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)    

def print_READ_IIN(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw  * LTC4286_SENSE_FULL_SCALE) / ((TWO_POWER_FIFTHTEEN - 1) * R_SHUNT_HS)) # Hotswap_DirectDecode_Current
    val = "{:.3f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/iin_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color)
                                                            
def print_READ_VOUT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        mode = self._board.read_byte(VOUT_MODE)
        val = vout_decode(raw, mode)
    else :
        val = ((raw  * LTC4286_SOURCE_FULL_SCALE) / (TWO_POWER_FIFTHTEEN - 1.0))

    EfficiencyPanel.set_vout(EfficiencyPanel, val)

    val = "{:.3f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/vout_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                                      
def print_READ_IOUT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw  * LTC4286_SENSE_FULL_SCALE) / ((TWO_POWER_FIFTHTEEN - 1) * R_SHUNT_HS)) # Hotswap_DirectDecode_Current
    
    EfficiencyPanel.set_iout(EfficiencyPanel, val)

    val = "{:.2f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/iout_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color)
                                                           
def print_READ_TEMPERATURE_1(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = raw + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
    val = "{:.2f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/temp1_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)
                                
def print_READ_TEMPERATURE_2(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = raw + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
    val = "{:.2f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/temp2_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)

def print_READ_TEMPERATURE_3(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = raw + LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS
    val = "{:.2f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/temp3_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)

def print_READ_POUT(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw * LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTHTEEN) / (((TWO_POWER_FIFTHTEEN - 1.0) ** 2) * R_SHUNT_HS))
    val = "{:.1f}".format(val)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/pout_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    EfficiencyPanel.set_pout(EfficiencyPanel, val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)
                                                           
def print_READ_PIN(self, line, pos, raw):
    if not (5 <= self._board._page <= 8): # not hotswap PAGE
        val = linear11_decode(raw)
    else :
        val = ((raw * LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTHTEEN) / (((TWO_POWER_FIFTHTEEN - 1.0) ** 2) * R_SHUNT_HS))

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    csv_file_path = f"/mnt/data/module/{self._board._addr}/pin_{self._board._timestamp}.csv"
    
    # Créez le répertoire ../mnt/data s'il n'existe pas déjà
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Écriture dans le fichier CSV avec un timestamp séparé par une virgule
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow([self._board._page, timestamp, val])

    EfficiencyPanel.set_pin(EfficiencyPanel, val)
    val = "{:.1f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)
                                                         
def print_PMBUS_REVISION(self, line, pos, raw):
    self._win.addstr(line, pos, "Part I: 1." + str(raw >> 4) + " Part II: 1." + str(raw & 0xF), self._board._color)                    
                  
def print_MFR_VIN_MIN(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                                         
def print_MFR_VIN_MAX(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
                                         
def print_MFR_IIN_MAX(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color)
                                                         
def print_MFR_PIN_MAX(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.1f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)
                                                      
def print_MFR_VOUT_MIN(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
  
def print_MFR_VOUT_MAX(self, line, pos, raw):
    mode = self._board.read_byte(VOUT_MODE)
    val = vout_decode(raw, mode)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)
  
def print_MFR_IOUT_MAX(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.2f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color)
                                        
def print_MFR_POUT_MAX(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.1f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "W", self._board._color)
                                                     
def print_MFR_TAMBIENT_MAX(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)
                                  
def print_MFR_TAMBIENT_MIN(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)

def print_MFR_EFFICIENCY_LL(self, line, pos, raw):

    count = int(time.time()) % 49
        # MFR_EFFICIENCY_INPUT_VOLTAGE_LL
    if 0 <= count < 7:
        encoded = (raw[0] + (raw[1] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "V input", self._board._color)

        # MFR_EFFICIENCY_POWER_LOW_LL
    elif 7 <= count < 14:
        encoded = (raw[2] + (raw[3] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.2f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "W low pow", self._board._color) 

        # MFR_EFFICIENCY_LOW_LL
    elif 14 <= count < 21:
        encoded = (raw[4] + (raw[5] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "% low pow", self._board._color)

        # MFR_EFFICIENCY_POWER_MEDIUM_LL
    elif 21 <= count < 28:
        encoded = (raw[6] + (raw[7] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.1f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "W med pow", self._board._color) 

        # MFR_EFFICIENCY_MEDIUM_LL
    elif 28 <= count < 35:
        encoded = (raw[8] + (raw[9] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "% med pow", self._board._color)

        # MFR_EFFICIENCY_POWER_HIGH_LL
    elif 35 <= count < 42:
        encoded = (raw[10] + (raw[11] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.1f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "W hgh pow", self._board._color)

        # MFR_EFFICIENCY_HIGH_LL
    elif 42 <= count < 49:
        encoded = (raw[12] + (raw[13] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "% hgh pow", self._board._color)         
                            
def print_MFR_EFFICIENCY_HL(self, line, pos, raw):
    
    count = int(time.time()) % 49

        # MFR_EFFICIENCY_INPUT_VOLTAGE_HL
    if 0 <= count < 7:
        encoded = (raw[0] + (raw[1] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "V input", self._board._color)

        # MFR_EFFICIENCY_POWER_LOW_HL
    elif 7 <= count < 14:
        encoded = (raw[2] + (raw[3] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.2f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "W low power", self._board._color) 

        # MFR_EFFICIENCY_LOW_HL
    elif 14 <= count < 21:
        encoded = (raw[4] + (raw[5] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "% low pow", self._board._color)

        # MFR_EFFICIENCY_POWER_MEDIUM_HL
    elif 21 <= count < 28:
        encoded = (raw[6] + (raw[7] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.1f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "W med pow", self._board._color) 

        # MFR_EFFICIENCY_MEDIUM_HL
    elif 28 <= count < 35:
        encoded = (raw[8] + (raw[9] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "% med pow", self._board._color)  

        # MFR_EFFICIENCY_POWER_HIGH_HL
    elif 35 <= count < 42:
        encoded = (raw[10] + (raw[11] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.1f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "W hgh pow", self._board._color)  

        # MFR_EFFICIENCY_HIGH_HL
    elif 42 <= count < 49:
        encoded = (raw[12] + (raw[13] << 8))
        val = linear11_decode(encoded)
        self._win.addstr(line, pos, "{:.3f}".format(val), self._board._color)
        self._win.addstr(line, pos + 7, "% hgh pow", self._board._color) 

def print_MFR_MAX_TEMP_1(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)

def print_MFR_MAX_TEMP_2(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)

def print_MFR_MAX_TEMP_3(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "Celsius", self._board._color)

def print_MFR_RESPONSE_UNIT_CFG(self, line, pos, raw):
    if not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT):
        self._win.addstr(line, pos+0, "Vo: 1ms", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT):
        self._win.addstr(line, pos+0, "Vo: 10ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT):
        self._win.addstr(line, pos+0, "Vo: 100ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VOUT):
        self._win.addstr(line, pos+0, "Vo: 1s", curses.color_pair(F_WHITE_B_BLACK))

    if not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN):
        self._win.addstr(line, pos+10, "Vi: 1ms", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN):
        self._win.addstr(line, pos+10, "Vi: 10ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN):
        self._win.addstr(line, pos+10, "Vi: 100ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_VIN):
        self._win.addstr(line, pos+10, "Vi: 1s", curses.color_pair(F_WHITE_B_BLACK))  

    if not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT):
        self._win.addstr(line, pos+20, "Io: 1ms", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT):
        self._win.addstr(line, pos+20, "Io: 10ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT):
        self._win.addstr(line, pos+20, "Io: 100ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_IOUT):
        self._win.addstr(line, pos+20, "Io: 1s", curses.color_pair(F_WHITE_B_BLACK))  

    if not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP):
        self._win.addstr(line, pos+30, "T: 1ms", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP):
        self._win.addstr(line, pos+30, "T: 10ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP + 1) and not CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP):
        self._win.addstr(line, pos+30, "T: 100ms", curses.color_pair(F_WHITE_B_BLACK))
    elif CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP + 1) and CHECK_BIT(raw, MFR_RESPONSE_UNIT_CFG_TEMP):
        self._win.addstr(line, pos+30, "T: 1s", curses.color_pair(F_WHITE_B_BLACK))     

def print_MFR_HW_COMPATIBILITY(self, line, pos, raw):
    self._win.addstr(line, pos, "0x{:02X}".format(raw), self._board._color)
                      
def print_MFR_FWUPLOAD_CAPABILITY(self, line, pos, raw):
    self._win.addstr(line, pos, "0x{:02X}".format(raw), self._board._color)
                   
def print_MFR_FWUPLOAD_MODE(self, line, pos, raw):
    string = items_NO_YES[raw]
    self._win.addstr(line, pos, string, self._board._color)
                                                      
#def print_MFR_FWUPLOAD_STATUS(self, line, pos, raw):
#    self._win.addstr(line, pos, str(raw), self._board._color)
                       
def print_MFR_FWVERSION_NUMBER(self, line, pos, raw):
            self._win.addstr(line, pos, f'{raw}')
                      
def print_MFR_PSU_FW_CRC16_READ(self, line, pos, raw):
    if raw[5] == raw[7] and raw[6] == raw[4] :
        self._win.addstr(line, pos, "Expected: " + "0x{:02X}".format(raw[5]) + "{:02X}".format(raw[4]) + "    Calculated: " + "0x{:02X}".format(raw[7]) + "{:02X}".format(raw[6]), curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos, "Expected: " + "0x{:02X}".format(raw[5]) + "{:02X}".format(raw[4]) + "    Calculated: " + "0x{:02X}".format(raw[7]) + "{:02X}".format(raw[6]), curses.color_pair(F_BLACK_B_RED))
                             
def print_MFR_BRICK_CONTROL(self, line, pos, raw):
    if CHECK_BIT(raw, 5):
        self._win.addstr(line, pos, "Brick_3", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos, "Brick_3", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 4):
        self._win.addstr(line, pos+11, "Brick_2", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+11, "Brick_2", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 3):
        self._win.addstr(line, pos+22, "Brick_1", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+22, "Brick_1", curses.color_pair(F_WHITE_B_BLACK)) 

    if (raw & 0b111) == BRICK_CONTROL_ONE_PLUS_ONE:
        self._win.addstr(line, pos+33, items_MFR_BRICK_CONTROL[0], curses.color_pair(F_BLACK_B_YELLOW))
    elif (raw & 0b111) == BRICK_CONTROL_TWO_PLUS_ONE:
        self._win.addstr(line, pos+33, items_MFR_BRICK_CONTROL[1], curses.color_pair(F_BLACK_B_YELLOW))
                       
def print_MFR_VIN_PG_THRESH(self, line, pos, raw):
    val = (MFR_VIN_PG_THRESH_GAIN * raw) + MFR_VIN_PG_THRESH_OFFSET
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "V", self._board._color)

def print_MFR_ISHARE_THRESHOLD(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.3f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7, "A", self._board._color)    
                     
def print_MFR_POWER_ENABLED(self, line, pos, raw): 
    if CHECK_BIT(raw, 2): # M_3
        self._win.addstr(line, pos, "Brick_3", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos, "Brick_3", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 1): # M_2
        self._win.addstr(line, pos+16, "Brick_2", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+16, "Brick_2", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 0): # M_1
        self._win.addstr(line, pos+31, "Brick_1", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+31, "Brick_1", curses.color_pair(F_WHITE_B_BLACK))  
                                 
def print_MFR_POWER_GOOD(self, line, pos, raw):
    if CHECK_BIT(raw, 2): # PG_M_3
        self._win.addstr(line, pos, "PowerGood_3", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos, "PowerGood_3", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 1): # PG_M_2
        self._win.addstr(line, pos+14, "PowerGood_2", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+14, "PowerGood_2", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 0): # PG_M_1
        self._win.addstr(line, pos+27, "PowerGood_1", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+27, "PowerGood_1", curses.color_pair(F_WHITE_B_BLACK)) 

def print_MFR_BRICK_FAILURE(self, line, pos, raw):

    if CHECK_BIT(raw, 2): # M_3
        self._win.addstr(line, pos, "Brick_3", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "Brick_3", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 1): # M_2
        self._win.addstr(line, pos+16, "Brick_2", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+16, "Brick_2", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 0): # M_1
        self._win.addstr(line, pos+31, "Brick_1", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+31, "Brick_1", curses.color_pair(F_WHITE_B_BLACK))     
                           
def print_MFR_FAULTS_ORING(self, line, pos, raw):
    if CHECK_BIT(raw, 2): # ALERT_OR_3
        self._win.addstr(line, pos, "OR-ing_3", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "OR-ing_3", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # ALERT_OR_2
        self._win.addstr(line, pos + 15, "OR-ing_2", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos + 15, "OR-ing_2", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # ALERT_OR_1
        self._win.addstr(line, pos + 30, "OR-ing_1", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos + 30, "OR-ing_1", curses.color_pair(F_WHITE_B_BLACK))
                           
def print_MFR_PARAMS_DIFF(self, line, pos, raw):
 
    if CHECK_BIT(raw, 1):  # HOTSWAPS
        self._win.addstr(line, pos + 0, "HS3 != HS2 != HS1", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos + 0, "HS3 == HS2 == HS1", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0): # MODULES/BRICKS
        self._win.addstr(line, pos + 21, "M_3 != M_2 != M_1", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos + 21, "M_3 == M_2 == M_1", curses.color_pair(F_WHITE_B_BLACK))

def print_MFR_BUS_STATUS(self, line, pos, raw):
    if CHECK_BIT(raw, 6): # hotswaps[2].alive
        self._win.addstr(line, pos, "HS_3", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos, "HS_3", curses.color_pair(F_BLACK_B_RED))
    if CHECK_BIT(raw, 5): # hotswaps[1].alive
        self._win.addstr(line, pos+5, "HS_2", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+5, "HS_2", curses.color_pair(F_BLACK_B_RED))     
    if CHECK_BIT(raw, 4): # hotswaps[0].alive
        self._win.addstr(line, pos+10, "HS_1", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+10, "HS_1", curses.color_pair(F_BLACK_B_RED))
    if CHECK_BIT(raw, 3): # hotswap_passthrough.alive
        self._win.addstr(line, pos+15, "HS_48v", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+15, "HS_48v", curses.color_pair(F_BLACK_B_RED))
    if CHECK_BIT(raw, 2): # modules[2].alive
        self._win.addstr(line, pos+27, "M_3", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+27, "M_3", curses.color_pair(F_BLACK_B_RED))     
    if CHECK_BIT(raw, 1): # modules[1].alive
        self._win.addstr(line, pos+31, "M_2", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+31, "M_2", curses.color_pair(F_BLACK_B_RED))
    if CHECK_BIT(raw, 0):  # modules[0].alive
        self._win.addstr(line, pos+35, "M_1", curses.color_pair(F_GREEN_B_BLACK))
    else:
        self._win.addstr(line, pos+35, "M_1", curses.color_pair(F_BLACK_B_RED))
                          
def print_MFR_PMBUS_ALERT(self, line, pos, raw):

    if CHECK_BIT(raw, 6): # ALERT_HS_3
        self._win.addstr(line, pos+0, "HS_3", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+0, "HS_3", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 5): # ALERT_HS_2
        self._win.addstr(line, pos+5, "HS_2", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+5, "HS_2", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 4): # ALERT_HS_1
        self._win.addstr(line, pos+10, "HS_1", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+10, "HS_1", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 3): # ALERT_HS_PT
        self._win.addstr(line, pos+15, "HS_48v", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+15, "HS_48v", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 7): # ALERT_TS
        self._win.addstr(line, pos+22, "Temp", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+22, "Temp", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 2): # ALERT_M_3
        self._win.addstr(line, pos+27, "M_3", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+27, "M_3", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 1): # ALERT_M_2
        self._win.addstr(line, pos+31, "M_2", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+31, "M_2", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0):  # ALERT_M_1
        self._win.addstr(line, pos+35, "M_1", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+35, "M_1", curses.color_pair(F_WHITE_B_BLACK))

def print_MFR_BMC_SIGNALS(self, line, pos, raw):
    if CHECK_BIT(raw, 4): # PMBUS_ALERT_L
        self._win.addstr(line, pos, "ALERT", curses.color_pair(F_WHITE_B_BLACK)) 
    else:
        self._win.addstr(line, pos, "ALERT", curses.color_pair(F_BLACK_B_RED)) # active when LOW

    if CHECK_BIT(raw, 3): # PMBUS_12V_OUTPUT_POWER_GOOD
        self._win.addstr(line, pos+7, "12V_OUT", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+7, "12V_OUT", curses.color_pair(F_WHITE_B_BLACK))     

    if CHECK_BIT(raw, 2): # PMBUS_48V_INPUT_POWER_GOOD
        self._win.addstr(line, pos+15, "48V_IN", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+15, "48V_IN", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 1): # PMBUS_48V_OUTPUT_POWER_GOOD
        self._win.addstr(line, pos+22, "48V_OUT", curses.color_pair(F_BLACK_B_GREEN))
    else:
        self._win.addstr(line, pos+22, "48V_OUT", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 0): # PMBUS_48V_OUTPUT_DISABLE_L
        self._win.addstr(line, pos+31, "48V_DIS", curses.color_pair(F_WHITE_B_BLACK))
    else:
        self._win.addstr(line, pos+31, "48V_DIS", curses.color_pair(F_BLACK_B_YELLOW))     
                          
def print_MFR_PMBUS_ADDR(self, line, pos, raw):
    self._win.addstr(line, pos, "0x{:02X}".format(raw), self._board._color)
                                                                                   
def print_MFR_CAL_DATA(self, line, pos, raw):

    count = int(time.time()) % 21

        # SOURCE
    if 0 <= count < 7:
        EncGain   = (raw[0] + (raw[1] << 8))
        EncOffset = (raw[2] + (raw[3] << 8))
        gain = linear11_decode(EncGain)
        offset = linear11_decode(EncOffset)

        self._win.addstr(line, pos, "Source gain: " + "{:.2f}".format(gain), self._board._color)
        self._win.addstr(line, pos + 22, "offset: " + "{:.2f}".format(offset), self._board._color)

        # SENSE
    elif 7 <= count < 14:
        EncGain   = (raw[4] + (raw[5] << 8))
        EncOffset = (raw[6] + (raw[7] << 8))
        gain = linear11_decode(EncGain)
        offset = linear11_decode(EncOffset)
        self._win.addstr(line, pos, "Sense  gain: " + "{:.2f}".format(gain), self._board._color)
        self._win.addstr(line, pos + 22, "offset: " + "{:.2f}".format(offset), self._board._color)

        # POWER
    elif 14 <= count < 21:
        EncGain   = (raw[8] + (raw[9] << 8))
        EncOffset = (raw[10] + (raw[11] << 8))
        gain = linear11_decode(EncGain)
        offset = linear11_decode(EncOffset)
        self._win.addstr(line, pos, "Power  gain: " + "{:.2f}".format(gain), self._board._color)                              
        self._win.addstr(line, pos + 22, "offset: " + "{:.2f}".format(offset), self._board._color)                              

def print_MFR_AUTO_ON(self, line, pos, raw):
    string = items_AUTO_ON[raw]  # auto_on is active low
    self._win.addstr(line, pos, string, self._board._color)
                    
def print_MFR_ENABLE_DIAG(self, line, pos, raw):
    string = items_NO_YES[raw]
    self._win.addstr(line, pos, string, self._board._color)

def print_MFR_DEBUG_STATUS(self, line, pos, raw):
    string = items_NO_YES[raw]
    self._win.addstr(line, pos, string, self._board._color)
      
def print_MFR_SYSTEM_STATUS1(self, line, pos, raw):
    if CHECK_BIT(raw, 15): # ALERT
        self._win.addstr(line, pos, "ALERT", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos, "ALERT", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 14): # L_ALERT
        self._win.addstr(line, pos+6, "L_ALERT", curses.color_pair(F_BLACK_B_RED))
    else:
        self._win.addstr(line, pos+6, "L_ALERT", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 11): # POWER_LOSS : power on reset or reboot-generated reset
        self._win.addstr(line, pos+14, "PON", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+14, "RBT", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 10): # RESET_DONE
        self._win.addstr(line, pos+18, "RST", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+18, "RST", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 7): # AVERAGE_DONE
        self._win.addstr(line, pos+22, "AVR", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+22, "AVR", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 6): # ADC_CONV
        self._win.addstr(line, pos+26, "ADC", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+26, "ADC", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0): # MFR_NONE_OF_ABOVE
        self._win.addstr(line, pos+30, "NONE", curses.color_pair(F_YELLOW_B_BLACK))
    else:
        self._win.addstr(line, pos+30, "NONE", curses.color_pair(F_WHITE_B_BLACK))  

def print_MFR_SYSTEM_STATUS2(self, line, pos, raw):
    if CHECK_BIT(raw, 15): # POWER_FAILED_WARNING
        self._win.addstr(line, pos, "PWR_FAILED", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos, "PWR_FAILED", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 14): # FET_SHORT_WARNING
        self._win.addstr(line, pos+11, "FET_SHORT", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+11, "FET_SHORT", curses.color_pair(F_WHITE_B_BLACK))     
    if CHECK_BIT(raw, 3): # POWER_LOSS : power on reset or reboot-generated reset
        self._win.addstr(line, pos+21, "UV", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+21, "UV", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 2): # RESET_DONE
        self._win.addstr(line, pos+24, "OV", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+24, "OV", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 1): # AVERAGE_DONE
        self._win.addstr(line, pos+27, "UC", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+27, "UC", curses.color_pair(F_WHITE_B_BLACK))
    if CHECK_BIT(raw, 0): # ADC_CONV
        self._win.addstr(line, pos+30, "UP", curses.color_pair(F_BLACK_B_YELLOW))
    else:
        self._win.addstr(line, pos+30, "UP", curses.color_pair(F_WHITE_B_BLACK))

def print_MFR_CONFIG1(self, line, pos, raw):
    self._win.addstr(line, pos, "Vsns:", curses.color_pair(F_WHITE_B_BLACK))
    if   not CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, " 6", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, " 7", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, " 8", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, " 9", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "10", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "11", curses.color_pair(F_WHITE_B_BLACK))
    elif not CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "12", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "13", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "14", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "15", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and not CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "16", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "17", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and not CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "18", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and not CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "19", curses.color_pair(F_WHITE_B_BLACK))
    elif     CHECK_BIT(raw, 13) and     CHECK_BIT(raw, 12) and     CHECK_BIT(raw, 11) and     CHECK_BIT(raw, 10): self._win.addstr(line, pos+6, "20", curses.color_pair(F_WHITE_B_BLACK))
    else : self._win.addstr(line, pos+6, "??", curses.color_pair(F_WHITE_B_BLACK))

    self._win.addstr(line, pos+9, "mV", curses.color_pair(F_WHITE_B_BLACK))
    self._win.addstr(line, pos+13, "Range:", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 1): # VRANGE_SELECT
        self._win.addstr(line, pos+20, "102.4V", curses.color_pair(F_WHITE_B_BLACK))
    else:
        self._win.addstr(line, pos+20, "25.6V", curses.color_pair(F_WHITE_B_BLACK))

    self._win.addstr(line, pos+27, "VPwMx:", curses.color_pair(F_WHITE_B_BLACK))

    if CHECK_BIT(raw, 0): # VPWR_SELECT
        self._win.addstr(line, pos+34, "Vout", curses.color_pair(F_WHITE_B_BLACK))
    else:
        self._win.addstr(line, pos+34, "Vin", curses.color_pair(F_WHITE_B_BLACK))

def print_MFR_ADDED_DROOP_DURING_RAMP(self, line, pos, raw):
    val = linear11_decode(raw)
    val = "{:.4f}".format(val)
    self._win.addstr(line, pos, str(val), self._board._color)
    self._win.addstr(line, pos + 7,  "mV/A", self._board._color)

###################################################### ACTION CALLBACK FOR WRITE COMMANDS ##########################################################

def action_PAGE(self, item):
    #get current item displayed
    for index, item_page in enumerate(items_PAGE):
        if item == item_page:
            try:
                self._pmbus.write_byte(PAGE, index)
            except:
                pass

def action_OPERATION(self, item):        
    if item == items_OPERATION[0]:
        self._pmbus.write_byte(OPERATION, OPERATION_IMMEDIATE_OFF)
    elif item == items_OPERATION[1]:
        self._pmbus.write_byte(OPERATION, OPERATION_SOFT_OFF)
    elif item == items_OPERATION[2]:
        self._pmbus.write_byte(OPERATION, OPERATION_ON_MARGIN_OFF)
    elif item == items_OPERATION[3]:
        self._pmbus.write_byte(OPERATION, OPERATION_ON_MARGIN_LOW_IGNORE_FAULT)   
    elif item == items_OPERATION[4]:
        self._pmbus.write_byte(OPERATION, OPERATION_ON_MARGIN_LOW_ACT_ON_FAULT)
    elif item == items_OPERATION[5]:
        self._pmbus.write_byte(OPERATION, OPERATION_ON_MARGIN_HIGH_IGNORE_FAULT)
    elif item == items_OPERATION[6]:
        self._pmbus.write_byte(OPERATION, OPERATION_ON_MARGIN_HIGH_ACT_ON_FAULT) 

def action_ON_OFF_CONFIG(self, item):        
    if item == items_ON_OFF_CONFIG[0]:
        self._pmbus.write_byte(ON_OFF_CONFIG, ON_OFF_CONFIG_IGNORE_ALL)
    elif item == items_ON_OFF_CONFIG[1]:
        self._pmbus.write_byte(ON_OFF_CONFIG, ON_OFF_CONFIG_PMBUS_ONLY)
    elif item == items_ON_OFF_CONFIG[2]:
        self._pmbus.write_byte(ON_OFF_CONFIG, ON_OFF_CONFIG_RC_ONLY)
    elif item == items_ON_OFF_CONFIG[3]:
        self._pmbus.write_byte(ON_OFF_CONFIG, ON_OFF_CONFIG_PMBUS_RC)   

def action_CLEAR_FAULTS(self, item):       
    self._pmbus.write_byte(CLEAR_FAULTS)

def action_WRITE_PROTECT(self, item):           
    #get current item displayed
    if item == items_WRITE_PROTECT[0]:
        self._pmbus.write_byte(WRITE_PROTECT, WP_DISABLE_OTHER)
    elif item == items_WRITE_PROTECT[1]:
        self._pmbus.write_byte(WRITE_PROTECT, WP_EN_WP_OP_PG)
    elif item == items_WRITE_PROTECT[2]:
        self._pmbus.write_byte(WRITE_PROTECT, WP_EN_WP_OP_PG_OFC_VOUTCMD)
    elif item == items_WRITE_PROTECT[3]:
        self._pmbus.write_byte(WRITE_PROTECT, WP_EN_ALL)

def action_RESTORE_DEFAULT_ALL(self, item):       
    self._pmbus.write_byte(RESTORE_DEFAULT_ALL)

def action_STORE_USER_ALL(self, item):       
    self._pmbus.write_byte(STORE_USER_ALL)

def action_RESTORE_USER_ALL(self, item):       
    self._pmbus.write_byte(RESTORE_USER_ALL)

def action_QUERY(self, item):                  
    if item == items_QUERY[0]:
        self._pmbus.write_byte(QUERY, OPERATION)
    elif item == items_QUERY[1]:                             
        self._pmbus.write_byte(QUERY, VOUT_MODE)
    elif item == items_QUERY[2]:
        self._pmbus.write_byte(QUERY, VOUT_COMMAND)
    elif item == items_QUERY[3]:
        self._pmbus.write_byte(QUERY, VOUT_OV_FAULT_LIMIT)
    elif item == items_QUERY[4]:
        self._pmbus.write_byte(QUERY, OT_FAULT_RESPONSE)
    elif item == items_QUERY[5]:
        self._pmbus.write_byte(QUERY, TON_DELAY)
    elif item == items_QUERY[6]:
        self._pmbus.write_byte(QUERY, STATUS_BYTE)
    elif item == items_QUERY[7]:
        self._pmbus.write_byte(QUERY, STATUS_WORD)
    elif item == items_QUERY[8]:
        self._pmbus.write_byte(QUERY, READ_VIN)
    elif item == items_QUERY[9]:
        self._pmbus.write_byte(QUERY, MFR_EFFICIENCY_LL)
    elif item == items_QUERY[10]:
        self._pmbus.write_byte(QUERY, MFR_PSU_POWER_CYCLE)   
    elif item == items_QUERY[11]:
        self._pmbus.write_byte(QUERY, MFR_FWVERSION_NUMBER)
    elif item == items_QUERY[12]:
        self._pmbus.write_byte(QUERY, MFR_BRICK_CONTROL)        

def action_VOUT_COMMAND(self, item):
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(VOUT_COMMAND, val)
    except:
        pass

def action_VOUT_MAX(self, item):           
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(VOUT_MAX, val)   
    except:
        pass

def action_VOUT_MARGIN_HIGH(self, item):    
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(VOUT_MARGIN_HIGH, val)
    except:
        pass

def action_VOUT_MARGIN_LOW(self, item):  
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(VOUT_MARGIN_LOW, val)
    except:
        pass

def action_VOUT_TRANSITION_RATE(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(VOUT_TRANSITION_RATE, val)

def action_VOUT_DROOP(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(VOUT_DROOP, val)

def action_VOUT_OV_FAULT_LIMIT(self, item):  
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(VOUT_OV_FAULT_LIMIT, val)
    except:
        pass

def action_VOUT_OV_FAULT_RESPONSE(self, item):
    response = get_response_hex(item) 
    self._pmbus.write_byte(VOUT_OV_FAULT_RESPONSE, response)

def action_VOUT_OV_WARN_LIMIT(self, item):  
    try:       
        if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
            mode = self._pmbus.read_byte(VOUT_MODE)
            val = vout_encode(item, mode)
        else :
            val = int((item  * (TWO_POWER_FIFTHTEEN - 1.0)) / LTC4286_SOURCE_FULL_SCALE) # Hotswap_EncodeDirect_Voltage     
        self._pmbus.write_word(VOUT_OV_WARN_LIMIT, val)
    except:
        pass

def action_VOUT_UV_WARN_LIMIT(self, item):  
    try:   
        if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
            mode = self._pmbus.read_byte(VOUT_MODE)
            val = vout_encode(item, mode)
        else :
            val = int((item  * (TWO_POWER_FIFTHTEEN - 1.0)) / LTC4286_SOURCE_FULL_SCALE) # Hotswap_EncodeDirect_Voltage     
        self._pmbus.write_word(VOUT_UV_WARN_LIMIT, val)
    except:
        pass

def action_VOUT_UV_FAULT_LIMIT(self, item):  
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(VOUT_UV_FAULT_LIMIT, val)
    except:
        pass
    
def action_VOUT_UV_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(VOUT_UV_FAULT_RESPONSE, response)

def action_IOUT_OC_FAULT_LIMIT(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(IOUT_OC_FAULT_LIMIT, val)    

def action_IOUT_OC_FAULT_RESPONSE(self, item): 
    response = get_current_response_hex(item) 
    self._pmbus.write_byte(IOUT_OC_FAULT_RESPONSE, response)

def action_IOUT_OC_LV_FAULT_LIMIT(self, item):  
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(IOUT_OC_LV_FAULT_LIMIT, val)
    except:
        pass

def action_IOUT_OC_WARN_LIMIT(self, item):
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item)
    else :
        val = int((item * ((TWO_POWER_FIFTHTEEN - 1.0) * R_SHUNT_HS)) / LTC4286_SENSE_FULL_SCALE)  # Hotswap_EncodeDirect_Current         
    self._pmbus.write_word(IOUT_OC_WARN_LIMIT, val) 

def action_IOUT_UC_FAULT_LIMIT(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(IOUT_UC_FAULT_LIMIT, val) 

def action_IOUT_UC_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(IOUT_UC_FAULT_RESPONSE, response)

def action_OT_FAULT_LIMIT(self, item): 
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int(item - LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS) # Hotswap_EncodeDirect_Temperature    
    self._pmbus.write_word(OT_FAULT_LIMIT, val)
    
def action_OT_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(OT_FAULT_RESPONSE, response)

def action_OT_WARN_LIMIT(self, item): 
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int(item - LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS) # Hotswap_EncodeDirect_Temperature  
    self._pmbus.write_word(OT_WARN_LIMIT, val)

def action_UT_WARN_LIMIT(self, item): 
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int(item - LTC4286_TEMP_CONSTANT_DIRECT_CELCIUS) # Hotswap_EncodeDirect_Temperature  
    self._pmbus.write_word(UT_WARN_LIMIT, val)

def action_UT_FAULT_LIMIT(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(UT_FAULT_LIMIT, val)

def action_UT_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(UT_FAULT_RESPONSE, response)

def action_VIN_OV_FAULT_LIMIT(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(VIN_OV_FAULT_LIMIT, val)

def action_VIN_OV_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(VIN_OV_FAULT_RESPONSE, response)

def action_VIN_OV_WARN_LIMIT(self, item): 
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int((item  * (TWO_POWER_FIFTHTEEN - 1.0)) / LTC4286_SOURCE_FULL_SCALE) # Hotswap_EncodeDirect_Voltage     
    self._pmbus.write_word(VIN_OV_WARN_LIMIT, val)

def action_VIN_UV_WARN_LIMIT(self, item): 
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int((item  * (TWO_POWER_FIFTHTEEN - 1.0)) / LTC4286_SOURCE_FULL_SCALE) # Hotswap_EncodeDirect_Voltage     
    self._pmbus.write_word(VIN_UV_WARN_LIMIT, val)

def action_VIN_UV_FAULT_LIMIT(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(VIN_UV_FAULT_LIMIT, val)

def action_VIN_UV_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(VIN_UV_FAULT_RESPONSE, response)

def action_IIN_OC_FAULT_LIMIT(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(IIN_OC_FAULT_LIMIT, val)

def action_IIN_OC_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(IIN_OC_FAULT_RESPONSE, response)

def action_IIN_OC_WARN_LIMIT(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(IIN_OC_WARN_LIMIT, val)

def action_POWER_GOOD_ON(self, item):
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(POWER_GOOD_ON, val)
    except:
        pass

def action_POWER_GOOD_OFF(self, item):
    try:       
        mode = self._pmbus.read_byte(VOUT_MODE)
        val = vout_encode(item, mode)
        self._pmbus.write_word(POWER_GOOD_OFF, val)
    except:
        pass

def action_TON_DELAY(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(TON_DELAY, val)
          
def action_TON_RISE(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(TON_RISE, val)
           
def action_TON_MAX_FAULT_LIMIT(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(TON_MAX_FAULT_LIMIT, val)

def action_TON_MAX_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(TON_MAX_FAULT_RESPONSE, response)

def action_TOFF_DELAY(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(TOFF_DELAY, val)

def action_TOFF_FALL(self, item):                    
    val = linear11_encode(item) 
    self._pmbus.write_word(TOFF_FALL, val)
              
def action_TOFF_MAX_WARN_LIMIT(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(TOFF_MAX_WARN_LIMIT, val)

def action_POUT_OP_FAULT_LIMIT(self, item):          
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int((item * ((((TWO_POWER_FIFTHTEEN - 1.0) ** 2)) * R_SHUNT_HS)) / (LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTHTEEN)) # Hotswap_EncodeDirect_Power     
    self._pmbus.write_word(POUT_OP_FAULT_LIMIT, val)

def action_POUT_OP_FAULT_RESPONSE(self, item): 
    response = get_response_hex(item) 
    self._pmbus.write_byte(POUT_OP_FAULT_RESPONSE, response)

def action_POUT_OP_WARN_LIMIT(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(POUT_OP_WARN_LIMIT, val)

def action_PIN_OP_WARN_LIMIT(self, item):          
    if not (5 <= self._pmbus._page <= 8): # not hotswap PAGE
        val = linear11_encode(item) 
    else :
        val = int((item * ((((TWO_POWER_FIFTHTEEN - 1.0) ** 2)) * R_SHUNT_HS)) / (LTC4286_SENSE_FULL_SCALE * LTC4286_SOURCE_FULL_SCALE * TWO_POWER_FIFTHTEEN)) # Hotswap_EncodeDirect_Power     
    self._pmbus.write_word(PIN_OP_WARN_LIMIT, val) 

def action_MFR_RESPONSE_UNIT_CFG(self, item):        
    if item == items_MFR_RESPONSE_UNIT_CFG[0]:
        self._pmbus.write_byte(MFR_RESPONSE_UNIT_CFG, MFR_RESPONSE_UNIT_CFG_1MS)
    elif item == items_MFR_RESPONSE_UNIT_CFG[1]:
        self._pmbus.write_byte(MFR_RESPONSE_UNIT_CFG, MFR_RESPONSE_UNIT_CFG_10MS)
    elif item == items_MFR_RESPONSE_UNIT_CFG[2]:
        self._pmbus.write_byte(MFR_RESPONSE_UNIT_CFG, MFR_RESPONSE_UNIT_CFG_100MS)
    elif item == items_MFR_RESPONSE_UNIT_CFG[3]:
        self._pmbus.write_byte(MFR_RESPONSE_UNIT_CFG, MFR_RESPONSE_UNIT_CFG_1S) 

def action_MFR_PSU_REBOOT(self, item):                         
    data = None
    if item == items_MFR_PSU_REBOOT[0]:
        data = DEV_UNLOCK_CODE + [0x88, 0x13]
    elif item == items_MFR_PSU_REBOOT[1]:
        data = DEV_UNLOCK_CODE + [0x10, 0x27]
    elif item == items_MFR_PSU_REBOOT[2]:
        data = DEV_UNLOCK_CODE + [0x20, 0x4E]        
    elif item == items_MFR_PSU_REBOOT[3]:
        data = DEV_UNLOCK_CODE + [0x30, 0x75]
    elif item == items_MFR_PSU_REBOOT[4]:
        data = DEV_UNLOCK_CODE + [0x40, 0x9C]
    elif item == items_MFR_PSU_REBOOT[5]:
        data = DEV_UNLOCK_CODE + [0x50, 0xC3]
    elif item == items_MFR_PSU_REBOOT[6]:
        data = DEV_UNLOCK_CODE + [0x60, 0xEA]
    else: return False

    self._pmbus.write_block(MFR_PSU_REBOOT, data)

def action_MFR_FWUPLOAD_MODE(self, item):
    self._pmbus.write_byte(MFR_FWUPLOAD_MODE, 0x01 if item == "Yes" else 0x00)
           
def action_MFR_PSU_POWER_CYCLE(self, item): 
    self._pmbus.write_byte(MFR_PSU_POWER_CYCLE, item)
      
def action_MFR_BRICK_CONTROL(self, item):  
    if item == items_MFR_BRICK_CONTROL[0]:
        self._pmbus.write_byte(MFR_BRICK_CONTROL, BRICK_CONTROL_ONE_PLUS_ONE)
    elif item == items_MFR_BRICK_CONTROL[1]:
        self._pmbus.write_byte(MFR_BRICK_CONTROL, BRICK_CONTROL_TWO_PLUS_ONE)

def action_MFR_VIN_PG_THRESH(self, item): 
    data = int((item - MFR_VIN_PG_THRESH_OFFSET) / MFR_VIN_PG_THRESH_GAIN)
    self._pmbus.write_byte(MFR_VIN_PG_THRESH, data)

def action_MFR_ISHARE_THRESHOLD(self, item):          
    val = linear11_encode(item) 
    self._pmbus.write_word(MFR_ISHARE_THRESHOLD, val)

def action_MFR_POWER_ENABLED(self, item):                            
    if item == items_MFR_POWER_ENABLED[0]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110000) # All OFF"
    elif item == items_MFR_POWER_ENABLED[1]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110111) # "All On"
    elif item == items_MFR_POWER_ENABLED[2]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110001) # "Only n°1"
    elif item == items_MFR_POWER_ENABLED[3]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110010) # "Only n°2" 
    elif item == items_MFR_POWER_ENABLED[4]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110100) # "Only n°3" 
    elif item == items_MFR_POWER_ENABLED[5]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110101) # "1&3"
    elif item == items_MFR_POWER_ENABLED[6]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110011) # "1&2"
    elif item == items_MFR_POWER_ENABLED[7]:
        self._pmbus.write_byte(MFR_POWER_ENABLED, 0b11110110) # "2&3"

def action_MFR_PMBUS_ADDR(self, item):    
    self._pmbus.write_byte(MFR_PMBUS_ADDR, item)
          
def action_MFR_DEBUG_STATUS(self, item):  
    self._pmbus.write_byte(MFR_DEBUG_STATUS, 0x01 if item == "Yes" else 0x00)

def action_MFR_AUTO_ON(self, item):         
    self._pmbus.write_byte(MFR_AUTO_ON, 0x01 if item == "NO auto on" else 0x00)

def action_MFR_CAL_DATA(self, item): 
    data = []

    if item == items_MFR_CAL_DATA[0]:
        parameters = MFR_CAL_DATA_CAL1
        
    elif item == items_MFR_CAL_DATA[1]:
        parameters = MFR_CAL_DATA_CAL2

    elif item == items_MFR_CAL_DATA[2]:
        parameters = MFR_CAL_DATA_CAL3

    for cal in parameters:
        encoded = linear11_encode(cal)
        encodedL = encoded & 0xFF
        encodedH = encoded >> 8

        data.append(encodedL)
        data.append(encodedH)

    self._pmbus.write_block(MFR_CAL_DATA, data)

def action_MFR_ENABLE_DIAG(self, item): 
    self._pmbus.write_byte(MFR_ENABLE_DIAG, 0x01 if item == "Yes" else 0x00)

def action_MFR_CONFIG1(self, item):
    if   item == items_MFR_CONFIG1[0]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_06)
    elif item == items_MFR_CONFIG1[1]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_07)
    elif item == items_MFR_CONFIG1[2]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_08)
    elif item == items_MFR_CONFIG1[3]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_09)   
    elif item == items_MFR_CONFIG1[4]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_10)
    elif item == items_MFR_CONFIG1[5]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_11)
    elif item == items_MFR_CONFIG1[6]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_12) 
    elif item == items_MFR_CONFIG1[7]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_13) 
    elif item == items_MFR_CONFIG1[8]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_14)
    elif item == items_MFR_CONFIG1[9]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_15)
    elif item == items_MFR_CONFIG1[10]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_16)
    elif item == items_MFR_CONFIG1[11]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_17)   
    elif item == items_MFR_CONFIG1[12]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_18)
    elif item == items_MFR_CONFIG1[13]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_19)
    elif item == items_MFR_CONFIG1[14]:
        self._pmbus.write_word(MFR_CONFIG1, MFR_CONFIG_1_ILIM_20) 

def action_MFR_ADDED_DROOP_DURING_RAMP(self, item): 
    val = linear11_encode(item) 
    self._pmbus.write_word(MFR_ADDED_DROOP_DURING_RAMP, val)

######################################################             PMBUS COMMANDS                     ##########################################################


pmbus_commands = [
    { 'command': 'PAGE',                        'read': PAGE,                           'write': PAGE,                          'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nPAGE,\nthen press ENTER",                         'print': print_PAGE,                            'action': action_PAGE,                          'items': items_PAGE,                        'slaves': SLAVE_ALL,        },  
    { 'command': 'OPERATION',                   'read': OPERATION,                      'write': OPERATION,                     'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nOPERATION,\nthen press ENTER",                    'print': print_OPERATION,                       'action': action_OPERATION,                     'items': items_OPERATION,                   'slaves': SLAVE_ALL,        }, 
    { 'command': 'ON_OFF_CONFIG',               'read': ON_OFF_CONFIG,                  'write': ON_OFF_CONFIG,                 'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nON_OFF_CONFIG,\nthen press ENTER",                'print': print_ON_OFF_CONFIG,                   'action': action_ON_OFF_CONFIG,                 'items': items_ON_OFF_CONFIG,               'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'CLEAR_FAULTS',                                                        'write': CLEAR_FAULTS,                  'type' : TYPE_ZERO,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER\nto clear faults",                                                                                              'action': action_CLEAR_FAULTS,                                                              'slaves': SLAVE_ALL,        },     
    { 'command': 'WRITE_PROTECT',               'read': WRITE_PROTECT,                  'write': WRITE_PROTECT,                 'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nWRITE_PROTECT,\nthen press ENTER",                'print': print_WRITE_PROTECT,                   'action': action_WRITE_PROTECT,                 'items': items_WRITE_PROTECT,               'slaves': SLAVE_ALL,        },     
    { 'command': 'RESTORE_DEFAULT_ALL',                                                 'write': RESTORE_DEFAULT_ALL,           'type' : TYPE_ZERO,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER\nto restore all \ndefault values",                                                                              'action': action_RESTORE_DEFAULT_ALL,                                                       'slaves': SLAVE_MCU_Mx,     },     
    { 'command': 'STORE_USER_ALL',                                                      'write': STORE_USER_ALL,                'type' : TYPE_ZERO,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER\nto store all \nuser values",                                                                                   'action': action_STORE_USER_ALL,                                                            'slaves': SLAVE_MCU_Mx,     },     
    { 'command': 'RESTORE_USER_ALL',                                                    'write': RESTORE_USER_ALL,              'type' : TYPE_ZERO,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER\nto restore all \nuser values",                                                                                 'action': action_RESTORE_USER_ALL,                                                          'slaves': SLAVE_MCU_Mx,     },     
    { 'command': 'CAPABILITY',                  'read': CAPABILITY,                                                             'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_CAPABILITY,                                                                                                                  'slaves': SLAVE_ALL,        }, 
    { 'command': 'QUERY',                       'read': QUERY,                          'write': QUERY,                         'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nQUERY,\nthen press ENTER",                        'print': print_QUERY,                           'action': action_QUERY,                         'items': items_QUERY,                       'slaves': SLAVE_MCU,        },     
    { 'command': 'VOUT_MODE',                   'read': VOUT_MODE,                                                              'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_VOUT_MODE,                                                                                                                   'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_COMMAND',                'read': VOUT_COMMAND,                   'write': VOUT_COMMAND,                  'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_COMMAND,\nthen press ENTER",                 'print': print_VOUT_COMMAND,                    'action': action_VOUT_COMMAND,                  'items': items_VOUT_COMMAND,                'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_MAX',                    'read': VOUT_MAX,                       'write': VOUT_MAX,                      'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_MAX,\nthen press ENTER",                     'print': print_VOUT_MAX,                        'action': action_VOUT_MAX,                      'items': items_VOUT_MAX,                    'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_MARGIN_HIGH',            'read': VOUT_MARGIN_HIGH,               'write': VOUT_MARGIN_HIGH,              'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_MARGIN_HIGH,\nthen press ENTER",             'print': print_VOUT_MARGIN_HIGH,                'action': action_VOUT_MARGIN_HIGH,              'items': items_VOUT_MARGIN_HIGH,            'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_MARGIN_LOW',             'read': VOUT_MARGIN_LOW,                'write': VOUT_MARGIN_LOW,               'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_MARGIN_LOW,\nthen press ENTER",              'print': print_VOUT_MARGIN_LOW,                 'action': action_VOUT_MARGIN_LOW,               'items': items_VOUT_MARGIN_LOW,             'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_TRANSITION_RATE',        'read': VOUT_TRANSITION_RATE,           'write': VOUT_TRANSITION_RATE,          'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V/ms', 'helper': "Press <- OR -> to change\nVOUT_TRANSITION_RATE,\nthen press ENTER",         'print': print_VOUT_TRANSITION_RATE,            'action': action_VOUT_TRANSITION_RATE,          'items': items_VOUT_TRANSITION_RATE,        'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_DROOP',                  'read': VOUT_DROOP,                     'write': VOUT_DROOP,                    'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'mV/A', 'helper': "Press <- OR -> to change\nVOUT_DROOP,\nthen press ENTER",                   'print': print_VOUT_DROOP,                      'action': action_VOUT_DROOP,                    'items': items_VOUT_DROOP,                  'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'VOUT_OV_FAULT_LIMIT',         'read': VOUT_OV_FAULT_LIMIT   ,         'write': VOUT_OV_FAULT_LIMIT   ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_OV_FAULT_LIMIT\nthen press ENTER",           'print': print_VOUT_OV_FAULT_LIMIT   ,          'action': action_VOUT_OV_FAULT_LIMIT   ,        'items': items_VOUT_OV_FAULT_LIMIT,         'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'VOUT_OV_FAULT_RESPONSE',      'read': VOUT_OV_FAULT_RESPONSE,         'write': VOUT_OV_FAULT_RESPONSE,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nVOUT_OV_FAULT_RESPONSE\nthen press ENTER",        'print': print_RESPONSE,                        'action': action_VOUT_OV_FAULT_RESPONSE,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'VOUT_OV_WARN_LIMIT',          'read': VOUT_OV_WARN_LIMIT    ,         'write': VOUT_OV_WARN_LIMIT    ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_OV_WARN_LIMIT\nthen press ENTER",            'print': print_VOUT_OV_WARN_LIMIT    ,          'action': action_VOUT_OV_WARN_LIMIT    ,        'items': items_VOUT_OV_WARN_LIMIT,          'slaves': SLAVE_ALL,        },
    { 'command': 'VOUT_UV_WARN_LIMIT',          'read': VOUT_UV_WARN_LIMIT    ,         'write': VOUT_UV_WARN_LIMIT    ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_UV_WARN_LIMIT\nthen press ENTER",            'print': print_VOUT_UV_WARN_LIMIT    ,          'action': action_VOUT_UV_WARN_LIMIT    ,        'items': items_VOUT_UV_WARN_LIMIT,          'slaves': SLAVE_ALL,        },
    { 'command': 'VOUT_UV_FAULT_LIMIT',         'read': VOUT_UV_FAULT_LIMIT   ,         'write': VOUT_UV_FAULT_LIMIT   ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVOUT_UV_FAULT_LIMIT\nthen press ENTER",           'print': print_VOUT_UV_FAULT_LIMIT   ,          'action': action_VOUT_UV_FAULT_LIMIT   ,        'items': items_VOUT_UV_FAULT_LIMIT,         'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'VOUT_UV_FAULT_RESPONSE',      'read': VOUT_UV_FAULT_RESPONSE,         'write': VOUT_UV_FAULT_RESPONSE,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nVOUT_UV_FAULT_RESPONSE\nthen press ENTER",        'print': print_RESPONSE,                        'action': action_VOUT_UV_FAULT_RESPONSE,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'IOUT_OC_FAULT_LIMIT',         'read': IOUT_OC_FAULT_LIMIT   ,         'write': IOUT_OC_FAULT_LIMIT   ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',    'helper': "Press <- OR -> to change\nIOUT_OC_FAULT_LIMIT\nthen press ENTER",           'print': print_IOUT_OC_FAULT_LIMIT   ,          'action': action_IOUT_OC_FAULT_LIMIT   ,        'items': items_IOUT_OC_FAULT_LIMIT,         'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'IOUT_OC_FAULT_RESPONSE',      'read': IOUT_OC_FAULT_RESPONSE,         'write': IOUT_OC_FAULT_RESPONSE,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nIOUT_OC_FAULT_RESPONSE\nthen press ENTER",        'print': print_Current_RESPONSE,                'action': action_IOUT_OC_FAULT_RESPONSE,        'items': items_Current_RESPONSE,            'slaves': SLAVE_ALL,        },
    { 'command': 'IOUT_OC_LV_FAULT_LIMIT',      'read': IOUT_OC_LV_FAULT_LIMIT,         'write': IOUT_OC_LV_FAULT_LIMIT,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nIOUT_OC_LV_FAULT_LIMIT\nthen press ENTER",        'print': print_IOUT_OC_LV_FAULT_LIMIT,          'action': action_IOUT_OC_LV_FAULT_LIMIT,        'items': items_IOUT_OC_LV_FAULT_LIMIT,      'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'IOUT_OC_WARN_LIMIT',          'read': IOUT_OC_WARN_LIMIT    ,         'write': IOUT_OC_WARN_LIMIT    ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',    'helper': "Press <- OR -> to change\nIOUT_OC_WARN_LIMIT\nthen press ENTER",            'print': print_IOUT_OC_WARN_LIMIT    ,          'action': action_IOUT_OC_WARN_LIMIT    ,        'items': items_IOUT_OC_WARN_LIMIT,          'slaves': SLAVE_ALL,        },
    { 'command': 'IOUT_UC_FAULT_LIMIT',         'read': IOUT_UC_FAULT_LIMIT   ,         'write': IOUT_UC_FAULT_LIMIT   ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',    'helper': "Press <- OR -> to change\nIOUT_UC_FAULT_LIMIT\nthen press ENTER",           'print': print_IOUT_UC_FAULT_LIMIT   ,          'action': action_IOUT_UC_FAULT_LIMIT   ,        'items': items_IOUT_UC_FAULT_LIMIT,         'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'IOUT_UC_FAULT_RESPONSE',      'read': IOUT_UC_FAULT_RESPONSE,         'write': IOUT_UC_FAULT_RESPONSE,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nIOUT_UC_FAULT_RESPONSE\nthen press ENTER",        'print': print_RESPONSE,                        'action': action_IOUT_UC_FAULT_RESPONSE,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'OT_FAULT_LIMIT',              'read': OT_FAULT_LIMIT        ,         'write': OT_FAULT_LIMIT        ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'C',    'helper': "Press <- OR -> to change\nOT_FAULT_LIMIT\nthen press ENTER",                'print': print_OT_FAULT_LIMIT        ,          'action': action_OT_FAULT_LIMIT        ,        'items': items_OT_FAULT_LIMIT,              'slaves': SLAVE_ALL,        },
    { 'command': 'OT_FAULT_RESPONSE',           'read': OT_FAULT_RESPONSE     ,         'write': OT_FAULT_RESPONSE     ,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nOT_FAULT_RESPONSE\nthen press ENTER",             'print': print_RESPONSE,                        'action': action_OT_FAULT_RESPONSE     ,        'items': items_RESPONSE,                    'slaves': SLAVE_ALL,        },
    { 'command': 'OT_WARN_LIMIT',               'read': OT_WARN_LIMIT         ,         'write': OT_WARN_LIMIT         ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'C',    'helper': "Press <- OR -> to change\nOT_WARN_LIMIT\nthen press ENTER",                 'print': print_OT_WARN_LIMIT         ,          'action': action_OT_WARN_LIMIT         ,        'items': items_OT_WARN_LIMIT,               'slaves': SLAVE_ALL,        },
    { 'command': 'UT_WARN_LIMIT',               'read': UT_WARN_LIMIT         ,         'write': UT_WARN_LIMIT         ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'C',    'helper': "Press <- OR -> to change\nUT_WARN_LIMIT\nthen press ENTER",                 'print': print_UT_WARN_LIMIT         ,          'action': action_UT_WARN_LIMIT         ,        'items': items_UT_WARN_LIMIT,               'slaves': SLAVE_ALL,        },
    { 'command': 'UT_FAULT_LIMIT',              'read': UT_FAULT_LIMIT        ,         'write': UT_FAULT_LIMIT        ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'C',    'helper': "Press <- OR -> to change\nUT_FAULT_LIMIT\nthen press ENTER",                'print': print_UT_FAULT_LIMIT        ,          'action': action_UT_FAULT_LIMIT        ,        'items': items_UT_FAULT_LIMIT,              'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'UT_FAULT_RESPONSE',           'read': UT_FAULT_RESPONSE     ,         'write': UT_FAULT_RESPONSE     ,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nUT_FAULT_RESPONSE\nthen press ENTER",             'print': print_RESPONSE,                        'action': action_UT_FAULT_RESPONSE     ,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'VIN_OV_FAULT_LIMIT',          'read': VIN_OV_FAULT_LIMIT    ,         'write': VIN_OV_FAULT_LIMIT    ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVIN_OV_FAULT_LIMIT\nthen press ENTER",            'print': print_VIN_OV_FAULT_LIMIT    ,          'action': action_VIN_OV_FAULT_LIMIT    ,        'items': items_VIN_OV_FAULT_LIMIT,          'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'VIN_OV_FAULT_RESPONSE',       'read': VIN_OV_FAULT_RESPONSE ,         'write': VIN_OV_FAULT_RESPONSE ,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nVIN_OV_FAULT_RESPONSE\nthen press ENTER",         'print': print_RESPONSE,                        'action': action_VIN_OV_FAULT_RESPONSE ,        'items': items_RESPONSE,                    'slaves': SLAVE_ALL,        },
    { 'command': 'VIN_OV_WARN_LIMIT',           'read': VIN_OV_WARN_LIMIT     ,         'write': VIN_OV_WARN_LIMIT     ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVIN_OV_WARN_LIMIT\nthen press ENTER",             'print': print_VIN_OV_WARN_LIMIT     ,          'action': action_VIN_OV_WARN_LIMIT     ,        'items': items_VIN_OV_WARN_LIMIT,           'slaves': SLAVE_ALL,        },
    { 'command': 'VIN_UV_WARN_LIMIT',           'read': VIN_UV_WARN_LIMIT     ,         'write': VIN_UV_WARN_LIMIT     ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVIN_UV_WARN_LIMIT\nthen press ENTER",             'print': print_VIN_UV_WARN_LIMIT     ,          'action': action_VIN_UV_WARN_LIMIT     ,        'items': items_VIN_UV_WARN_LIMIT,           'slaves': SLAVE_ALL,        },
    { 'command': 'VIN_UV_FAULT_LIMIT',          'read': VIN_UV_FAULT_LIMIT    ,         'write': VIN_UV_FAULT_LIMIT    ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nVIN_UV_FAULT_LIMIT\nthen press ENTER",            'print': print_VIN_UV_FAULT_LIMIT    ,          'action': action_VIN_UV_FAULT_LIMIT    ,        'items': items_VIN_UV_FAULT_LIMIT,          'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'VIN_UV_FAULT_RESPONSE',       'read': VIN_UV_FAULT_RESPONSE ,         'write': VIN_UV_FAULT_RESPONSE ,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nVIN_UV_FAULT_RESPONSE\nthen press ENTER",         'print': print_RESPONSE,                        'action': action_VIN_UV_FAULT_RESPONSE ,        'items': items_RESPONSE,                    'slaves': SLAVE_ALL,        },
    { 'command': 'IIN_OC_FAULT_LIMIT',          'read': IIN_OC_FAULT_LIMIT    ,         'write': IIN_OC_FAULT_LIMIT    ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',    'helper': "Press <- OR -> to change\nIIN_OC_FAULT_LIMIT\nthen press ENTER",            'print': print_IIN_OC_FAULT_LIMIT,              'action': action_IIN_OC_FAULT_LIMIT    ,        'items': items_IIN_OC_FAULT_LIMIT,          'slaves': SLAVE_MCU,        },
    { 'command': 'IIN_OC_FAULT_RESPONSE',       'read': IIN_OC_FAULT_RESPONSE ,         'write': IIN_OC_FAULT_RESPONSE ,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nIIN_OC_FAULT_RESPONSE\nthen press ENTER",         'print': print_RESPONSE,                        'action': action_IIN_OC_FAULT_RESPONSE ,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU,        },
    { 'command': 'IIN_OC_WARN_LIMIT',           'read': IIN_OC_WARN_LIMIT     ,         'write': IIN_OC_WARN_LIMIT     ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',    'helper': "Press <- OR -> to change\nIIN_OC_WARN_LIMIT\nthen press ENTER",             'print': print_IIN_OC_WARN_LIMIT,               'action': action_IIN_OC_WARN_LIMIT     ,        'items': items_IIN_OC_WARN_LIMIT,           'slaves': SLAVE_MCU,        },
    { 'command': 'POWER_GOOD_ON',               'read': POWER_GOOD_ON         ,         'write': POWER_GOOD_ON         ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nPOWER_GOOD_ON\nthen press ENTER",                 'print': print_POWER_GOOD_ON         ,          'action': action_POWER_GOOD_ON         ,        'items': items_POWER_GOOD_ON,               'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'POWER_GOOD_OFF',              'read': POWER_GOOD_OFF        ,         'write': POWER_GOOD_OFF        ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',    'helper': "Press <- OR -> to change\nPOWER_GOOD_OFF\nthen press ENTER",                'print': print_POWER_GOOD_OFF        ,          'action': action_POWER_GOOD_OFF        ,        'items': items_POWER_GOOD_OFF,              'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'TON_DELAY',                   'read': TON_DELAY             ,         'write': TON_DELAY             ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'ms',   'helper': "Press <- OR -> to change\nTON_DELAY\nthen press ENTER",                     'print': print_TON_DELAY             ,          'action': action_TON_DELAY             ,        'items': items_TON_DELAY,                   'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'TON_RISE',                    'read': TON_RISE              ,         'write': TON_RISE              ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'ms',   'helper': "Press <- OR -> to change\nTON_RISE\nthen press ENTER",                      'print': print_TON_RISE              ,          'action': action_TON_RISE              ,        'items': items_TON_RISE,                    'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'TON_MAX_FAULT_LIMIT',         'read': TON_MAX_FAULT_LIMIT   ,         'write': TON_MAX_FAULT_LIMIT   ,        'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'ms',   'helper': "Press <- OR -> to change\nTON_MAX_FAULT_LIMIT\nthen press ENTER",           'print': print_TON_MAX_FAULT_LIMIT   ,          'action': action_TON_MAX_FAULT_LIMIT   ,        'items': items_TON_MAX_FAULT_LIMIT,         'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'TON_MAX_FAULT_RESPONSE',      'read': TON_MAX_FAULT_RESPONSE,         'write': TON_MAX_FAULT_RESPONSE,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nTON_MAX_FAULT_RESPONSE\nthen press ENTER",        'print': print_RESPONSE,                        'action': action_TON_MAX_FAULT_RESPONSE,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'TOFF_DELAY',                  'read': TOFF_DELAY,                     'write': TOFF_DELAY,                    'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'ms',   'helper': "Press <- OR -> to change\nTOFF_DELAY,\nthen press ENTER",                   'print': print_TOFF_DELAY,                      'action': action_TOFF_DELAY,                    'items': items_TOFF_DELAY,                  'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'TOFF_FALL',                   'read': TOFF_FALL,                      'write': TOFF_FALL,                     'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'ms',   'helper': "Press <- OR -> to change\nTOFF_FALL,\nthen press ENTER",                    'print': print_TOFF_FALL,                       'action': action_TOFF_FALL,                     'items': items_TOFF_FALL,                   'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'TOFF_MAX_WARN_LIMIT',         'read': TOFF_MAX_WARN_LIMIT,            'write': TOFF_MAX_WARN_LIMIT,           'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'ms',   'helper': "Press <- OR -> to change\nTOFF_MAX_WARN_LIMIT,\nthen press ENTER",          'print': print_TOFF_MAX_WARN_LIMIT,             'action': action_TOFF_MAX_WARN_LIMIT,           'items': items_TOFF_MAX_WARN_LIMIT,         'slaves': SLAVE_MCU_Mx,     },  
    { 'command': 'POUT_OP_FAULT_LIMIT',         'read': POUT_OP_FAULT_LIMIT,            'write': POUT_OP_FAULT_LIMIT,           'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',    'helper': "Press <- OR -> to change\nPOUT_OP_FAULT_LIMIT,\nthen press ENTER",          'print': print_POUT_OP_FAULT_LIMIT,             'action': action_POUT_OP_FAULT_LIMIT,           'items': items_POUT_OP_FAULT_LIMIT,         'slaves': SLAVE_MCU_HSAux,  },
    { 'command': 'POUT_OP_FAULT_RESPONSE',      'read': POUT_OP_FAULT_RESPONSE,         'write': POUT_OP_FAULT_RESPONSE,        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nPOUT_OP_FAULT_RESPONSE\nthen press ENTER",        'print': print_RESPONSE,                        'action': action_POUT_OP_FAULT_RESPONSE,        'items': items_RESPONSE,                    'slaves': SLAVE_MCU_HSAux,  },
    { 'command': 'POUT_OP_WARN_LIMIT',          'read': POUT_OP_WARN_LIMIT,             'write': POUT_OP_WARN_LIMIT,            'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',    'helper': "Press <- OR -> to change\nPOUT_OP_WARN_LIMIT,\nthen press ENTER",           'print': print_POUT_OP_WARN_LIMIT,              'action': action_POUT_OP_WARN_LIMIT,            'items': items_POUT_OP_WARN_LIMIT,          'slaves': SLAVE_MCU_Mx,     },   
    { 'command': 'PIN_OP_WARN_LIMIT',           'read': PIN_OP_WARN_LIMIT,              'write': PIN_OP_WARN_LIMIT,             'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',    'helper': "Press <- OR -> to change\nPIN_OP_WARN_LIMIT,\nthen press ENTER",            'print': print_PIN_OP_WARN_LIMIT,               'action': action_PIN_OP_WARN_LIMIT,             'items': items_PIN_OP_WARN_LIMIT,           'slaves': SLAVE_ALL,        },   
    { 'command': 'MFR_HW_COMPATIBILITY',        'read': MFR_HW_COMPATIBILITY,                                                   'type' : TYPE_WORD,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_HW_COMPATIBILITY,                                                                                                        'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_FWUPLOAD_CAPABILITY',     'read': MFR_FWUPLOAD_CAPABILITY,                                                'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_FWUPLOAD_CAPABILITY,                                                                                                     'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_FWUPLOAD_MODE',           'read': MFR_FWUPLOAD_MODE,              'write': MFR_FWUPLOAD_MODE,             'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nMFR_FWUPLOAD_MODE,\nthen press ENTER",            'print': print_MFR_FWUPLOAD_MODE,               'action': action_MFR_FWUPLOAD_MODE,             'items': items_NO_YES,                      'slaves': SLAVE_MCU,        },
    { 'command': 'STATUS_BYTE',                 'read': STATUS_BYTE,                                                            'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_BYTE,                                                                                                                 'slaves': SLAVE_ALL,        },     
    { 'command': 'STATUS_WORD',                 'read': STATUS_WORD,                                                            'type' : TYPE_WORD,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_WORD,                                                                                                                 'slaves': SLAVE_ALL,        },    
    { 'command': 'STATUS_VOUT',                 'read': STATUS_VOUT,                                                            'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_VOUT,                                                                                                                 'slaves': SLAVE_ALL,        },                                     
    { 'command': 'STATUS_IOUT',                 'read': STATUS_IOUT,                                                            'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_IOUT,                                                                                                                 'slaves': SLAVE_ALL,        },                                     
    { 'command': 'STATUS_INPUT',                'read': STATUS_INPUT,                                                           'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_INPUT,                                                                                                                'slaves': SLAVE_ALL,        },                                     
    { 'command': 'STATUS_TEMPERATURE',          'read': STATUS_TEMPERATURE,                                                     'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_TEMPERATURE,                                                                                                          'slaves': SLAVE_ALL,        },                                     
    { 'command': 'STATUS_CML',                  'read': STATUS_CML,                                                             'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_CML,                                                                                                                  'slaves': SLAVE_ALL,        },                                     
    { 'command': 'STATUS_OTHER',                'read': STATUS_OTHER,                                                           'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_OTHER,                                                                                                                'slaves': SLAVE_MCU_HS,     },                                     
    { 'command': 'STATUS_MFR_SPECIFIC',         'read': STATUS_MFR_SPECIFIC,                                                    'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_STATUS_MFR_SPECIFIC,                                                                                                         'slaves': SLAVE_MCU_HS,     },                                     
    { 'command': 'READ_VIN',                    'read': READ_VIN,                                                               'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',                                                                                           'print': print_READ_VIN,                                                                                                                    'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_IIN',                    'read': READ_IIN,                                                               'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',                                                                                           'print': print_READ_IIN,                                                                                                                    'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_VOUT',                   'read': READ_VOUT,                                                              'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',                                                                                           'print': print_READ_VOUT,                                                                                                                   'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_IOUT',                   'read': READ_IOUT,                                                              'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',                                                                                           'print': print_READ_IOUT,                                                                                                                   'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_TEMPERATURE_1',          'read': READ_TEMPERATURE_1,                                                     'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_READ_TEMPERATURE_1,                                                                                                          'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_TEMPERATURE_2',          'read': READ_TEMPERATURE_2,                                                     'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_READ_TEMPERATURE_2,                                                                                                          'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_TEMPERATURE_3',          'read': READ_TEMPERATURE_3,                                                     'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_READ_TEMPERATURE_3,                                                                                                          'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_POUT',                   'read': READ_POUT,                                                              'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',                                                                                           'print': print_READ_POUT,                                                                                                                   'slaves': SLAVE_ALL,        },                                     
    { 'command': 'READ_PIN',                    'read': READ_PIN,                                                               'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',                                                                                           'print': print_READ_PIN,                                                                                                                    'slaves': SLAVE_ALL,        },                                     
    { 'command': 'PMBUS_REVISION',              'read': PMBUS_REVISION,                                                         'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_PMBUS_REVISION,                                                                                                              'slaves': SLAVE_ALL,        },                                     
    { 'command': 'MFR_ID',                      'read': MFR_ID,                                                                 'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                                                                                                                                                                     'slaves': SLAVE_ALL,        },                                     
    { 'command': 'MFR_MODEL',                   'read': MFR_MODEL,                                                              'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                                                                                                                                                                     'slaves': SLAVE_ALL,        },                                     
    { 'command': 'MFR_REVISION',                'read': MFR_REVISION,                                                           'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                                                                                                                                                                     'slaves': SLAVE_ALL,        },                                     
    { 'command': 'MFR_LOCATION',                'read': MFR_LOCATION,                                                           'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                                                                                                                                                                     'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_DATE',                    'read': MFR_DATE,                                                               'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                                                                                                                                                                     'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_SERIAL',                  'read': MFR_SERIAL,                                                             'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                                                                                                                                                                     'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_VIN_MIN',                 'read': MFR_VIN_MIN,                                                            'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',                                                                                           'print': print_MFR_VIN_MIN,                                                                                                                 'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_VIN_MAX',                 'read': MFR_VIN_MAX,                                                            'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',                                                                                           'print': print_MFR_VIN_MAX,                                                                                                                 'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_IIN_MAX',                 'read': MFR_IIN_MAX,                                                            'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',                                                                                           'print': print_MFR_IIN_MAX,                                                                                                                 'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_PIN_MAX',                 'read': MFR_PIN_MAX,                                                            'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',                                                                                           'print': print_MFR_PIN_MAX,                                                                                                                 'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_VOUT_MIN',                'read': MFR_VOUT_MIN,                                                           'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',                                                                                           'print': print_MFR_VOUT_MIN,                                                                                                                'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_VOUT_MAX',                'read': MFR_VOUT_MAX,                                                           'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'V',                                                                                           'print': print_MFR_VOUT_MAX,                                                                                                                'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_IOUT_MAX',                'read': MFR_IOUT_MAX,                                                           'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',                                                                                           'print': print_MFR_IOUT_MAX,                                                                                                                'slaves': SLAVE_MCU_Mx,     },                                     
    { 'command': 'MFR_POUT_MAX',                'read': MFR_POUT_MAX,                                                           'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'W',                                                                                           'print': print_MFR_POUT_MAX,                                                                                                                'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_TAMBIENT_MAX',            'read': MFR_TAMBIENT_MAX,                                                       'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_MFR_TAMBIENT_MAX,                                                                                                            'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_TAMBIENT_MIN',            'read': MFR_TAMBIENT_MIN,                                                       'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_MFR_TAMBIENT_MIN,                                                                                                            'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_EFFICIENCY_LL',           'read': MFR_EFFICIENCY_LL,                                                      'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_EFFICIENCY_LL,                                                                                                           'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_EFFICIENCY_HL',           'read': MFR_EFFICIENCY_HL,                                                      'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_EFFICIENCY_HL,                                                                                                           'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_MAX_TEMP_1',              'read': MFR_MAX_TEMP_1,                                                         'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_MFR_MAX_TEMP_1,                                                                                                              'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_MAX_TEMP_2',              'read': MFR_MAX_TEMP_2,                                                         'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_MFR_MAX_TEMP_2,                                                                                                              'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_MAX_TEMP_3',              'read': MFR_MAX_TEMP_3,                                                         'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': '°C',                                                                                          'print': print_MFR_MAX_TEMP_3,                                                                                                              'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_RESPONSE_UNIT_CFG',       'read': MFR_RESPONSE_UNIT_CFG,          'write': MFR_RESPONSE_UNIT_CFG,         'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nMFR_RESP_UNIT_CFG,\nthen press ENTER",            'print': print_MFR_RESPONSE_UNIT_CFG,           'action': action_MFR_RESPONSE_UNIT_CFG,         'items': items_MFR_RESPONSE_UNIT_CFG,       'slaves': SLAVE_MCU_Mx,     }, 
    { 'command': 'MFR_PSU_REBOOT',                                                      'write': MFR_PSU_REBOOT,                'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,  'unit': 's',    'helper': "Press <- OR -> to change\nMFR_PSU_REBOOT,\nthen press ENTER",                                                               'action': action_MFR_PSU_REBOOT,                'items': items_MFR_PSU_REBOOT,              'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_FWVERSION_NUMBER',        'read': MFR_FWVERSION_NUMBER,                                                   'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_FWVERSION_NUMBER,                                                                                                        'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_PSU_FW_CRC16_READ',       'read': MFR_PSU_FW_CRC16_READ,                                                  'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_PSU_FW_CRC16_READ,                                                                                                       'slaves': SLAVE_MCU,        },                                     
    { 'command': 'MFR_PSU_POWER_CYCLE',                                                 'write': MFR_PSU_POWER_CYCLE,           'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,  'unit': 's',    'helper': "Press <- OR -> to change\nMFR_PSU_POWER_CYCLE,\nthen press ENTER",                                                          'action': action_MFR_PSU_POWER_CYCLE,           'items': items_MFR_PSU_POWER_CYCLE,         'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_BRICK_CONTROL',           'read': MFR_BRICK_CONTROL,              'write': MFR_BRICK_CONTROL,             'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nMFR_BRICK_CONTROL,\nthen press ENTER",            'print': print_MFR_BRICK_CONTROL,               'action': action_MFR_BRICK_CONTROL,             'items': items_MFR_BRICK_CONTROL,           'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_VIN_PG_THRESH',           'read': MFR_VIN_PG_THRESH,              'write': MFR_VIN_PG_THRESH,             'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,  'unit': 'V',    'helper': "Press <- OR -> to change\nMFR_VIN_PG_THRESH,\nthen press ENTER",            'print': print_MFR_VIN_PG_THRESH,               'action': action_MFR_VIN_PG_THRESH,             'items': items_MFR_VIN_PG_THRESH,           'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_ISHARE_THRESHOLD',        'read': MFR_ISHARE_THRESHOLD,     'write': MFR_ISHARE_THRESHOLD,    'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'A',    'helper': "Press <- OR -> to change\nMFR_ISHARE_THRESHOLD,\nthen press ENTER",         'print': print_MFR_ISHARE_THRESHOLD,      'action': action_MFR_ISHARE_THRESHOLD,    'items': items_MFR_ISHARE_THRESHOLD,  'slaves': SLAVE_MCU_Mx,     },
    { 'command': 'MFR_POWER_ENABLED',           'read': MFR_POWER_ENABLED,              'write': MFR_POWER_ENABLED,             'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER to change\nMFR_POWER_ENABLED",                                  'print': print_MFR_POWER_ENABLED,               'action': action_MFR_POWER_ENABLED,             'items': items_MFR_POWER_ENABLED,           'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_POWER_GOOD',              'read': MFR_POWER_GOOD,                                                         'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_POWER_GOOD,                                                                                                              'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_BRICK_FAILURE',           'read': MFR_BRICK_FAILURE,                                                      'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_BRICK_FAILURE,                                                                                                           'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_FAULTS_ORING',            'read': MFR_FAULTS_ORING,                                                       'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_FAULTS_ORING,                                                                                                            'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_PARAMS_DIFF',             'read': MFR_PARAMS_DIFF,                                                        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_PARAMS_DIFF,                                                                                                             'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_BUS_STATUS',              'read': MFR_BUS_STATUS,                                                         'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_BUS_STATUS,                                                                                                              'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_PMBUS_ALERT',             'read': MFR_PMBUS_ALERT,                                                        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_PMBUS_ALERT,                                                                                                             'slaves': SLAVE_MCU,        }, 
    { 'command': 'MFR_BMC_SIGNALS',             'read': MFR_BMC_SIGNALS,                                                        'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_BMC_SIGNALS,                                                                                                             'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_PMBUS_ADDR',              'read': MFR_PMBUS_ADDR,                 'write': MFR_PMBUS_ADDR,                'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nMFR_PMBUS_ADDR,\nthen press ENTER",               'print': print_MFR_PMBUS_ADDR,                  'action': action_MFR_PMBUS_ADDR,                'items': items_MFR_PMBUS_ADDR,              'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_CAL_DATA',                'read': MFR_CAL_DATA,                   'write': MFR_CAL_DATA,                  'type' : TYPE_BLOCK,           'format' : FORMAT_OTHER,                  'helper': "Press <- OR -> to change\nMFR_CAL_DATA,\nthen press ENTER",                 'print': print_MFR_CAL_DATA,                    'action': action_MFR_CAL_DATA,                  'items': items_MFR_CAL_DATA,                'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_AUTO_ON',                 'read': MFR_AUTO_ON,                    'write': MFR_AUTO_ON,                   'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER to change\nMFR_AUTO_ON",                                        'print': print_MFR_AUTO_ON,                     'action': action_MFR_AUTO_ON,                   'items': items_AUTO_ON,                     'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_ENABLE_DIAG',             'read': MFR_ENABLE_DIAG,                'write': MFR_ENABLE_DIAG,               'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER to change\nMFR_ENABLE_DIAG,",                                   'print': print_MFR_ENABLE_DIAG,                 'action': action_MFR_ENABLE_DIAG,               'items': items_NO_YES,                      'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_DEBUG_STATUS',            'read': MFR_DEBUG_STATUS,               'write': MFR_DEBUG_STATUS,              'type' : TYPE_BYTE,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER to change\nMFR_DEBUG_STATUS,\nthen press ENTER",                'print': print_MFR_DEBUG_STATUS,                'action': action_MFR_DEBUG_STATUS,              'items': items_NO_YES,                      'slaves': SLAVE_MCU,        },
    { 'command': 'MFR_SYSTEM_STATUS1',          'read': MFR_SYSTEM_STATUS1,                                                     'type' : TYPE_WORD,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_SYSTEM_STATUS1,                                                                                                          'slaves': SLAVE_HS,         },
    { 'command': 'MFR_SYSTEM_STATUS2',          'read': MFR_SYSTEM_STATUS2,                                                     'type' : TYPE_WORD,            'format' : FORMAT_OTHER,                                                                                                         'print': print_MFR_SYSTEM_STATUS2,                                                                                                          'slaves': SLAVE_HS,         },
    { 'command': 'MFR_CONFIG1',                 'read': MFR_CONFIG1,                    'write': MFR_CONFIG1,                   'type' : TYPE_WORD,            'format' : FORMAT_OTHER,                  'helper': "Press ENTER to change\nILIM,\nthen press ENTER",                            'print': print_MFR_CONFIG1,                     'action': action_MFR_CONFIG1,                   'items': items_MFR_CONFIG1,                 'slaves': SLAVE_MCU_HS,     },
    { 'command': 'MFR_ADD_DROOP_DURG_RAMP',     'read': MFR_ADDED_DROOP_DURING_RAMP,    'write': MFR_ADDED_DROOP_DURING_RAMP,   'type' : TYPE_WORD,            'format' : FORMAT_LINEAR, 'unit': 'mV/A', 'helper': "Press <- OR -> to change\nMFR_ADD_DROOP_DURG_RAMP,\nthen press ENTER",      'print': print_MFR_ADDED_DROOP_DURING_RAMP,     'action': action_MFR_ADDED_DROOP_DURING_RAMP,   'items': items_VOUT_DROOP,                  'slaves': SLAVE_MCU_Mx,     }, 
]     

######################################################      PARSE ARGS      ##########################################################

# Parse args
read_hex = lambda x: int(x, 16)
read_int = lambda x: int(x, 10)

parser = argparse.ArgumentParser(description="GUI for PMBus board")
parser.add_argument('--bus', help='select the I2C bus', default=2, type=read_int)
parser.add_argument('--board', help='select the PMBus I2C address', default=0x59, type=read_hex)
parser.add_argument('--sensor', help='select the Sensor I2C address', default=0x1a, type=read_hex)
parser.add_argument('--pec', help='enable/disable PEC byte', default=True, action=argparse.BooleanOptionalAction)
args = parser.parse_args()

######################################################  MATH FUNCTION FOR PMBUS  ##########################################################

# PMBus functions
def twos_complement(value, width):
    return value - int((value << 1) & (2 ** width))

def vout_decode(raw, mode):
    exp = twos_complement(mode & 0x1F, 5)
    return raw * ( 2 ** exp)

def vout_encode(val, mode):

    exp = twos_complement(mode & 0x1F, 5)
    
    if exp == 0:
        pass  # Nothing to do
    elif exp > 0:
        for i in range(exp):
            val /= 2
    elif exp < 0:
        for i in range(abs(exp)):
            val *= 2
            
    return int(val)

def linear11_decode(raw):
    mantissa = raw & 0x7FF
    exponent = (raw >> 11) & 0x1F

    m = twos_complement(mantissa, 11)
    e = twos_complement(exponent, 5)

    return m * (2 ** e)

def linear11_encode(value):

    exp = 0

    # Highest mantissa for exponent 0 is 0x3FF, ignoring sign bit
    if abs(value) > 0x3FF:
        while abs(value) > 0x3FF and exp < 15:
            value /= 2
            exp += 1
    else :
        while abs(value) < 0x1FF and has_fractional(value) and exp > -16:
            # Encode smaller values (preserve decimals)
            value *= 2
            exp -= 1

    # Two's complement
    exponent = int(exp if exp > 0 else (1 << 5) + exp)
    mantissa = int(value if value > 0 else (1 << 11) + value)

    return (mantissa & 0x7FF) | (exponent << 11)

def has_fractional(d):
    # Vérifie si la partie fractionnaire est "suffisamment petite" pour être négligeable
    fpart = d - int(d)
    return abs(fpart) > 1e-16

def CHECK_BIT(a, n):
    return (a >> n) & 0x1

def get_operation(operation):
    op = OPERATION_NONE
    
    if not CHECK_BIT(operation, 7) and not CHECK_BIT(operation, 6):  # 0b00XXXXXX
        # Immediate Off (No Sequencing)
        op = OPERATION_IMMEDIATE_OFF
    elif not CHECK_BIT(operation, 7) and CHECK_BIT(operation, 6):  # 0b01XXXXXX
        # Soft Off (With Sequencing) The device powers down according to the programmed turn-off delay and fall time.
        op = OPERATION_SOFT_OFF
    elif CHECK_BIT(operation, 7) and not CHECK_BIT(operation, 6):  # 0b10XXXXXX
        if not CHECK_BIT(operation, 5) and not CHECK_BIT(operation, 4):  # 0b1000XXXX
            # Unit On margin off
            op = OPERATION_ON_MARGIN_OFF
        elif not CHECK_BIT(operation, 5) and CHECK_BIT(operation, 4):  # 0b1001XXXX
            if not CHECK_BIT(operation, 3) and CHECK_BIT(operation, 2):  # 0b100101XX
                # Unit On margin low (Ignore Fault)
                op = OPERATION_ON_MARGIN_LOW_IGNORE_FAULT
            elif CHECK_BIT(operation, 3) and not CHECK_BIT(operation, 2):  # 0b100110XX
                # Unit On margin low (Act On Fault)
                op = OPERATION_ON_MARGIN_LOW_ACT_ON_FAULT
        elif CHECK_BIT(operation, 5) and not CHECK_BIT(operation, 4):  # 0b1010XXXX
            if not CHECK_BIT(operation, 3) and CHECK_BIT(operation, 2):  # 0b101001XX
                # Unit On margin high (Ignore Fault)
                op = OPERATION_ON_MARGIN_HIGH_IGNORE_FAULT
            elif CHECK_BIT(operation, 3) and not CHECK_BIT(operation, 2):  # 0b101010XX
                # Unit On margin high (Act On Fault)
                op = OPERATION_ON_MARGIN_HIGH_ACT_ON_FAULT
    
    return op

def get_response_hex(item):
    for index in range(len(items_RESPONSE)):
        if item == items_RESPONSE[index]:
            return hex_RESPONSE[index]
        
def get_current_response_hex(item):
    for index in range(len(items_Current_RESPONSE)):
        if item == items_Current_RESPONSE[index]:
            return hex_RESPONSE[index]
######################################################  GENERIC PANEL  ##########################################################

# GUI items
class Panel:

    _win = None
    _title = None
    _refresh = True

    def __init__(self, x, y, w, h, title):
        self._win = curses.newwin(h, w, y, x)
        self._title = title

    def redraw(self):
        pass

    def update(self):
        self._win.erase()
        self._win.attron(curses.A_DIM)
        self._win.box()
        self._win.attroff(curses.A_DIM)

        self.redraw()

        if self._title is not None:
            self._win.addstr(0, 1, f" {self._title} ")
    
        if self._refresh:
            self._win.refresh()         

class ActionPanel(Panel):

    _board = None

    def __init__(self, board, x, y, w, h, title):
        super().__init__(x, y, w, h, title)
        self._win.keypad(True)
        self._win.timeout(100)
        self._refresh = True
        self._board = board

    def keypress(self, key):
        pass

    def click(self, evt):
        pass

    def isResponseCmd(self, command):
        if (command == 'VOUT_OV_FAULT_RESPONSE') or (command == 'VOUT_UV_FAULT_RESPONSE') or (command == 'IOUT_OC_FAULT_RESPONSE') or (command == 'IOUT_OC_LV_FAULT_RESPONSE') or (command == 'IOUT_UC_FAULT_RESPONSE') or (command == 'OT_FAULT_RESPONSE') or (command == 'UT_FAULT_RESPONSE') or (command == 'VIN_OV_FAULT_RESPONSE') or (command == 'VIN_UV_FAULT_RESPONSE') or (command == 'IIN_OC_FAULT_RESPONSE') or (command == 'TON_MAX_FAULT_RESPONSE') or (command == 'POUT_OP_FAULT_RESPONSE'):
            return True
        else:
            return False

class RefreshPanel(Panel):

    _last_redraw = 0
    _refresh_delay = 1

    def update(self):
        if time.time() - self._last_redraw > self._refresh_delay:
            super().update()
            self._last_redraw = time.time()

######################################################  TITLE PANEL  ##########################################################

class TitlePanel(Panel):

    _board = None

    def __init__(self, board, x, y, w, h, title):
        super().__init__(x, y, w, h, title)
        self._board = board

    def redraw(self):
        super().redraw()
        line = 1

        # Display logo
        #h, w = self._win.getmaxyx()
        #logo_y = h - len(logo) - 2
        #logo_x = (w - len(logo[0])) // 2
        logo_x = 1
        logo_y = 2
        for i, l in enumerate(logo):
            self._win.addstr(logo_y+i, logo_x, l, curses.color_pair(F_YELLOW_B_BLACK))

        # Display Title
        self._win.addstr(line, ((W_TITLE - len(AppTitle)) // 2) + 1,  AppTitle, curses.color_pair(F_YELLOW_B_BLACK))



######################################################  MENU PANEL  ##########################################################

class MenuPanel(ActionPanel):

    _items = []

    def __init__(self, board, x, y, w, h, title):
        super().__init__(board, x, y, w, h, title)
        self._board = board

    def add_item(self, item):
        self._items.append(item)

    def redraw(self):

        _, w = self._win.getmaxyx()
        selected = self._board.get_selected() - MENU_SELECTION_OFFSET
        zone = self._board.get_previousZone()

        for index, item in enumerate(self._items):
            attr = curses.color_pair(F_BLACK_B_WHITE) | curses.A_BOLD if index == selected and zone == MENU_SELECTION_OFFSET else curses.A_BOLD
            self._win.addstr(index + 1, 1, " {0:{1}s} ".format(item.get_label(), w-4), attr)

            if index == selected:
                HelperPanel.set_helper(HelperPanel, self._items[selected].get_helper())
      

    def click(self, evt):

        if self._board.get_oldSelected() == self._board.get_selected():

            clicked = self._board.get_selected() - MENU_SELECTION_OFFSET

            if 0 <= clicked < len(self._items):
                    if evt == curses.BUTTON1_RELEASED:
                        self._items[clicked].do_action(ord('\n'))
                    elif evt == curses.BUTTON3_RELEASED:
                        self._items[clicked].do_action(curses.KEY_RIGHT)

    def keypress(self, key):

        selected = self._board.get_selected() - MENU_SELECTION_OFFSET

        if selected >= 0 and selected <= len(self._items):
            if key == curses.KEY_UP:
                selected = (selected - 1)
            elif key == curses.KEY_DOWN:
                selected = (selected + 1)
            else:
                self._items[selected].do_action(key)

            self._board.set_selected(selected + MENU_SELECTION_OFFSET)

class MenuItem:

    _label = None
    _helper = None
    _action = None

    def __init__(self, label, helper, action = None):
        self._label = label
        self._helper= helper
        self._action = action

    def get_label(self):
        return self._label

    def get_helper(self):
        return self._helper

    def do_action(self, key):
        self._action(self, key)

class SimpleMenuItem(MenuItem):

    def do_action(self, key):
        if key == ord('\n'):
            self._action(self)

######################################################  HELPER PANEL  ##########################################################

class HelperPanel(RefreshPanel):

    _board = None
    _helper = "helper"
    def __init__(self, board, x, y, w, h, title):
        super().__init__(x, y, w, h, title)
        self._board = board

    def set_helper(self, helper):
        self._helper = helper

    def redraw(self): 
        # Découper le texte en morceaux
        text_lines = self._helper.split("\n")

        # Afficher chaque ligne dans la boîte
        for i, line in enumerate(text_lines):
            self._win.addstr(i+1, 1, line)

######################################################  EFFICIENCY PANEL  ##########################################################

class EfficiencyPanel(RefreshPanel):

    _board = None
    _pout = 0
    _pin  = 0    
    _vout = 0
    _iout = 0

    def __init__(self, board, x, y, w, h, title):
        super().__init__(x, y, w, h, title)
        self._board = board

    def set_pout(self, pout):
        self._pout = pout

    def set_pin(self, pin):
        self._pin = pin

    def set_vout(self, vout):
        self._vout = vout

    def set_iout(self, iout):
        self._iout = iout

    def redraw(self): 
        if not self._pin == 0:
            percent = 0
            percent = (float(self._pout) / float(self._pin)) * 100
            percent = "{:.2f}".format(percent)
            self._win.addstr(1, 8, percent, curses.color_pair(self._board._color))
            self._win.addstr(1, 16, "%", curses.color_pair(self._board._color))
        else:
            self._win.addstr(1, 8, "n/a", curses.color_pair(F_RED_B_BLACK))

        pout_calc = float(self._vout) * float(self._iout)
        pout_str = "{:.2f}".format(pout_calc)
        self._win.addstr(2, 1, "Pout: ", curses.color_pair(F_WHITE_B_BLACK))
        self._win.addstr(2, 8, pout_str, curses.color_pair(self._board._color))
        self._win.addstr(2, 16, "W", curses.color_pair(self._board._color))  

            

######################################################  PMBUS WRITE PANEL  ##########################################################

class PMBusWrite1Panel(ActionPanel):

    _commands = []

    def add_command(self, command):
        self._commands.append(command)

    def redraw(self):
        selected = self._board.get_selected() - PMBUS_WRITE_1_SELECTION_OFFSET
        zone = self._board.get_previousZone()

        if selected >= 0 and selected < len(self._commands):
            HelperPanel.set_helper(HelperPanel, self._commands[selected].get_helper())

        for index, command in enumerate(self._commands):

            attr = curses.color_pair(F_BLACK_B_WHITE) | curses.A_BOLD if index == selected and zone == PMBUS_WRITE_1_SELECTION_OFFSET else curses.A_BOLD
            pos = ((W_PMBUS_WRITE_1 // 2) + 2)
            self._win.addstr(index + 1, 1, "{0:{1}s}".format(command.get_label(),len(command.get_label())), attr)    

            if not self.isResponseCmd(command.get_label()):
                try:
                    val = command._items[command._ValueSelected]

                    if val <= -100 :
                        val = "{:.1f}".format(val)                  
                    elif(val > -100) and (val <= -10):
                        val = "{:.2f}".format(val)                     
                    elif(val > -10) and (val < 0):
                        val = "{:.3f}".format(val)                
                    elif(val >= 0) and (val < 10):
                        val = "{:.4f}".format(val)
                    elif(val >= 10) and (val < 100):
                        val = "{:.3f}".format(val)
                    elif(val >= 100) and (val < 1000):
                        val = "{:.2f}".format(val)                
                    else:
                        val = "{:.1f}".format(val)

                    self._win.addstr(index + 1, pos, str(val), self._board._color)
                except:
                    try:
                        self._win.addstr(index + 1, pos, command._items[command._ValueSelected], self._board._color)    
                    except:
                        continue
                try:
                    self._win.addstr(index + 1, pos + 7, command._unit, self._board._color)     
                except:
                    continue
            else:
                self._win.addstr(index + 1, pos, command._items[command._ValueSelected], self._board._color)    

    def click(self, evt):

        if self._board.get_oldSelected() == self._board.get_selected():

            clicked = self._board.get_selected() - PMBUS_WRITE_1_SELECTION_OFFSET

            if 0 <= clicked < len(self._commands):
                    if evt == curses.BUTTON1_RELEASED:
                        self._commands[clicked].do_action(ord('\n'))
                    elif evt == curses.BUTTON3_RELEASED:
                        self._commands[clicked].do_action(curses.KEY_RIGHT)          

    def keypress(self, key):

        selected = self._board.get_selected() - PMBUS_WRITE_1_SELECTION_OFFSET

        if selected >= 0 and selected <= len(self._commands):
            if key == curses.KEY_UP:
                selected = (selected - 1)
            elif key == curses.KEY_DOWN:
                selected = (selected + 1)
            else:
                self._commands[selected].do_action(key)

            self._board.set_selected(selected + PMBUS_WRITE_1_SELECTION_OFFSET)      

class PMBusWrite2Panel(ActionPanel):

    _commands = []

    def add_command(self, command):
        self._commands.append(command)

    def redraw(self):
        selected = self._board.get_selected() - PMBUS_WRITE_2_SELECTION_OFFSET
        zone = self._board.get_previousZone()
   
        if selected >= 0 and selected < len(self._commands):
            HelperPanel.set_helper(HelperPanel, self._commands[selected].get_helper())

        for index, command in enumerate(self._commands):

            attr = curses.color_pair(F_BLACK_B_WHITE) | curses.A_BOLD if index == selected and zone == PMBUS_WRITE_2_SELECTION_OFFSET else curses.A_BOLD
            pos = ((W_PMBUS_WRITE_2 // 2) + 3)
            self._win.addstr(index + 1, 1, "{0:{1}s}".format(command.get_label(),len(command.get_label())), attr)    

            try:
                self._win.addstr(index + 1, pos, command._items[command._ValueSelected], self._board._color)     
            except:
                try:
                    self._win.addstr(index + 1, pos, str(command._items[command._ValueSelected]), self._board._color)
                except:
                    continue

            try:
                self._win.addstr(index + 1, pos + 7, command._unit, self._board._color)     
            except:
                continue
            

    def click(self, evt):

        if self._board.get_oldSelected() == self._board.get_selected():

            clicked = self._board.get_selected() - PMBUS_WRITE_2_SELECTION_OFFSET

            if 0 <= clicked < len(self._commands):
                    if evt == curses.BUTTON1_RELEASED:
                        self._commands[clicked].do_action(ord('\n'))
                    elif evt == curses.BUTTON3_RELEASED:
                        self._commands[clicked].do_action(curses.KEY_RIGHT)

    def keypress(self, key):

        selected = self._board.get_selected() - PMBUS_WRITE_2_SELECTION_OFFSET

        if selected >= 0 and selected <= len(self._commands):
            if key == curses.KEY_UP:
                selected = (selected - 1)
            elif key == curses.KEY_DOWN:
                selected = (selected + 1)
            else:
                self._commands[selected].do_action(key)

            self._board.set_selected(selected + PMBUS_WRITE_2_SELECTION_OFFSET)      

class WriteItem:

    _pmbus = None
    _label = None
    _commandIndex = 0
    _command = 0x0
    _helper = None
    _type = 0
    _format = 0
    _unit = None
    _action = None

    def __init__(self, pmbus, label, index, command, helper, type, format, unit, action):
        self._pmbus = pmbus
        self._label = label
        self._commandIndex = index
        self._command = command
        self._helper= helper
        self._type= type
        self._format= format
        self._unit= unit
        self._action = action

    def get_label(self):
        return self._label

    def get_commandIndex(self):
        return self._commandIndex
    
    def get_helper(self):
        return self._helper

    def get_type(self):
        return self._type

    def get_format(self):
        return self._format
    
    def get_unit(self):
        return self._unit
    
    def do_action(self, key):
        self._action(self, key)

class SelectWriteItem(WriteItem):

    _ValueSelected = 0
    _items = []

    def __init__(self, pmbus, label, index, command, helper, type, format, unit, action, items):
        super().__init__(pmbus, label, index, command, helper, type, format, unit, action)
        self._items = items

    def get_label(self):
        item = self._items[self._ValueSelected]
        return WriteItem.get_label(self).format(item)

    def do_action(self, key):
        if key < 0: pass
        elif key == curses.KEY_LEFT:
            self._ValueSelected = (self._ValueSelected - 1) % len(self._items)
        elif key == curses.KEY_RIGHT:
            self._ValueSelected = (self._ValueSelected + 1) % len(self._items)
        elif key == ord('\n'):
            item = self._items[self._ValueSelected]
            self._action(self, item)

######################################################  PMBUS READ PANEL  ##########################################################

class PMBusRead1Panel(RefreshPanel):

    _board = None

    def __init__(self, board, x, y, w, h, title):
        super().__init__(x, y, w, h, title)
        self._board = board

    def redraw(self):

        line = 1
        skip = False

        for command in pmbus_commands:
            try:
                read_command = command['read']                                                                      # Display PMBus only if read command exist because another panel is used for write commands
                if read_command == FIRST_PMBUS_READ_2:
                    skip = True

                if skip == False:
                    slaves = command['slaves']
                    if (((not (slaves == SLAVE_HS_Mx) and not (slaves == SLAVE_HS) and not (slaves == SLAVE_Mx)) and (self._board._page == 0)) or ((not (slaves == SLAVE_MCU) and not (slaves == SLAVE_MCU_HS) and not (slaves == SLAVE_MCU_HSAux) and not (slaves == SLAVE_HS)) and (1 <= self._board._page <= 3)) or ((not (slaves == SLAVE_MCU) and not (slaves == SLAVE_MCU_Mx)and not (slaves == SLAVE_MCU_HSAux) and not (slaves == SLAVE_Mx)) and (6 <= self._board._page <= 8)) or ((not (slaves == SLAVE_MCU) and not (slaves == SLAVE_MCU_Mx) and not (slaves == SLAVE_Mx)) and (self._board._page == 5))) :
                        self._win.addstr(line, 1, command['command'], curses.A_BOLD)                                    # Display name of the command
                        pos = (W_PMBUS_READ_1 // 2) + 3
                        
                        if command['type'] == TYPE_BYTE:                                                                # If type of data is byte
                            try:
                                raw = self._board.read_byte(read_command)
                                try:
                                    print = command["print"]
                                    print(self, line, pos, raw)
                                except:
                                    self._win.addstr(line, pos, format(raw, '#08b'), curses.color_pair(F_WHITE_B_BLACK))

                            except IOError:
                                self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))  

                        elif command['type'] == TYPE_WORD: 
                            try:
                                raw = self._board.read_word(read_command)
                                try:
                                    print = command["print"]
                                    print(self, line, pos, raw)
                                except:
                                    self._win.addstr(line, pos, format(raw, '#16b'), curses.color_pair(F_WHITE_B_BLACK))  

                            except IOError:
                                self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK)) 

                        elif command['type'] == TYPE_BLOCK: 
                            try:
                                print = command["print"]
                                try:
                                    raw = self._board.read_block(read_command)
                                    print(self, line, pos, raw)
                                except:
                                    self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))
                            except:
                                try:
                                    value = self._board.read_string(read_command)
                                    self._win.addstr(line, pos, value, curses.color_pair(F_WHITE_B_BLACK))

                                except:
                                    self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))

                        line += 1
            except:
                continue                               


class PMBusRead2Panel(RefreshPanel):

    _board = None

    def __init__(self, board, x, y, w, h, title):
        super().__init__(x, y, w, h, title)
        self._board = board

    def redraw(self):

        line = 1
        skip = True

        for command in pmbus_commands:
            try:
                read_command = command['read']                                                                      # Display PMBus only if read command exist because another panel is used for write commands
                if read_command == FIRST_PMBUS_READ_2:
                    skip = False

                if skip == False:
                    slaves = command['slaves']
                    if (((not (slaves == SLAVE_HS_Mx) and not (slaves == SLAVE_HS) and not (slaves == SLAVE_Mx)) and (self._board._page == 0)) or ((not (slaves == SLAVE_MCU) and not (slaves == SLAVE_MCU_HS) and not (slaves == SLAVE_MCU_HSAux) and not (slaves == SLAVE_HS)) and (1 <= self._board._page <= 3)) or ((not (slaves == SLAVE_MCU) and not (slaves == SLAVE_MCU_Mx)and not (slaves == SLAVE_MCU_HSAux) and not (slaves == SLAVE_Mx)) and (6 <= self._board._page <= 8)) or ((not (slaves == SLAVE_MCU) and not (slaves == SLAVE_MCU_Mx) and not (slaves == SLAVE_Mx)) and (self._board._page == 5))) :
                        self._win.addstr(line, 1, command['command'], curses.A_BOLD)                                    # Display name of the command
                        pos = (W_PMBUS_READ_2 // 3) + 5

                        if command['type'] == TYPE_BYTE:                                                                # If type of data is byte
                            try:
                                raw = self._board.read_byte(read_command)

                                try:
                                    print = command["print"]
                                    print(self, line, pos, raw)
                                except:
                                    self._win.addstr(line, pos, format(raw, '#08b'), curses.color_pair(F_WHITE_B_BLACK))

                            except IOError:
                                self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))  

                        elif command['type'] == TYPE_WORD: 
                            try:
                                raw = self._board.read_word(read_command)
                                try:
                                    print = command["print"]
                                    print(self, line, pos, raw)
                                except:
                                    self._win.addstr(line, pos, format(raw, '#16b'), curses.color_pair(F_WHITE_B_BLACK))

                            except IOError:
                                self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))

                        elif command['type'] == TYPE_BLOCK: 
                            try:
                                print = command["print"]
                                try:
                                    raw = self._board.read_block(read_command)
                                    print(self, line, pos, raw)
                                except:
                                    self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))
                            except:
                                try:
                                    value = self._board.read_string(read_command)
                                    self._win.addstr(line, pos, value, curses.color_pair(F_WHITE_B_BLACK))

                                except:
                                    self._win.addstr(line, pos, "n/a", curses.color_pair(F_RED_B_BLACK))

                        line += 1
            except:
                continue 

######################################################  PMBUS CLASS  ##########################################################

class PMBus:

    _bus = None
    _addr = 0
    _selected = MENU_SELECTION_OFFSET
    _oldSelected = MENU_SELECTION_OFFSET
    _previousZone = MENU_SELECTION_OFFSET
    _color = None
    _page = 0
    _timestamp = None

    def __init__(self, bus, addr, color):
        self._bus = SMBus(bus)
        self._addr = addr
        self._bus.pec = 1 if args.pec else 0
        self._color = color
        self._timestamp = datetime.now().strftime("%y%m%d_%H%M%S")

    def read_word(self, reg):
        return self._bus.read_word_data(self._addr, reg)

    def write_word(self, reg, word):
        self._bus.write_word_data(self._addr, reg, word)

    def read_byte(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def write_byte(self, reg, byte = None):
        if byte is None: self._bus.write_byte(self._addr, reg)
        else: self._bus.write_byte_data(self._addr, reg, byte)

    def write_block(self, reg, data):
        self._bus.write_block_data(self._addr, reg, data)

    # Driver does not support read_block_data natively
    def read_block(self, reg):
        # Use I2C functions to ignore PEC byte
        [length] = self._bus.read_i2c_block_data(self._addr, reg, 1)
        # Mimic the IO error thrown with PEC enabled
        if length >= 0xFF: raise IOError(errno.EBADMSG)
        raw = self._bus.read_i2c_block_data(self._addr, reg, length+1)
        return raw[1:]

    def read_string(self, reg):
        raw = self.read_block(reg)
        chars = map(chr, raw)
        return ''.join(chars)

    def get_previousZone(self):
        return self._previousZone

    def get_selected(self):
        return self._selected
    
    def get_oldSelected(self):
        return self._oldSelected
        
    # >->MENU>------>PMBUS_WRITE_1>------>PMBUS_WRITE_2>->
    def set_selected(self, selected):
        MENU_SELECTION_END = MENU_SELECTION_OFFSET + len(MenuPanel._items)
        PMBUS_WRITE_1_SELECTION_END = PMBUS_WRITE_1_SELECTION_OFFSET + len(PMBusWrite1Panel._commands)
        PMBUS_WRITE_2_SELECTION_END = PMBUS_WRITE_2_SELECTION_OFFSET + len(PMBusWrite2Panel._commands)

        self._oldSelected = self._selected

        if selected < MENU_SELECTION_OFFSET: # Before Menu zone
            self._selected =  PMBUS_WRITE_2_SELECTION_END - 1 # select last of PMBus Write 2
            self._previousZone = PMBUS_WRITE_2_SELECTION_OFFSET

        elif selected >= MENU_SELECTION_OFFSET and selected < MENU_SELECTION_END: # inside Menu Zone
            self._selected = selected # No jump
            self._previousZone = MENU_SELECTION_OFFSET

        elif selected >= MENU_SELECTION_END and selected < PMBUS_WRITE_1_SELECTION_OFFSET: # Between Menu zone & PMBus write 1 Zone
            if self._previousZone == MENU_SELECTION_OFFSET: # Coming from Menu zone
                self._selected = PMBUS_WRITE_1_SELECTION_OFFSET
                self._previousZone = PMBUS_WRITE_1_SELECTION_OFFSET
            elif self._previousZone == PMBUS_WRITE_1_SELECTION_OFFSET: # Coming from PMBus write 1 zone
                self._selected = MENU_SELECTION_END - 1
                self._previousZone = MENU_SELECTION_OFFSET
            else: # Should not exist
                self._selected = MENU_SELECTION_OFFSET
                self._previousZone = MENU_SELECTION_OFFSET

        elif selected >= PMBUS_WRITE_1_SELECTION_OFFSET and selected < PMBUS_WRITE_1_SELECTION_END: # inside PMBus write 1 Zone
            self._selected = selected # No jump
            self._previousZone = PMBUS_WRITE_1_SELECTION_OFFSET

        elif selected >= PMBUS_WRITE_1_SELECTION_END and selected < PMBUS_WRITE_2_SELECTION_OFFSET: # Between PMBus write 1 Zone & PMBus write 2 Zone
            if self._previousZone == PMBUS_WRITE_1_SELECTION_OFFSET: # Coming from PMBus write 1
                self._selected = PMBUS_WRITE_2_SELECTION_OFFSET
                self._previousZone = PMBUS_WRITE_2_SELECTION_OFFSET
            elif self._previousZone == PMBUS_WRITE_2_SELECTION_OFFSET: # Coming from PMBus write 2 zone
                self._selected = PMBUS_WRITE_1_SELECTION_END - 1
                self._previousZone = PMBUS_WRITE_1_SELECTION_OFFSET
            else: # Should not exist
                self._selected = MENU_SELECTION_OFFSET
                self._previousZone = MENU_SELECTION_OFFSET

        elif selected >= PMBUS_WRITE_2_SELECTION_OFFSET and selected < PMBUS_WRITE_2_SELECTION_END: # inside PMBus write 2 Zone
            self._selected = selected # No jump
            self._previousZone = PMBUS_WRITE_2_SELECTION_OFFSET

        else : # selected >= PMBUS_WRITE_2_SELECTION_END
            self._selected = MENU_SELECTION_OFFSET  # select first of Menu
            self._previousZone = MENU_SELECTION_OFFSET

######################################################  MAIN  ##########################################################

def main(win):

    # Check terminal size

    h, w = win.getmaxyx()

    if w < W_TOTAL or h < H_TOTAL:
        curses.resizeterm(H_TOTAL, W_TOTAL)

    ######################### Curses initialisation
    curses.halfdelay(10)  # add timeout when waiting for a keyboard or mouse event in order to update display when no user action
    curses.curs_set(False)
    curses.mousemask(curses.BUTTON1_RELEASED | curses.BUTTON3_RELEASED)
    curses.init_pair(F_RED_B_BLACK,     curses.COLOR_RED,    curses.COLOR_BLACK)
    curses.init_pair(F_GREEN_B_BLACK,   curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(F_YELLOW_B_BLACK,  curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(F_BLUE_B_BLACK,    curses.COLOR_BLUE,   curses.COLOR_BLACK)
    curses.init_pair(F_MAGENTA_B_BLACK, curses.COLOR_MAGENTA,curses.COLOR_BLACK)
    curses.init_pair(F_CYAN_B_BLACK,    curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(F_WHITE_B_BLACK,   curses.COLOR_WHITE,  curses.COLOR_BLACK)
    curses.init_pair(F_BLACK_B_RED,     curses.COLOR_BLACK,  curses.COLOR_RED)
    curses.init_pair(F_BLACK_B_GREEN,   curses.COLOR_BLACK,  curses.COLOR_GREEN)
    curses.init_pair(F_BLACK_B_YELLOW,  curses.COLOR_BLACK,  curses.COLOR_YELLOW)
    curses.init_pair(F_BLACK_B_WHITE,   curses.COLOR_BLACK,  curses.COLOR_WHITE)

    ######################### Init UI elements

    pmbus        = PMBus           (args.bus, args.board, curses.color_pair(F_WHITE_B_BLACK))
    titlepanel   = TitlePanel      (pmbus, X_TITLE,          Y_TITLE,          W_TITLE,          H_TITLE,          None)
    menu         = MenuPanel       (pmbus, X_MENU,           Y_MENU,           W_MENU,           H_MENU,           TITLE_MENU)
    helper       = HelperPanel     (pmbus, X_HELPER,         Y_HELPER,         W_HELPER,         H_HELPER,         TITLE_HELPER)
    efficiency   = EfficiencyPanel (pmbus, X_EFFICIENCY,     Y_EFFICIENCY,     W_EFFICIENCY,     H_EFFICIENCY,     TITLE_EFFICIENCY)
    pmbuswrite1  = PMBusWrite1Panel(pmbus, X_PMBUS_WRITE_1,  Y_PMBUS_WRITE_1,  W_PMBUS_WRITE_1,  H_PMBUS_WRITE_1,  TITLE_PMBUS_WRITE_1)
    pmbuswrite2  = PMBusWrite2Panel(pmbus, X_PMBUS_WRITE_2,  Y_PMBUS_WRITE_2,  W_PMBUS_WRITE_2,  H_PMBUS_WRITE_2,  TITLE_PMBUS_WRITE_2)
    pmbusread1   = PMBusRead1Panel (pmbus, X_PMBUS_READ_1,   Y_PMBUS_READ_1,   W_PMBUS_READ_1,   H_PMBUS_READ_1,   TITLE_PMBUS_READ_1)
    pmbusread2   = PMBusRead2Panel (pmbus, X_PMBUS_READ_2,   Y_PMBUS_READ_2,   W_PMBUS_READ_2,   H_PMBUS_READ_2,   TITLE_PMBUS_READ_2)

    ######################### Callbacks

    def action_exit(self):
        sys.exit(0)

    ######################### PMBus_Write items
    # create output items
    commands_items = []
    commandIndex = 0
    for command in pmbus_commands:
        try:
            write_command = command['write']
            try: 
                items = command['items']
                try:
                    unit = command['unit'] 
                    commands_items.append(SelectWriteItem(pmbus, command["command"], commandIndex, write_command, command["helper"], command["type"], command["format"], unit, command["action"], items  ))
                    commandIndex +=1
                except:
                    commands_items.append(SelectWriteItem(pmbus, command["command"], commandIndex, write_command, command["helper"], command["type"], command["format"], None, command["action"], items  ))
                    commandIndex +=1
            except:
                try:
                    unit = command['unit'] 
                    commands_items.append(WriteItem(pmbus, command["command"], commandIndex, write_command, command["helper"], command["type"], command["format"], unit, command["action"]  ))
                    commandIndex +=1
                except:
                    commands_items.append(WriteItem(pmbus, command["command"], commandIndex, write_command, command["helper"], command["type"], command["format"], None, command["action"]  ))
                    commandIndex +=1 
        except:
            continue

    # build outputs panel

    for command_item in commands_items:
            if command_item.get_commandIndex() < H_PMBUS_WRITE_1 - 2 :
                pmbuswrite1.add_command(command_item)
            else :
                pmbuswrite2.add_command(command_item)

    ######################### MENU

    # Menu items
    item_exit           = SimpleMenuItem("Exit",                        "Press Enter to exit",                                                 action_exit    )

    # build menu
    menu.add_item(item_exit)



    ######################### LOOP

    while True:

        ### Update PAGE color :
        try:
            # pmbus._page = pmbus.read_byte(PAGE)
            pmbus._page = 1
            if pmbus._page == 0:
                pmbus._color = curses.color_pair(F_WHITE_B_BLACK)
            elif 1 <= pmbus._page <= 3:
                pmbus._color = curses.color_pair(F_YELLOW_B_BLACK)
            elif 5 <= pmbus._page <= 8:
                pmbus._color = curses.color_pair(F_CYAN_B_BLACK)
        except:
            pass

        ### Update display
        titlepanel.update()
        menu.update()             
        pmbuswrite1.update()  
        pmbuswrite2.update() 
        helper.update()
        efficiency.update()        
        pmbusread1.update()
        pmbusread2.update()

        ### Get mouse or keyboard events

        key = win.getch()

        if key < 0: pass
        elif key == curses.KEY_MOUSE:
            #try:
            _, mx, my, _, evt = curses.getmouse()

            # does the click is inside menu windows ?
            if X_MENU <= mx <= X_MENU + W_MENU and Y_MENU <= my <= Y_MENU + H_MENU:
                clicked = my - Y_MENU - 1
                pmbus.set_selected(clicked + MENU_SELECTION_OFFSET)
                menu.click(evt)

            # does the click is inside pmbuswrite1 windows ?
            elif X_PMBUS_WRITE_1 <= mx <= X_PMBUS_WRITE_1 + W_PMBUS_WRITE_1 and Y_PMBUS_WRITE_1 <= my <= Y_PMBUS_WRITE_1 + H_PMBUS_WRITE_1:
                clicked = my - Y_PMBUS_WRITE_1 - 1
                pmbus.set_selected(clicked + PMBUS_WRITE_1_SELECTION_OFFSET)
                pmbuswrite1.click(evt)

            # does the click is inside pmbuswrite2 windows ?
            elif X_PMBUS_WRITE_2 <= mx <= X_PMBUS_WRITE_2 + W_PMBUS_WRITE_2 and Y_PMBUS_WRITE_2 <= my <= Y_PMBUS_WRITE_2 + H_PMBUS_WRITE_2:
                clicked = my - Y_PMBUS_WRITE_2 - 1
                pmbus.set_selected(clicked + PMBUS_WRITE_2_SELECTION_OFFSET)
                pmbuswrite2.click(evt)
            #except: pass

            # click is somewhere else
            else: 
                pass
        else :
            if pmbus._previousZone == MENU_SELECTION_OFFSET:
                menu.keypress(key)
            elif pmbus._previousZone == PMBUS_WRITE_1_SELECTION_OFFSET:
                pmbuswrite1.keypress(key)
            elif pmbus._previousZone == PMBUS_WRITE_2_SELECTION_OFFSET:
                pmbuswrite2.keypress(key)

######################### EXIT

# ===========================================================================
#  Bloc principal d'exécution et de gestion des erreurs
# ===========================================================================

try:
    curses.wrapper(main)
    
    # Si le programme se termine normalement, on sort avec un code 0
    sys.exit(0)

except KeyboardInterrupt:
    print("\nInterruption clavier. Sortie propre.")
    sys.exit(0)

except IOError as e:
    code = e.args[0]
    print("\nErreur de bus (IOError) :", errno.errorcode[code])
    sys.exit(1)

except curses.error as err:
    print("\nErreur d'affichage (curses) :", err)
    sys.exit(1)