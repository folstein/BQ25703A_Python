from ssl import CHANNEL_BINDING_TYPES
from numpy import byte
from smbus2 import SMBus
import time
import RPi.GPIO as GPIO

from enum import IntFlag
from bq25703a_reg import *

BQ25703A_I2C_ADDRESS        = 0x6b
BQ25703A_MANUFACTURER_ID    = 0x40
BQ25703A_DEVICE_ID          = 0x78

ADC_ENABLED_BITMASK         = EN_ADC_VBUS_MASK | EN_ADC_IIN_MASK | EN_ADC_ICHG_MASK | EN_ADC_VSYS_MASK | EN_ADC_VBAT_MASK
ADC_START_CONVERSION_MASK   = ADC_START_MASK | ADC_FULLSCALE_MASK

CHARGEVOLT_MAX = 19200
CHARGEVOLT_MIN = 1024

MINSYSVOLT_MAX = 16128
MINSYSVOLT_MIN = 1024

CHARGECURRENT_MAX = CHARGECURRENT_BASE + (0x7F * CHARGECURRENT_LSB)

OTGVOLT_MAX = OTGVOLT_BASE + (0xFF * OTGVOLT_LSB)
OTGCURRENT_MAX = OTGCURRENT_BASE + (0x7F * OTGCURRENT_LSB)

class bq25703a:
    i2c_address = BQ25703A_I2C_ADDRESS
    i2c_bus = 1
    ilim_hiz_pin = 17
    otg_en_pin = 16

    connected = 0
    charging_status: bool = False

    vbat_voltage = 0
    vbus_voltage = 0
    vsys_voltage = 0

    input_current = 0
    charge_current = 0
    max_charge_current = 0

    charger_status_20 = 0
    charger_status_21 = 0

    manufacturer_id = 0
    device_id = 0


    def __init__(self, bus = i2c_bus, address = i2c_address, ilim_hiz_pin = ilim_hiz_pin, otg_en_pin = otg_en_pin):
        self.i2c_bus = bus
        self.i2c_address = address
        self.ilim_hiz_pin = ilim_hiz_pin
        self.otg_en_pin = otg_en_pin

        print("Starting bq25703a Interface on I2C bus " + str(self.i2c_bus) + " with address " + str(hex(self.i2c_address)))

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ilim_hiz_pin, GPIO.OUT)
        GPIO.output(self.ilim_hiz_pin, 0)

        GPIO.setup(self.otg_en_pin, GPIO.OUT)
        GPIO.output(self.otg_en_pin, 0)

        try:
            with SMBus(self.i2c_bus) as smbus:

                # reset the chip
                self.__reset(smbus)

                # Get the manufacturer id
                self.manufacturer_id = smbus.read_byte_data(self.i2c_address, MANUFACTUREID_REG)

                # Get the device id
                self.device_id = smbus.read_byte_data(self.i2c_address, DEVICE_ID_REG)

                # Set the ADC Options (0b01010111)
                smbus.write_byte_data(self.i2c_address, ADCOPTION_0_REG, ADC_ENABLED_BITMASK)

                # CHARGEOPTION0_1_REG settings (all defaults except low power disable)
                # 0b01100010
                self.__clear_byte_bit(smbus, CHARGEOPTION0_1_REG, EN_LWPWR_SHIFT) # disable low power mode

                # CHARGEOPTION0_0_REG settings (all defaults)
                # 0b00001110

                # print(format(smbus.read_byte_data(self.i2c_address, CHARGEOPTION0_1_REG), "08b"))
                # print(format(smbus.read_byte_data(self.i2c_address, CHARGEOPTION0_0_REG), "08b"))

        except:
            self.manufacturer_id = 0
            self.device_id = 0

        if (self.device_id == BQ25703A_DEVICE_ID) and (self.manufacturer_id == BQ25703A_MANUFACTURER_ID):
            self.connected = 1
            print("bq25703a connected")
        else:
            self.connected = 0
            print("bq25703a not found!")


    def GetRegulatorChargingState(self):
        with SMBus(self.i2c_bus) as smbus:
            self.charger_status_21 = smbus.read_byte_data(self.i2c_address, CHARGERSTATUS_1_REG)

        self.charging_status = self.charger_status_21 & CHARGERSTATUS_IN_FCHRG

        return self.charging_status


    def EnableOTG(self):
        GPIO.output(self.otg_en_pin, 1)

        with SMBus(self.i2c_bus) as smbus:

            # disable charge
            val = CHRG_INHIBIT << CHRG_INHIBIT_SHIFT
            self.__update_bits_byte(smbus, CHARGEOPTION0_0_REG, CHRG_INHIBIT_MASK, val)

            # enable OTG mode
            val = OTG_ENABLE << EN_OTG_SHIFT
            self.__update_bits_byte(smbus, CHARGEOPTION3_1_REG, EN_OTG_MASK, val)


    def DisableOTG(self):
        GPIO.output(self.otg_en_pin, 0)

        with SMBus(self.i2c_bus) as smbus:

            # enable charge
            val = CHRG_ENABLE << CHRG_INHIBIT_SHIFT
            self.__update_bits_byte(smbus, CHARGEOPTION0_0_REG, CHRG_INHIBIT_MASK, val)

            # disable OTG mode
            val = OTG_DISABLE << EN_OTG_SHIFT
            self.__update_bits_byte(smbus, CHARGEOPTION3_1_REG, EN_OTG_MASK, val)


    def SetOTGVoltage(self, voltage: int):

        if voltage > OTGVOLT_MAX:
            voltage = OTGVOLT_MAX

        # make sure it's divisible by the count value 
        voltage -= voltage % OTGVOLT_LSB

        if voltage < OTGVOLT_BASE:
            voltage = OTGVOLT_BASE

        val = int((voltage - OTGVOLT_BASE) / OTGVOLT_LSB)

        with SMBus(self.i2c_bus) as smbus:
            self.__update_bits_word(smbus, OTGVOLT_REG, OTGVOLT_MASK, val << OTGVOLT_SHIFT)


    def SetOTGCurrent(self, current: int):

        if current > OTGCURRENT_MAX:
            current = OTGCURRENT_MAX

        # make sure it's divisible by the count value 
        current -= current % OTGCURRENT_LSB

        if current < OTGCURRENT_BASE:
            current = OTGCURRENT_BASE

        val = int((current - OTGCURRENT_BASE) / OTGCURRENT_LSB)
        with SMBus(self.i2c_bus) as smbus:
            self.__update_bits_word(smbus, OTGCURRENT_REG, OTGCURRENT_MASK, val << OTGCURRENT_SHIFT)

        GPIO.output(self.ilim_hiz_pin, 1)


    def SetChargeVoltage(self, voltage: int):

        if voltage > CHARGEVOLT_MAX:
            voltage = CHARGEVOLT_MAX

        # make sure it's divisible by the count value 
        voltage -= voltage % CHARGEVOLT_LSB
        
        if voltage < CHARGEVOLT_MIN:
            voltage = CHARGEVOLT_MIN

        maxVal = int(voltage / CHARGEVOLT_LSB)

        minSysVoltage = voltage - (MINSYSVOLT_LSB * 20)

        if minSysVoltage > MINSYSVOLT_MAX:
            minSysVoltage = MINSYSVOLT_MAX

        # make sure it's divisible by the count value 
        minSysVoltage -= minSysVoltage % MINSYSVOLT_LSB

        if minSysVoltage < MINSYSVOLT_MIN:
            minSysVoltage = MINSYSVOLT_MIN

        minVal = int(minSysVoltage / MINSYSVOLT_LSB)

        with SMBus(self.i2c_bus) as smbus:
            self.__update_bits_word(smbus, MINSYSVOLT_REG, MINSYSVOLT_MASK, minVal << MINSYSVOLT_SHIFT)
            self.__update_bits_word(smbus, CHARGEVOLT_REG, CHARGEVOLT_MASK, maxVal << CHARGEVOLT_SHIFT)


    def SetChargeCurrent(self, current: int):

        if current > CHARGECURRENT_MAX:
            current = CHARGECURRENT_MAX
        
        # make sure it's divisible by the count value 
        current -= current % CHARGECURRENT_LSB

        if current < CHARGECURRENT_BASE:
            current = CHARGECURRENT_BASE

        self.max_charge_current = current

        val = int((current - CHARGECURRENT_BASE) / CHARGECURRENT_LSB)

        with SMBus(self.i2c_bus) as smbus:
            self.__update_bits_word(smbus, CHARGECURRENT_REG, CHARGECURRENT_MASK, val << CHARGECURRENT_SHIFT)

        GPIO.output(self.ilim_hiz_pin, 1)


    def ReadChargerStatus(self):
        with SMBus(self.i2c_bus) as smbus:
            self.charger_status_20 = smbus.read_byte_data(self.i2c_address, CHARGERSTATUS_0_REG)
            self.charger_status_21 = smbus.read_byte_data(self.i2c_address, CHARGERSTATUS_1_REG)


    def ReadADC(self):
        with SMBus(self.i2c_bus) as smbus:
            self.__read_adc(smbus)


    def __read_adc(self, smbus: SMBus):
        # Perform single conversion
        smbus.write_byte_data(self.i2c_address, ADCOPTION_1_REG, ADC_START_CONVERSION_MASK)
        data = smbus.read_byte_data(self.i2c_address, ADCOPTION_1_REG)
        while data & ADC_START_MASK:
            time.sleep(0.05)
            data = smbus.read_byte_data(self.i2c_address, ADCOPTION_1_REG)

        data = smbus.read_byte_data(self.i2c_address, ADCVBAT_REG)
        self.vbat_voltage = (data * ADCVBAT_LSB) + ADCVBAT_BASE

        data = smbus.read_byte_data(self.i2c_address, ADCVSYS_REG)
        self.vsys_voltage = (data * ADCVSYS_LSB) + ADCVSYS_BASE

        data = smbus.read_byte_data(self.i2c_address, ADCIBAT_CHG_REG)
        self.charge_current = (data * ADCIBAT_CHG_LSB) + ADCIBAT_CHG_BASE

        data = smbus.read_byte_data(self.i2c_address, ADCIBUS_REG)
        self.input_current = (data * ADCIBUS_LSB) + ADCIBUS_BASE

        data = smbus.read_byte_data(self.i2c_address, ADCVBUS_REG)
        self.vbus_voltage = (data * ADCVBUS_LSB) + ADCVBUS_BASE


    def __set_byte_bit(self, smbus: SMBus, reg: int, bit: int):
        tmp = smbus.read_byte_data(self.i2c_address, reg)
        mask = 1 << bit
        smbus.write_byte_data(self.i2c_address, reg, tmp | mask)


    def __clear_byte_bit(self, smbus: SMBus, reg: int, bit: int):
        tmp = smbus.read_byte_data(self.i2c_address, reg)
        mask = ~(1 << bit)
        smbus.write_byte_data(self.i2c_address, reg, tmp & mask)


    def __update_bits_byte(self, smbus: SMBus, reg: int, mask: int, data: int):
        tmp = smbus.read_byte_data(self.i2c_address, reg)
        tmp = tmp & ~mask
        tmp = tmp | (data & mask)
        smbus.write_byte_data(self.i2c_address, reg, tmp)


    def __update_bits_word(self, smbus: SMBus, reg: int, mask: int, data: int):
        tmp = smbus.read_word_data(self.i2c_address, reg)
        tmp = tmp & ~mask
        tmp = tmp | (data & mask)
        smbus.write_word_data(self.i2c_address, reg, tmp)


    def __reset(self, smbus: SMBus):
        self.__set_byte_bit(smbus, CHARGEOPTION3_1_REG, RESET_REG_SHIFT)

        # wait for chip reset done (bit returns to zero)
        data = smbus.read_byte_data(self.i2c_address, CHARGEOPTION3_1_REG)
        while data & RESET_REG_MASK:
            time.sleep(0.05)
            data = smbus.read_byte_data(self.i2c_address, CHARGEOPTION3_1_REG)
