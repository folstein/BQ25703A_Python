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

MANUFACTURER_ID_ADDR        = 0x2E
DEVICE_ID_ADDR              = 0x2F

EN_LWPWR                    = 0b0
EN_OOA                      = 0b1

ADC_ENABLED_BITMASK         = 0b01010111
ADC_START_CONVERSION_MASK   = 0b01100000
ADC_CONT_CONVERSION_MASK    = 0b10100000

#Max voltage register 1 values
MAX_VOLT_ADD_16384_MV       = 0b01000000
MAX_VOLT_ADD_8192_MV        = 0b00100000
MAX_VOLT_ADD_4096_MV        = 0b00010000
MAX_VOLT_ADD_2048_MV        = 0b00001000
MAX_VOLT_ADD_1024_MV        = 0b00000100
MAX_VOLT_ADD_512_MV         = 0b00000010
MAX_VOLT_ADD_256_MV         = 0b00000001

#Max voltage register 2 values
MAX_VOLT_ADD_128_MV         = 0b10000000
MAX_VOLT_ADD_64_MV          = 0b01000000
MAX_VOLT_ADD_32_MV          = 0b00100000
MAX_VOLT_ADD_16_MV          = 0b00010000

#Minimum system voltage register values
MIN_VOLT_ADD_8192_MV        = 0b00100000
MIN_VOLT_ADD_4096_MV        = 0b00010000
MIN_VOLT_ADD_2048_MV        = 0b00001000
MIN_VOLT_ADD_1024_MV        = 0b00000100
MIN_VOLT_ADD_512_MV         = 0b00000010
MIN_VOLT_ADD_256_MV         = 0b00000001

VBUS_ADC_SCALE              = 0.064
VBUS_ADC_OFFSET             = 3.2

PSYS_ADC_SCALE              = 0.012

VSYS_ADC_SCALE              = 0.064
VSYS_ADC_OFFSET             = 2.88

VBAT_ADC_SCALE              = 0.064
VBAT_ADC_OFFSET             = 2.88

ICHG_ADC_SCALE              = 0.064
IDCHG_ADC_SCALE             = 0.256

IIN_ADC_SCALE               = 0.050

CHARGEVOLT_MAX = 19200
CHARGEVOLT_MIN = 1024

CHARGECURRENT_MAX = CHARGECURRENT_BASE + (0x7F * CHARGECURRENT_LSB)

OTGVOLT_MAX = OTGVOLT_BASE + (0xFF * OTGVOLT_LSB)
OTGCURRENT_MAX = OTGCURRENT_BASE + (0x7F * OTGCURRENT_LSB)

class bq25703a:
    i2c_address = BQ25703A_I2C_ADDRESS
    i2c_bus = 1
    ilim_hiz_pin = 17
    otg_en_pin = 16

    connected = 0
    charging_status = 0
    vbat_voltage = 0
    vbus_voltage = 0
    vsys_voltage = 0
    input_current = 0
    charge_current = 0
    discharge_current = 0
    max_charge_current_ma = 0

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
                # Get the manufacturer id
                manufacturer_id = smbus.read_byte_data(self.i2c_address, MANUFACTURER_ID_ADDR)
                # Get the device id
                device_id = smbus.read_byte_data(self.i2c_address, DEVICE_ID_ADDR)
                # Set the ADC Options (0b01010111)
                smbus.write_byte_data(self.i2c_address, ADCOPTION_0_REG, ADC_ENABLED_BITMASK)

                val  = WDTMR_ADJ_5S << WDTMR_ADJ_SHIFT
                val += PWM_FREQ_800KHZ << PWM_FREQ_SHIFT
                val += OOA_ENABLE << EN_OOA_SHIFT
                smbus.write_byte_data(self.i2c_address, CHARGEOPTION0_1_REG, val)

                charge_option_0_register_2_value = 0b00001110
                print(f"{charge_option_0_register_2_value:08b}")
                print(f"{val:08b}")
                smbus.write_byte_data(self.i2c_address, CHARGEOPTION0_0_REG, charge_option_0_register_2_value)                

                # print(format(smbus.read_byte_data(self.i2c_address, CHARGEOPTION0_1_REG), "08b"))
                # print(format(smbus.read_byte_data(self.i2c_address, CHARGEOPTION0_0_REG), "08b"))

        except:
            manufacturer_id = 0
            device_id = 0

        if ((device_id == BQ25703A_DEVICE_ID) and (manufacturer_id == BQ25703A_MANUFACTURER_ID)):
            self.connected = 1
            print("bq25703a connected")
        else:
            self.connected = 0
            print("bq25703a not found!")

    # @brief Returns whether the regulator is charging
    # @retval uint8_t 1 if charging, 0 if not charging
    def Get_Regulator_Charging_State(self):
        with SMBus(self.i2c_bus) as smbus:
            data = smbus.read_byte_data(self.i2c_address, CHARGERSTATUS_1_REG)
            # print(f"{data:08b}")

        if (data & CHARGERSTATUS_IN_FCHRG):
            self.charging_status = 1
        else:
            self.charging_status = 0

        return self.charging_status

    def EnableOTG(self, enable: int):
        GPIO.output(self.otg_en_pin, enable)

        if enable:
            # disable charge
            val = CHRG_INHIBIT << CHRG_INHIBIT_SHIFT
            self.update_bits_byte(CHARGEOPTION0_0_REG, CHRG_INHIBIT_MASK, val)

            # enable OTG mode
            val = OTG_ENABLE << EN_OTG_SHIFT
            self.update_bits_byte(CHARGEOPTION3_1_REG, EN_OTG_MASK, val)
        else:
            # enable charge
            val = CHRG_ENABLE << CHRG_INHIBIT_SHIFT
            self.update_bits_byte(CHARGEOPTION0_0_REG, CHRG_INHIBIT_MASK, val)

            # disable OTG mode
            val = OTG_DISABLE << EN_OTG_SHIFT
            self.update_bits_byte(CHARGEOPTION3_1_REG, EN_OTG_MASK, val)


    def Set_OTG_Voltage(self, voltage: float):
        # convert to mV
        round(voltage, 3)
        voltage = int(voltage * 1000)

        if voltage > OTGVOLT_MAX:
            voltage = OTGVOLT_MAX
        if voltage < OTGVOLT_BASE:
            voltage = OTGVOLT_BASE

        # increment down until voltage is divisiable by OTGVOLT_LSB mV
        while (voltage % OTGVOLT_LSB) != 0:
            voltage = voltage - 1

        val = int((voltage - OTGVOLT_BASE) / OTGVOLT_LSB)
        self.update_bits_word(OTGVOLT_REG, OTGVOLT_MASK, val << OTGVOLT_SHIFT)

    def Set_OTG_Current(self, current: float):
        # convert to mA
        round(current, 3)
        current = int(current * 1000)

        if (current > OTGCURRENT_MAX):
            current = OTGCURRENT_MAX
        if (current < OTGCURRENT_BASE):
            current = OTGCURRENT_BASE

        # Make sure the value is divisiable OTGCURRENT_LSB mA
        while (current % OTGCURRENT_LSB) != 0:
            current = current - 1

        val = int((current - OTGCURRENT_BASE) / OTGCURRENT_LSB)
        self.update_bits_word(OTGCURRENT_REG, OTGCURRENT_MASK, val << OTGCURRENT_SHIFT)

        GPIO.output(self.ilim_hiz_pin, 1)

    def Set_Charge_Voltage(self, voltage: float):
        # convert to mV
        round(voltage, 3)
        voltage = int(voltage * 1000)

        if (voltage > CHARGEVOLT_MAX):
            voltage = CHARGEVOLT_MAX
        if (voltage < CHARGEVOLT_MIN):
            voltage = CHARGEVOLT_MIN

        # increment down until voltage is divisiable by CHARGEVOLT_LSB mV
        while (voltage % CHARGEVOLT_LSB) != 0:
            voltage = voltage - 1
        maxVal = int(voltage / CHARGEVOLT_LSB)

        minVal = voltage - 5000

        # increment down until val is divisiable by MINSYSVOLT_LSB
        while (minVal % MINSYSVOLT_LSB) != 0:
            minVal = minVal - 1
        if (minVal < CHARGEVOLT_MIN):
            minVal = CHARGEVOLT_MIN
        minVal = int(minVal / MINSYSVOLT_LSB)

        self.update_bits_word(MINSYSVOLT_REG, MINSYSVOLT_MASK, minVal << MINSYSVOLT_SHIFT)
        self.update_bits_word(CHARGEVOLT_REG, CHARGEVOLT_MASK, maxVal << CHARGEVOLT_SHIFT)

    def Set_Charge_Current(self, current: float):
        # convert to mA
        round(current, 3)
        current = int(current * 1000)

        if current > CHARGECURRENT_MAX:
            current = CHARGECURRENT_MAX
        if current < CHARGECURRENT_BASE:
            current = CHARGECURRENT_BASE
        
        # Make sure the value is divisiable CHARGECURRENT_LSB mA
        while (current % CHARGECURRENT_LSB) != 0:
            current = current - 1

        self.max_charge_current_ma = current

        val = int((current - CHARGECURRENT_BASE) / CHARGECURRENT_LSB)
        self.update_bits_word(CHARGECURRENT_REG, CHARGECURRENT_MASK, val << CHARGECURRENT_SHIFT)

        # with SMBus(self.i2c_bus) as smbus:
        #     print(format(smbus.read_word_data(self.i2c_address, INPUTCURRENTLIM_REG), "08b"))

        GPIO.output(self.ilim_hiz_pin, 1)

    def Read_Charger_Status(self):
        with SMBus(self.i2c_bus) as smbus:
            data = smbus.read_byte_data(self.i2c_address, CHARGERSTATUS_1_REG)
            print("Charge Status Address 0x21 = " + format(data, "08b"))
            data = smbus.read_byte_data(self.i2c_address, CHARGERSTATUS_0_REG)
            print("Charge Status Address 0x20 = " + format(data, "08b"))

    def ReadADC(self):
        self.__read_adc()

    # @brief Gets VBAT voltage that was read in from the ADC on the regulator
    # @retval VBAT voltage in volts
    def Get_VBAT_ADC_Reading(self):
        #self.__read_adc()
        return self.vbat_voltage

    # @brief Gets VBUS voltage that was read in from the ADC on the regulator
    # @retval VBUS voltage in volts
    def Get_VBUS_ADC_Reading(self):
        #self.__read_adc()
        return self.vbus_voltage

    # @brief Gets Input Current that was read in from the ADC on the regulator
    # @retval Input Current in amps
    def Get_Input_Current_ADC_Reading(self):
        #self.__read_adc()
        return self.input_current

    # @brief Gets Charge Current that was read in from the ADC on the regulator
    # @retval Charge Current in amps
    def Get_Charge_Current_ADC_Reading(self):
        #self.__read_adc()
        return self.charge_current

    def Get_Discharge_Current(self):
        return self.discharge_current

    # @brief Gets the max output current for charging
    # @retval Max Charge Current in miliamps
    def Get_Max_Charge_Current(self):
        #self.__read_adc()
        return self.max_charge_current_ma

    def __read_adc(self):
        with SMBus(self.i2c_bus) as smbus:
            # Perform single conversion
            smbus.write_byte_data(self.i2c_address, (ADCOPTION_1_REG), ADC_START_CONVERSION_MASK)

            conversion_finished = 0
            while (conversion_finished == 0):
                data = smbus.read_byte_data(self.i2c_address, (ADCOPTION_1_REG))
                conversion_finished = (data and (1<<6))
                time.sleep(0.05)

            data = smbus.read_byte_data(self.i2c_address, ADCVBAT_REG)
            self.vbat_voltage = (data * VBAT_ADC_SCALE) + VBAT_ADC_OFFSET

            data = smbus.read_byte_data(self.i2c_address, ADCVSYS_REG)
            self.vsys_voltage = (data * VSYS_ADC_SCALE) + VSYS_ADC_OFFSET

            data = smbus.read_byte_data(self.i2c_address, ADCIBAT_CHG_REG)
            self.charge_current = data * ICHG_ADC_SCALE

            data = smbus.read_byte_data(self.i2c_address, ADCIBAT_DSG_REG)
            self.discharge_current = data * IDCHG_ADC_SCALE

            data = smbus.read_byte_data(self.i2c_address, ADCIBUS_REG)
            self.input_current = data * IIN_ADC_SCALE

            data = smbus.read_byte_data(self.i2c_address, ADCVBUS_REG)
            self.vbus_voltage = (data * VBUS_ADC_SCALE) + VBUS_ADC_OFFSET

    def set_byte_bit(self, reg, bit):
        with SMBus(self.i2c_bus) as smbus:
            tmp = smbus.read_byte_data(self.i2c_address, reg)
            mask = 1 << bit
            smbus.write_byte_data(self.i2c_address, reg, tmp | mask)

    def clear_byte_bit(self, reg, bit):
        with SMBus(self.i2c_bus) as smbus:
            tmp = smbus.read_byte_data(self.i2c_address, reg)
            mask = ~(1 << bit)
            smbus.write_byte_data(self.i2c_address, reg, tmp & mask)

    def update_bits_byte(self, reg, mask, data):
        with SMBus(self.i2c_bus) as smbus:
            tmp = smbus.read_byte_data(self.i2c_address, reg)

            tmp = tmp & ~mask
            tmp = tmp | (data & mask)

            smbus.write_byte_data(self.i2c_address, reg, tmp)

    def update_bits_word(self, reg, mask, data):
        with SMBus(self.i2c_bus) as smbus:
            tmp = smbus.read_word_data(self.i2c_address, reg)

            # print(format(tmp, '016b'))

            tmp = tmp & ~mask
            tmp = tmp | (data & mask)

            print(format(tmp, '016b'))

            smbus.write_word_data(self.i2c_address, reg, tmp)

            # tmp = smbus.read_word_data(self.i2c_address, reg)
            # print(format(tmp, '016b'))
