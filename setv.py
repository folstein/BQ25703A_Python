import os

import argparse
import time

from bq25703a import bq25703a
import RPi.GPIO as GPIO

clear = lambda: os.system('clear')

def main():
    parser = argparse.ArgumentParser(description="Voltage Setter")
    parser.add_argument("-v", default=15.1, type=float)
    parser.add_argument("-c", default=3.0, type=float)
    parser.add_argument("--otg", action=argparse.BooleanOptionalAction)
    parser.add_argument("--cleanup", action=argparse.BooleanOptionalAction)

    args = parser.parse_args()

    current = args.c
    voltage = args.v

    # convert V and A to mV and mA
    round(current, 3)
    round(voltage, 3)
    current = int(current * 1000)
    voltage = int(voltage * 1000)

    bq = bq25703a()

    if bq.connected:
        if args.otg:
            mainOTG(bq, voltage, current)
        else:
            mainCharge(bq, voltage, current)

    # reset all GPIO in INPUT
    if args.cleanup:
        print("GPIO Cleanup...")
        GPIO.cleanup()


def mainCharge(bq: bq25703a, v: int, c: int):
    # bq.DisableOTG()

    print(f"Charging {v/1000:.03f}V {c/1000:.03f}mA")
    
    bq.SetChargeVoltage(v)
    bq.SetChargeCurrent(c)

    while True:
        try:
            # send again to ping watchdog timer
            bq.SetChargeVoltage(v)
            bq.SetChargeCurrent(c)

            bq.ReadADC()

            clear()
            print(f"VBAT / VBUS  {bq.vbat_voltage:6d} {bq.vbus_voltage:6d}")
            print(f"IInp / IChg  {bq.input_current:6d} {bq.charge_current:6d}")

            bq.ReadChargerStatus()

            print("")

            print(f"Status 0x20       {bq.charger_status_20:08b}")
            print(f"Status 0x21       {bq.charger_status_21:08b}")

            time.sleep(0.25)

        except KeyboardInterrupt:
            print("Stopping...")
            break
    

def mainOTG(bq: bq25703a, v: int, c: int):
    bq.EnableOTG()

    print(f"OTG {v/1000:.03f}V {c/1000:.03f}A")
    
    bq.SetOTGVoltage(v)
    bq.SetOTGCurrent(c)

    while True:
        try:
            bq.ReadADC()

            clear()
            print(f"VBAT / VBUS  {bq.vbat_voltage:6d} {bq.vbus_voltage:6d}")
            print(f"IInp / IChg  {bq.input_current:6d} {bq.charge_current:6d}")

            bq.ReadChargerStatus()

            print("")

            print(f"Status 0x20       {bq.charger_status_20:08b}")
            print(f"Status 0x21       {bq.charger_status_21:08b}")

            time.sleep(.25)

        except KeyboardInterrupt:
            print("Stopping...")
            break


if __name__ == '__main__':
    main()
