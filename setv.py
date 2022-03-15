import argparse
import time

from bq25703a import bq25703a
import RPi.GPIO as GPIO


def main():
    parser = argparse.ArgumentParser(description="Voltage Setter")
    parser.add_argument("--otg", action=argparse.BooleanOptionalAction)
    parser.add_argument("-v", default=15.1, type=float)
    parser.add_argument("-c", default=3.0, type=float)
    parser.add_argument("--cleanup", action=argparse.BooleanOptionalAction)

    args = parser.parse_args()

    if args.otg:
        mainOTG(args.v, args.c)
    else:
        mainCharge(args.v, args.c)

    # reset all GPIO in INPUT
    if args.cleanup:
        print("GPIO Cleanup...")
        GPIO.cleanup()
        time.sleep(0.25)


def mainCharge(v: float, c: float):
    bq = bq25703a()
    bq.EnableOTG(0)

    print(f"Charging {v}v {c}a")
    
    bq.Set_Charge_Voltage(v)
    bq.Set_Charge_Current(c)

    while True:
        try:
            # send again to ping watchdog timer
            bq.Set_Charge_Voltage(v)
            bq.Set_Charge_Current(c)

            bq.ReadADC()
            print(f"VBAT / VBUS  {bq.Get_VBAT_ADC_Reading():6.03f} {bq.Get_VBUS_ADC_Reading():6.03f}")
            # print(f"Regs {bq.Get_Regulator_Charging_State()}")
            print(f"IInp / IChg  {bq.Get_Input_Current_ADC_Reading():6.03f} {bq.Get_Charge_Current_ADC_Reading():6.03f}")
            # print(f"MaxC {bq.Get_Max_Charge_Current()}")

            bq.Read_Charger_Status()

            print("")

            time.sleep(.25)
        except KeyboardInterrupt:
            print("Stopping...")
            break

    time.sleep(0.25)


def mainOTG(v: float, c: float):
    bq = bq25703a()
    bq.EnableOTG(1)

    print(f"OTG {v}v {c}a")
    
    bq.Set_OTG_Voltage(v)
    bq.Set_OTG_Current(c)

    while True:
        try:
            bq.ReadADC()
            print(f"VBAT / VBUS  {bq.Get_VBAT_ADC_Reading():6.03f} {bq.Get_VBUS_ADC_Reading():6.03f}")
            print(f"IInp / IChg  {bq.Get_Input_Current_ADC_Reading():6.03f} {bq.Get_Charge_Current_ADC_Reading():6.03f}")
            # print(f"IDischarge   {bq.Get_Discharge_Current():6.03f}")

            bq.Read_Charger_Status()

            print("")

            time.sleep(.25)
        except KeyboardInterrupt:
            print("Stopping...")
            break

    time.sleep(0.25)


if __name__ == '__main__':
    main()
