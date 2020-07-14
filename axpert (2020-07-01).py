import crcmod
import os
import re
#import serial
import signal
import sys
import time
from binascii import unhexlify
import mysql.connector
import datetime
#from curtsies import Input
#import sonoff
#import config

connection = "USB"

mode0 = -1
mode1 = -1
load = 0
wake_up_start = 0
parrallel_num = 0


# Axpert Commands and examples
# Q1		# Undocumented command: LocalInverterStatus (seconds from absorb), ParaExistInfo (seconds from end of Float), SccOkFlag, AllowSccOnFlag, ChargeAverageCurrent, SCC PWM Temperature, Inverter Temperature, Battery Temperature, Transformer Temperature, GPDAT, FanLockStatus, FanPWMDuty, FanPWM, SCCChargePowerWatts, ParaWarning, SYNFreq, InverterChargeStatus
# QPI            # Device protocol ID inquiry
# QID            # The device serial number inquiry
# QVFW           # Main CPU Firmware version inquiry
# QVFW2          # Another CPU Firmware version inquiry
# QFLAG          # Device flag status inquiry
# QPIGS          # Device general status parameters inquiry
# GridVoltage, GridFrequency, OutputVoltage, OutputFrequency, OutputApparentPower, OutputActivePower, OutputLoadPercent, BusVoltage, BatteryVoltage, BatteryChargingCurrent, BatteryCapacity, InverterHeatSinkTemperature, PV-InputCurrentForBattery, PV-InputVoltage, BatteryVoltageFromSCC, BatteryDischargeCurrent, DeviceStatus,
# QMOD           # Device mode inquiry P: PowerOnMode, S: StandbyMode, L: LineMode, B: BatteryMode, F: FaultMode, H: PowerSavingMode
# QPIWS          # Device warning status inquiry: Reserved, InverterFault, BusOver, BusUnder, BusSoftFail, LineFail, OPVShort, InverterVoltageTooLow, InverterVoltageTooHIGH, OverTemperature, FanLocked, BatteryVoltageHigh, BatteryLowAlarm, Reserved, ButteryUnderShutdown, Reserved, OverLoad, EEPROMFault, InverterSoftFail, SelfTestFail, OPDCVoltageOver, BatOpen, CurrentSensorFail, BatteryShort, PowerLimit, PVVoltageHigh, MPPTOverloadFault, MPPTOverloadWarning, BatteryTooLowToCharge, Reserved, Reserved
# QDI            # The default setting value information
# QMCHGCR        # Enquiry selectable value about max charging current
# QMUCHGCR       # Enquiry selectable value about max utility charging current
# QBOOT          # Enquiry DSP has bootstrap or not
# QOPM           # Enquiry output mode
# QPIRI          # Device rating information inquiry - nefunguje
# QPGS0          # Parallel information inquiry
# TheParallelNumber, SerialNumber, WorkMode, FaultCode, GridVoltage, GridFrequency, OutputVoltage, OutputFrequency, OutputAparentPower, OutputActivePower, LoadPercentage, BatteryVoltage, BatteryChargingCurrent, BatteryCapacity, PV-InputVoltage, TotalChargingCurrent, Total-AC-OutputApparentPower, Total-AC-OutputActivePower, Total-AC-OutputPercentage, InverterStatus, OutputMode, ChargerSourcePriority, MaxChargeCurrent, MaxChargerRange, Max-AC-ChargerCurrent, PV-InputCurrentForBattery, BatteryDischargeCurrent
# QBV		# Compensated Voltage, SoC
# PEXXX          # Setting some status enable
# PDXXX          # Setting some status disable
# PF             # Setting control parameter to default value
# FXX            # Setting device output rating frequency
# POP02          # set to SBU
# POP01          # set to Solar First
# POP00          # Set to UTILITY
# PBCVXX_X       # Set battery re-charge voltage
# PBDVXX_X       # Set battery re-discharge voltage
# PCP00          # Setting device charger priority: Utility First
# PCP01          # Setting device charger priority: Solar First
# PCP02          # Setting device charger priority: Solar and Utility
# PGRXX          # Setting device grid working range
# PBTXX          # Setting battery type
# PSDVXX_X       # Setting battery cut-off voltage
# PCVVXX_X       # Setting battery C.V. charging voltage
# PBFTXX_X       # Setting battery float charging voltage
# PPVOCKCX       # Setting PV OK condition
# PSPBX          # Setting solar power balance
# MCHGC0XX       # Setting max charging Current          M XX
# MUCHGC002      # Setting utility max charging current  0 02
# MUCHGC010      # Setting utility max charging current  0 10
# MUCHGC020      # Setting utility max charging current  0 20
# MUCHGC030      # Setting utility max charging current  0 30
# POPMMX         # Set output mode       M 0:single, 1: parrallel, 2: PH1, 3: PH2, 4: PH3

# notworking
# PPCP000        # Setting parallel device charger priority: UtilityFirst - notworking
# PPCP001        # Setting parallel device charger priority: SolarFirst - notworking
# PPCP002        # Setting parallel device charger priority: OnlySolarCharging - notworking

# print(mydb)

# client = InfluxDBClient(host='localhost', port=8086)
# client.create_database('Inverter')
# measurement_name = 'data'


def handler(signum, frame):
    print
    'Signal handler called with signal', signum
    raise Exception("Handler")


try:
    usb0 = os.open('/dev/hidraw0', os.O_RDWR | os.O_NONBLOCK)
#    usb1 = os.open('/dev/hidraw1', os.O_RDWR | os.O_NONBLOCK)

except Exception as e:
    print("error open USB port: " + str(e))
    exit()


def connect():
    try:
        usb0 = os.open('/dev/hidraw0', os.O_RDWR | os.O_NONBLOCK)
    #    usb1 = os.open('/dev/hidraw1', os.O_RDWR | os.O_NONBLOCK)

    except Exception as e:
        print("error open USB port: " + str(e))
        exit()


def get_data(command, inverter):
    # collect data from axpert inverter
    global mode0
    global mode1
    global load
    status = -1
    global parrallel_num
    global nums_global

    if inverter == 0: device = usb0
    #    if inverter == 1: device = usb1
    try:
        data = "{"
        response = serial_command(command, device)
        response_num = re.sub('[^0-9. ]', '', response)
        if command == "QPGS0":
            response.rstrip()
            response_num.rstrip()
            nums = response_num.split(' ', 99)
            nums_mode = response.split(' ', 99)

            if nums_mode[2] == "L":
                data += "Gridmode0:1"
                data += ",Solarmode0:0"
                mode0 = 0
            elif nums_mode[2] == "B":
                data += "Gridmode0:0"
                data += ",Solarmode0:1"
                mode0 = 1
            elif nums_mode[2] == "S":
                data += "Gridmode0:0"
                data += ",Solarmode0:0"
                mode0 = 2
            elif nums_mode[2] == "F":
                data += "Gridmode0:0"
                data += ",Solarmode0:0"
                mode0 = 3

                data += ",The_parallel_num0:" + nums[0]
                data += ",Serial_number0:" + nums[1]
                data += ",Fault_code0:" + nums[3]
                data += ",Load_percentage0:" + nums[10]
                data += ",Total_charging_current:" + nums[15]
                data += ",Total_AC_output_active_power:" + nums[17]
                data += ",Total_AC_output_apparent_power:" + nums[16]
                data += ",Total_AC_output_percentage:" + nums[18]
                data += ",Inverter_Status0:" + nums[19]
                data += ",Output_mode0:" + nums[20]
                data += ",Charger_source_priority0:" + nums[21]
                data += ",Max_Charger_current0:" + nums[22]
                data += ",Max_Charger_range0:" + nums[23]
                data += ",Max_AC_charger_current0:" + nums[24]
                data += ",Inverter_mode0:" + str(mode0)
                parrallel_num = int(nums[0])
                load = int(nums[17])

        elif command == "QPGS1":
            response.rstrip()
            response_num.rstrip()
            nums = response_num.split(' ', 99)
            nums_mode = response.split(' ', 99)
            if nums_mode[2] == "L":
                data += "Gridmode1:1"
                data += ",Solarmode1:0"
                mode1 = 0
            elif nums_mode[2] == "B":
                data += "Gridmode1:0"
                data += ",Solarmode1:1"
                mode1 = 1
            elif nums_mode[2] == "S":
                data += "Gridmode1:0"
                data += ",Solarmode1:0"
                mode1 = 2
            elif nums_mode[2] == "F":
                data += "Gridmode1:0"
                data += ",Solarmode1:0"
                mode1 = 3

            data += ",The_parallel_num1:" + nums[0]
            data += ",Serial_number1:" + nums[1]
            data += ",Fault_code1:" + nums[3]
            data += ",Load_percentage1:" + nums[10]
            data += ",Total_charging_current:" + nums[15]
            data += ",Total_AC_output_active_power:" + nums[17]
            data += ",Total_AC_output_apparent_power:" + nums[16]
            data += ",Total_AC_output_percentage:" + nums[18]
            data += ",Inverter_Status1:" + nums[19]
            data += ",Output_mode1:" + nums[20]
            data += ",Charger_source_priority1:" + nums[21]
            data += ",Max_Charger_current1:" + nums[22]
            data += ",Max_Charger_range1:" + nums[23]
            data += ",Max_AC_charger_current1:" + nums[24]
            data += ",Inverter_mode1:" + str(mode1)
            parrallel_num = int(nums[0])
            load = int(nums[17])

        elif command == "QPIGS":
            response_num.rstrip()
            nums = response_num.split(' ', 99)
            data += "Grid_voltage" + str(inverter) + ":" + nums[0]
            data += ",Grid_frequency" + str(inverter) + ":" + nums[1]
            data += ",AC_output_voltage" + str(inverter) + ":" + nums[2]
            data += ",AC_output_frequency" + str(inverter) + ":" + nums[3]
            data += ",AC_output_apparent_power" + str(inverter) + ":" + nums[4]
            data += ",AC_output_active_power" + str(inverter) + ":" + nums[5]
            data += ",Output_Load_Percent" + str(inverter) + ":" + nums[6]
            data += ",Bus_voltage" + str(inverter) + ":" + nums[7]
            data += ",Battery_voltage" + str(inverter) + ":" + nums[8]
            data += ",Battery_charging_current" + str(inverter) + ":" + nums[9]
            data += ",Battery_capacity" + str(inverter) + ":" + nums[10]
            data += ",Inverter_heatsink_temperature" + str(inverter) + ":" + nums[11]
            data += ",PV_input_current_for_battery" + str(inverter) + ":" + nums[12]
            data += ",PV_Input_Voltage" + str(inverter) + ":" + nums[13]
            data += ",Battery_voltage_from_SCC" + str(inverter) + ":" + nums[14]
            data += ",Battery_discharge_current" + str(inverter) + ":" + nums[15]
            data += ",Device_status" + str(inverter) + ":" + nums[16]

            nums_global = nums
            PVpower = float(nums[8]) * float(nums[12])
            try:
                mydb = mysql.connector.connect(host = 'localhost', user = 'inverter', password = '4a2fc629',
                                               database = 'inverter')
                cursor = mydb.cursor()
                cursor.execute(
                    "INSERT IGNORE INTO data (time, Output_Power, Battery_Voltage, PV_Input_Current, Charge_Current, Discharge_Current, PV_Power) VALUES (NOW(), %s, %s, %s, %s, %s,%s)",
                    (nums[5], nums[8], nums[12], nums[9], nums[15], PVpower))
                mydb.commit()
                cursor.close()
                mydb.close()
            except Exception as e:
                print("Databae Error: ", str(e))

        elif command == "Q1":
            response_num.rstrip()
            nums = response_num.split(' ', 99)
            data += "SCCOkFlag" + str(inverter) + ":" + nums[2]
            data += ",AllowSCCOkFlag" + str(inverter) + ":" + nums[3]
            data += ",ChargeAverageCurrent" + str(inverter) + ":" + nums[4]
            data += ",SCCPWMTemperature" + str(inverter) + ":" + nums[5]
            data += ",InverterTemperature" + str(inverter) + ":" + nums[6]
            data += ",BatteryTemperature" + str(inverter) + ":" + nums[7]
            data += ",TransformerTemperature" + str(inverter) + ":" + nums[8]
            data += ",GPDAT" + str(inverter) + ":" + nums[9]
            data += ",FanLockStatus" + str(inverter) + ":" + nums[10]
            data += ",FanPWM" + str(inverter) + ":" + nums[12]
            data += ",SCCChargePower" + str(inverter) + ":" + nums[13]
            data += ",ParaWarning" + str(inverter) + ":" + nums[14]
            data += ",InverterChargeStatus" + str(inverter) + ":" + nums[16]

        elif command == "QBV":
            response_num.rstrip()
            nums = response_num.split(' ', 99)
            data += "Battery_voltage_compensated" + str(inverter) + ":" + nums[0]
            data += ",SoC" + str(inverter) + ":" + nums[1]
        else:
            return ''
        data += "}"

    except Exception as e:
        print("error parsong inverter data...: ", str(e))
        print("problem command: " + command + ": " + response)
        usb0.close
        connect()
        return ''

    return data


def set_charge_current():
    # Automaticly adjust axpert inverter grid charging current

    # 2A = 100W, 10A = 500W, 20A = 1000W, 30 = 1500W
    # load >3000W -> 02A
    # load <3000W -> 10A
    # load <2000W -> 20A
    # load <1000W -> 30A

    try:
        if (connection == "serial" and ser.isOpen() or connection == "USB"):
            current = 0
            load_power = 0
            response = serial_command("QPGS0", usb0)
            if "NAKss" in response:
                if connection == "serial": time.sleep(0.5)
                return ''
            response.rstrip()
            nums = response.split(' ', 99)
            current = int(nums[24])
            response = serial_command("QPIGS", usb0)
            if "NAKss" in response:
                if connection == "serial": time.sleep(0.5)
                return ''
            response.rstrip()
            nums = response.split(' ', 99)
            load_power = int(nums[5])
            print
            load_power
            if load_power > 3000:
                if not current == 2:
                    current = 2
                    response = serial_command("MUCHGC002", usb0)
            elif load_power > 2000:
                if not current == 10:
                    current = 10
                    response = serial_command("MUCHGC010", usb0)
            elif load_power > 1000:
                if not current == 20:
                    current = 20
                    response = serial_command("MUCHGC020", usb0)
            else:
                if not current == 30:
                    current = 30
                    response = serial_command("MUCHGC030", usb0)
            print
            current
            if "NAKss" in response:
                if connection == "serial": time.sleep(0.5)
                return ''

        elif (connection == "serial"):
            ser.close()
            print("cannot use serial port ...")
            return ""

    except Exception as e:
        print("error parsing inverter data...: " + str(e))
        return ''

    return current


def get_output_source_priority():
    # get inverter output mode priority
    output_source_priority = "8"
    try:
        if (connection == "serial" and ser.isOpen() or connection == "USB"):
            response = serial_command("QPIRI", usb0)
            if "NAKss" in response:
                if connection == "serial": time.sleep(0.5)
                return ""
            response.rstrip()
            nums = response.split(' ', 99)
            output_source_priority = nums[16]

        elif (connection == "serial"):
            ser.close()
            print
            "cannot use serial port ..."
            return ""

    except Exception as e:
        print("error parsing inverter data...: " + str(e))
        return ""

    return output_source_priority


def get_charger_source_priority():
    # get inverter charger mode priority
    charger_source_priority = "8"
    try:
        if (connection == "serial" and ser.isOpen() or connection == "USB"):
            response = serial_command("QPIRI", usb0)
            if "NAKss" in response:
                if connection == "serial": time.sleep(0.5)
                return ""
            response.rstrip()
            nums = response.split(' ', 99)
            charger_source_priority = nums[17]

        elif (connection == "serial"):
            ser.close()
            print
            "cannot use serial port ..."
            return ""

    except Exception as e:
        print("error parsing inverter data...: " + str(e))
        return ""

    return charger_source_priority


def set_output_source_priority(output_source_priority):
    # set inverter output mode priority
    if not output_source_priority == "":
        try:
            if (connection == "serial" and ser.isOpen() or connection == "USB"):
                if output_source_priority == 0:
                    response = serial_command("POP00", usb0)
                    print
                    response
                elif output_source_priority == 1:
                    response = serial_command("POP01", usb0)
                    print
                    response
                elif output_source_priority == 2:
                    response = serial_command("POP02", usb0)
                    print
                    response

            elif (connection == "serial"):
                ser.close()
                print
                "cannot use serial port ..."
                return ""

        except Exception as e:
            print("error parsing inverter data...: " + str(e))
            return ''

    return 1


def set_charger_source_priority(charger_source_priority):
    # set inverter charge mode priority
    if not charger_source_priority == "":
        try:
            if (connection == "serial" and ser.isOpen() or connection == "USB"):

                if charger_source_priority == 0:
                    response = serial_command("PCP00", usb0)
                    print
                    response
                elif charger_source_priority == 1:
                    response = serial_command("PCP01", usb0)
                    print
                    response
                elif charger_source_priority == 2:
                    response = serial_command("PCP02", usb0)
                    print
                    response
                elif charger_source_priority == 3:
                    response = serial_command("PCP03", usb0)
                    print
                    response

            elif (connection == "serial"):
                ser.close()
                print
                "cannot use serial port ..."
                return ""

        except Exception as e:
            print("error parsing inverter data...: " + str(e))
            return ''

    return 1



def serial_command(command, device):
    global response

    try:
        response = ""
        xmodem_crc_func = crcmod.predefined.mkCrcFun('xmodem')
        # wierd mistake in Axpert firmware - correct CRC is: 0xE2 0x0A
        #if command == "POP02":
        #    command_crc = '\x50\x4f\x50\x30\x32\xe2\x0b\x0d\r'
        #else:
        command_crc = command + unhexlify(hex(xmodem_crc_func(command)).replace('0x', '', 1)) + '\r'

        # Set the signal handler and a 5-second alarm
        signal.signal(signal.SIGALRM, handler)
        signal.alarm(10)
        if len(command_crc) < 9:
            time.sleep(0.35)
            os.write(device, command_crc)

        else:
            cmd1 = command_crc[:8]
            cmd2 = command_crc[8:]
            time.sleep(0.35)
            os.write(device, cmd1)
            time.sleep(0.35)
            os.write(device, cmd2)
            time.sleep(0.25)
        while True:
            time.sleep(0.15)
            r = os.read(device, 256)
            response += r
            if '\r' in r: break

    except Exception as e:
        print("1error reading inverter...: " + str(e) + "Response :" + response)
        data = ""

    signal.alarm(0)

    #    sys.stdout.write(command + " : ")
    #    sys.stdout.flush()
    #    print(response)
    return response


def main():
    global mode0
    global mode1
    global parrallel_num
    global wake_up_start
    global load
    mode = 1
    wake_up_start = time.time()
    BattEng = 3201.984
    Wattsec = 0
#    response = serial_command("POP02", usb0)
#    print(response)
#    time.sleep(5)
#    response = serial_command("QMOD", usb0)
#    print(response)
#    time.sleep(2)
#    response = serial_command("POP02", usb0)
#    print(response)
#    time.sleep(2)
#    response = serial_command("QMOD", usb0)
#    print(response)
    endtime = time.time()
    response  = serial_command("QPIRI", usb0)
    print(response)
    while True:


        #with Input(keynames = 'curses') as input_generator:
         #   for e in input_generator:
          #      if e == "q":
           #         print("working")
            #    print(repr(e))

        for inverter in range(0, 1):
            data = get_data("QPIGS", inverter)

            deltat = time.time() - endtime
            Wattsec = float(nums_global[8]) * (float(nums_global[9]) - float(nums_global[15])) * float(deltat) / 3600
            BattEng = BattEng + Wattsec
#            print(BattEng)
            endtime = time.time()


            #print(mode)
            #print(nums_global[15])
            #print(str(float(nums_global[5])-float(nums_global[8])*float(nums_global[12])))

            #mode = 0 - Grid
            #mode = 1 - Battery
            #if (mode == 1) and (float(nums_global[0]) > 200):
            #    if float(nums_global[15]) > 30:
            #        response = serial_command("POP00", usb0)
            #        mode = 0
            #        print("Going to Grid")
            #print("1")
            #if mode == 0:
            #    if float(nums_global[5])-float(nums_global[8])*float(nums_global[12]) < 1250:
            #        response = serial_command("POP02", usb0)
            #        mode = 1
            #        print("Going to Battery")
            #print("2")
            #if mode == 0:
            #    if float(nums_global[5]) < 1250:
            #        response = serial_command("POP02", usb0)
            #        print(response)
            #        mode = 1
            #        print("Going to Battery")
            #print("3")
#            if not data == "": send = send_data(data)
#            if inverter == 0:
#                data = get_data("QPGS0", inverter)
#                if not data == "": send = send_data(data)
#            elif inverter == 1:
#                data = get_data("QPGS1", inverter)
#                if not data == "": send = send_data(data)
#            if (load > 0 and mode0 >= 0 and mode1 >= 0 and parrallel_num == 1):
#                dynamic_control()


#        charge_current = set_charge_current ()
#        hdo_tmp_LT = read_hdo(68)       #Read emoncms feed id=68 = LowTarif
#        hdo_tmp_HT = read_hdo(69)       #Read emoncms feed id=69 = HighTarif
#        output_source_priority = get_output_source_priority()
#        charger_source_priority = get_charger_source_priority()
#        if not output_source_priority == "8":
#            if not charger_source_priority == "8":
#                if hdo_tmp_LT == "1":
#                    print "LT"  # electricity is cheap, so charge batteries from grid and hold them fully charged! important for Lead Acid Batteries Only!
#                    if not output_source_priority == "0":       # Utility First (0: Utility first, 1: Solar First, 2: SBU)
#                        set_output_source_priority(0)
#                    if not charger_source_priority == "2":      # Utility First (0: Utility first, 1: Solar First, 2: Solar+Utility, 3: Solar Only)
#                        set_charger_source_priority(2)
#                if hdo_tmp_HT == "1":
#                    print "HT"  # electricity is expensive, so supply everything from batteries not from grid
#                    if not output_source_priority == "2":       # Utility First (0: Utility first, 1: Solar First, 2: SBU)
#                        set_output_source_priority(2)
#                    if not charger_source_priority == "3":      # Utility First (0: Utility first, 1: Solar First, 2: Solar+Utility, 3: Solar Only)
#                        set_charger_source_priority(3)

if __name__ == '__main__':
    main()

