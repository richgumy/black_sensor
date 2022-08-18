# from Phidget22.Phidget import *
# from Phidget22.Devices.VoltageRatioInput import *
# import time

# voltage_V = 0

# def onVoltageRatioChange(self, voltageRatio):
#     print("VoltageRatio: " + str(voltageRatio))
#     voltage_V = voltageRatio

# ch = VoltageRatioInput()

# # Register for event before calling open
# ch.setOnVoltageRatioChangeHandler(onVoltageRatioChange)

# ch.open()

# while True:
#     # Do work, wait for events, etc.
#     time.sleep(1)
#     print("Get voltage:"+str(voltageRatio))
#     time.sleep(1)

####### NEW CODE ##########
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import time

def onVoltageRatioChange(self, voltageRatio):
    # print("VoltageRatio: " + str(voltageRatio))
    mass_g = voltageRatio*4.94462e+6 + 70.13
    self.voltage_V = voltageRatio
    self.mass_g = mass_g

def main():
    voltageRatioInput0 = VoltageRatioInput()

    voltageRatioInput0.setOnVoltageRatioChangeHandler(onVoltageRatioChange)

    voltageRatioInput0.openWaitForAttachment(5000)

    voltageRatioInput0.setDataRate(8)

    while(1):
        time.sleep(0.5)

        # print(voltageRatioInput0.voltage_V)
        print(voltageRatioInput0.mass_g)

    # try:
    #     input("Press Enter to Stop\n")
    # except (Exception, KeyboardInterrupt):
    #     pass

    voltageRatioInput0.close()

main()