import odrive
from odrive.enums import *
import time
import math
# from fibre.protocol import ChannelBrokenException

# Find a connected ODrive (this will block until you connect one)
print("\r\nFinding an ODrive...")
odrv0 = odrive.find_any()

# odrv0.erase_configuration()
# odrv0 = odrive.find_any()

print("\r\nODrive Found")
print("Bus voltage is:" + str(odrv0.vbus_voltage) + "V")

print("Calibration current:" + str(odrv0.axis1.motor.config.calibration_current) + "A")


odrv0.axis0.motor.config.current_lim = 50

## motor will be limited to this speed in [turn/s]
odrv0.axis0.controller.config.vel_limit = 20

# largest value running through the motor continuously when the motor is stationary
# odrv0.axis0.motor.config.calibration_current = 60

odrv0.config.dc_max_negative_current = -30

odrv0.axis0.motor.config.pole_pairs = 2

# set to 8.27 / (motor KV) = 8.27/1900
odrv0.axis0.motor.config.torque_constant = 8.27/4700

odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

# W/ ENCODER
# CPR = PPR * 4 = 2049 * 4 (Default resolution of CUI-AMT102 encoder)
odrv0.axis0.encoder.config.cpr = 8192

odrv0.axis0.encoder.config.bandwidth = 1000

odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL



odrv0.axis0.encoder.config.bandwidth = 1000
odrv0.axis0.controller.config.pos_gain = .5
odrv0.axis0.controller.config.vel_gain = 0.0003 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_integrator_gain = 0.001 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr

print("\r\nset parameters\r\n\r\nrequesting calibration")

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

print("Saving manual configuration and rebooting...")
odrv0.save_configuration()

print("Manual configuration saved.")
# # odrv0.reboot()
# try:
#     odrv0.reboot()
# except ChannelBrokenException:
#     pass
            
odrv0 = odrive.find_any()


print("recalibrating after reboot")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# odrv0.save_configuration()

# odrv0 = odrive.find_any()

print("phase resistance: ", odrv0.axis1.motor.config.phase_resistance)

time.sleep(5)
print("\r\naxis errors are:")
print(hex(odrv0.axis1.error)) 
print("motor errors are:")
print(hex(odrv0.axis1.motor.error))
print("encoder errors are:")
print(hex(odrv0.axis1.encoder.error))
print("sensorless estimator errors are:")
print(hex(odrv0.axis1.sensorless_estimator.error))

print("\r\nIs calibrated? ", odrv0.axis1.motor.is_calibrated)
print("")

# start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])

print("Starting closed loop control and going to position 3")

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.input_pos = 3

print("Starting velocity control mode and entering velocity 1")

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.input_vel = 1