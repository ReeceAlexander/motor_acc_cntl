#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Int32

def send_acceleration_command(can_id, accel):
    """
    Sends acceleration and deceleration commands to the motor.

    :param can_id: The motor CAN ID (integer).
    :param accel: The desired acceleration in dps/s (integer).
    """
    # Ensure accel is within the valid range
    accel = max(100, min(60000, accel))
    
    # Convert acceleration to 32-bit hexadecimal value
    accel_hex = f"{accel:08X}"
    
    # Split the 32-bit hexadecimal value into 4 bytes
    byte4 = accel_hex[0:2]  # High byte
    byte3 = accel_hex[2:4]
    byte2 = accel_hex[4:6]
    byte1 = accel_hex[6:8]  # Low byte
    
    # Construct CAN data strings for acceleration and deceleration
    can_acc_data = f"43 02 00 00 {byte1} {byte2} {byte3} {byte4}"
    can_dec_data = f"43 03 00 00 {byte1} {byte2} {byte3} {byte4}"
    
    # Create the 'cansend' commands
    can_id_hex = f"{can_id:03X}"  # Convert CAN ID to 3-digit hex format
    cansend_acc_command = f"cansend can0 {can_id_hex}#{can_acc_data.replace(' ', '')}"
    cansend_dec_command = f"cansend can0 {can_id_hex}#{can_dec_data.replace(' ', '')}"
    
    # Send commands to the CAN BUS
    os.system(cansend_acc_command)
    rospy.loginfo(f"Sent acceleration command to motor {can_id_hex}: {accel} dps/s")
    time.sleep(0.1)
    os.system(cansend_dec_command)
    rospy.loginfo(f"Sent deceleration command to motor {can_id_hex}: {accel} dps/s")

def motor_acc_callback(msg):
    accel = msg.data
    motor_ids = [0x141, 0x142, 0x143, 0x144]
    
    for motor_id in motor_ids:
        send_acceleration_command(motor_id, accel)
        time.sleep(0.1)  # Small delay to avoid errors

def main():
    rospy.init_node('motor_can_node')

    rospy.Subscriber('/motor_acc', Int32, motor_acc_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
        print("update_motors.py")
    except rospy.ROSInterruptException:
        pass