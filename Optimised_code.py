import time
import math
import board
import busio
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3

# Create I2C bus
i2c = board.I2C()

# Create TCA9548A I2C Multiplexer object
tca = adafruit_tca9548a.TCA9548A(i2c)

LeftShoulder = LSM6DS3(tca[0]) # Left shoulder
MiddleBack = LSM6DS3(tca[1]) # Middle back
RightShoulder = LSM6DS3(tca[7]) # Right shoulder
LeftLeg = LSM6DS3(tca[3]) # Left leg
RightLeg = LSM6DS3(tca[4]) # Right leg
LeftArm = LSM6DS3(tca[2]) # Left arm
RightArm = LSM6DS3(tca[5]) # Right arm

LA_reps = 0
RA_reps = 0
L_reps = 0

def calibration():
    file = open("Calibration_data.txt","w") # file contents: MB pitch, MB roll, LS to MB pitch, LS to MB roll, RS to MB pitch, RS to MB roll, low rep pitch, low rep roll, high rep pitch, high rep roll, low squat pitch, low squat roll, high squat pitch, high squat roll
    #posture difference calibration
    print("Please stand up straight ")
    start = input("Press enter when you're ready")
    accelMiddleBack = MiddleBack.acceleration
    Recorded_MB_pitch, Recorded_MB_roll = process_body_part("MB", accelMiddleBack)
    accelLeftShoulder = LeftShoulder.acceleration
    Recorded_LS_pitch, Recorded_LS_roll = process_body_part("LS", accelLeftShoulder)
    accelRightShoulder = RightShoulder.acceleration
    Recorded_RS_pitch, Recorded_RS_roll = process_body_part("RS", accelRightShoulder)
    LS_to_MB_diff_pitch = calculate_difference(Recorded_MB_pitch, Recorded_LS_pitch)
    LS_to_MB_diff_roll = calculate_difference(Recorded_MB_roll, Recorded_LS_roll)
    RS_to_MB_diff_pitch = calculate_difference(Recorded_MB_pitch, Recorded_RS_pitch)
    RS_to_MB_diff_roll = calculate_difference(Recorded_MB_roll, Recorded_RS_roll)
    file.write('%r\n' % (Recorded_MB_pitch))
    file.write('%r\n' % (Recorded_MB_roll))
    file.write('%r\n' % (LS_to_MB_diff_pitch))
    file.write('%r\n' % (LS_to_MB_diff_roll))
    file.write('%r\n' % (RS_to_MB_diff_pitch))
    file.write('%r\n' % (RS_to_MB_diff_roll))
    print("Posture calibration complete!")

    #arm rep angle calibration
    arm_chosen = input("Please enter which arm you would like to use to calibrate your rep counter (L or R)")
    print("Please put your arm in your lowered rep position")
    start = input("Press enter when you are ready")
    if arm_chosen == "R":
        accelRightArm = RightArm.acceleration
        low_rep_RA_pitch, low_rep_RA_roll = process_body_part("RA", accelRightArm)
        print("Done!")
        print("Please put your arm in your peak rep position!")
        start = input("Press enter when you are ready")
        accelRightArm = RightArm.acceleration
        high_rep_RA_pitch, high_rep_RA_roll = process_body_part("RA", accelRightArm)
        file.write('%r\n' % (low_rep_RA_pitch))
        file.write('%r\n' % (low_rep_RA_roll))
        file.write('%r\n' % (high_rep_RA_pitch))
        file.write('%r\n' % (high_rep_RA_roll))

    elif arm_chosen == "L":
        accelLeftArm = LeftArm.acceleration
        low_rep_LA_pitch, low_rep_LA_roll = process_body_part("LA", accelLeftArm)
        print("Done!")
        print("Please put your arm in your peak rep position!")
        start = input("Press enter when you are ready")
        accelLeftArm = LeftArm.acceleration
        high_rep_LA_pitch, high_rep_LA_roll = process_body_part("LA", accelLeftArm)
        file.write('%r\n' % (low_rep_LA_pitch))
        file.write('%r\n' % (low_rep_LA_roll))
        file.write('%r\n' % (high_rep_LA_pitch))
        file.write('%r\n' % (high_rep_LA_roll))

    # leg 'rep' angle calibration
    print("Please put your legs in your lowered squat position")
    start = input("Press enter when you are ready")
    accelRightLeg = RightLeg.acceleration
    accelLeftLeg = LeftLeg.acceleration
    low_rep_RL_pitch, low_rep_RL_roll = process_body_part("RL", accelRightLeg)
    low_rep_LL_pitch, low_rep_LL_roll = process_body_part("LL", accelLeftLeg)
    low_rep_pitch_mean = (low_rep_RL_pitch + low_rep_LL_pitch) / 2
    low_rep_roll_mean = (low_rep_RL_roll + low_rep_LL_roll) / 2
    print("Done!")
    print("Please put your arm in your peak rep position!")
    start = input("Press enter when you are ready")
    accelRightLeg = RightLeg.acceleration
    accelLeftLeg = LeftLeg.acceleration
    high_rep_RL_pitch, high_rep_RL_roll = process_body_part("RL", accelRightLeg)
    high_rep_LL_pitch, high_rep_LL_roll = process_body_part("LL", accelLeftLeg)
    high_rep_pitch_mean = (high_rep_RL_pitch + high_rep_LL_pitch) / 2
    high_rep_roll_mean = (high_rep_RL_roll + high_rep_LL_roll) / 2
    file.write('%r\n' % (low_rep_pitch_mean))
    file.write('%r\n' % (low_rep_roll_mean))
    file.write('%r\n' % (high_rep_pitch_mean))
    file.write('%r\n' % (high_rep_roll_mean))
    file.close()

def calculate_difference(reference, other_part): # reference is the value for the part and same for other
    if reference > 0:
        difference = (reference + (-other_part)) * -1
    elif reference < 0:
        difference = (reference - other_part) * -1
    else:
        difference = (reference + (-other_part)) * -1
    return difference


def process_body_part(part_name, accel):
    # isolating x, y, and z accelerations (in g's)
    accel_X = accel[0] / -9.8
    accel_Y = accel[1] / -9.8
    accel_Z = accel[2] / -9.8

    # calculating rotation
    roll = math.atan(accel_Y / (math.sqrt((accel_X**2) + (accel_Z**2)))) * (180 / math.pi)
    pitch = -math.atan(accel_X / (math.sqrt((accel_Y**2) + (accel_Z**2)))) * (180 / math.pi)

    if accel_Z > 0 and accel_Y > 0:
        roll = 90 + (90 - roll)
    elif accel_Z > 0 and accel_Y < 0:
        roll = -90 - (90 + roll)

    if accel_Z > 0 and accel_X > 0:
        pitch = -90 - (90 + pitch)
    elif accel_Z > 0 and accel_X < 0:
        pitch = 90 + (90 - pitch)

    return pitch, roll


# START OF ACTUAL CODE #
calibrateQ = input("Do you want to calibrate the system? (Y or N)")
if calibrateQ == "Y":
    calibration()

#putting all of the data into an accessible list
base_data = []
file = open("Calibration_data.txt","r")
for line in file:
    base_data.append(line.strip("\n"))
    print(base_data)

hasbeenatbottom_LA = False
hasbeenattop_LA = False
hasbeenatbottom_RA = False
hasbeenattop_RA = False
hasbeenatbottom_L = False
hasbeenattop_L = False

for count in range(20000):
    # Access each sensor via its instance
    accelLeftArm = LeftArm.acceleration
    accelRightArm = RightArm.acceleration
    accelLeftLeg = LeftLeg.acceleration
    accelRightLeg = RightLeg.acceleration
    accelLeftShoulder = LeftShoulder.acceleration
    accelRightShoulder = RightShoulder.acceleration
    accelMiddleBack = MiddleBack.acceleration

    #gyroLeftArm = LeftArm.gyro
    #gyroRightArm = RightArm.gyro
    #gyroLeftLeg = LeftLeg.gyro
    #gyroRightLeg = RightLeg.gyro
    #gyroLeftShoulder = LeftShoulder.gyro
    #gyroRightShoulder = RightShoulder.gyro
    #gyroMiddleBack = MiddleBack.gyro

    # Process and print pitch and roll values for each body part
    # Left Shoulder
    LS_pitch, LS_roll = process_body_part("LS", accelLeftShoulder)
  #  print("Left Shoulder pitch =", LS_pitch)
  #  print("Left Shoulder roll =", LS_roll)

    # Right Shoulder
    RS_pitch, RS_roll = process_body_part("RS", accelRightShoulder)
 #   print("Right Shoulder pitch =", RS_pitch)
#    print("Right Shoulder roll =", RS_roll)

    # Middle Back
    MB_pitch, MB_roll = process_body_part("MB", accelMiddleBack)
#    print("Middle Back pitch =", MB_pitch)
 #   print("Middle Back roll =", MB_roll)

    # Left Arm
    LA_pitch, LA_roll = process_body_part("LA", accelLeftArm)
    #print("Left Arm pitch =", LA_pitch)
    #print("Left Arm roll =", LA_roll)

    # Right Arm
    RA_pitch, RA_roll = process_body_part("RA", accelRightArm)
    #print("Right Arm pitch =", RA_pitch)
    #print("Right Arm roll =", RA_roll)

    # Left Leg
    LL_pitch, LL_roll = process_body_part("LL", accelLeftLeg)
    #print("Left Leg pitch =", LL_pitch)
    #print("Left Leg roll =", LL_roll)

    # Right Leg
    RL_pitch, RL_roll = process_body_part("RL", accelRightLeg)
    #print("Right Leg pitch =", RL_pitch)
    #print("Right Leg roll =", RL_roll)

    # actually calculating useful information
    # shoulder to midddle differences
    LS_to_MB_pitch = calculate_difference(MB_pitch, LS_pitch)
    LS_to_MB_roll = calculate_difference(MB_roll, LS_roll)
    RS_to_MB_pitch = calculate_difference(MB_pitch, RS_pitch)
    RS_to_MB_roll = calculate_difference(MB_roll, RS_roll)

    # checking if the posture is ok
    calibrated_MB_pitch = base_data[0]
    calibrated_MB_roll = base_data[1]
    calibrated_RS_middle_pitch = base_data[4]
    calibrated_RS_middle_roll = base_data[5]
    calibrated_LS_middle_pitch = base_data[2]
    calibrated_LS_middle_roll = base_data[3]
    base_to_current_MB_pitch = calculate_difference(float(calibrated_MB_pitch), MB_pitch)
    base_to_current_MB_roll = calculate_difference(float(calibrated_MB_roll), MB_roll)
    base_to_current_RS_middle_pitch = calculate_difference(float(calibrated_RS_middle_pitch), RS_to_MB_pitch)
    base_to_current_RS_middle_roll = calculate_difference(float(calibrated_RS_middle_roll), RS_to_MB_roll)
    base_to_current_LS_middle_pitch = calculate_difference(float(calibrated_LS_middle_pitch), LS_to_MB_pitch)
    base_to_current_LS_middle_roll = calculate_difference(float(calibrated_LS_middle_roll), LS_to_MB_roll)

    if base_to_current_MB_pitch > 10 or base_to_current_MB_pitch < -10:
        print("middle bad (pitch)")
    if base_to_current_MB_roll > 10 or base_to_current_MB_roll < -10:
        print("middle bad (roll)")
    if base_to_current_RS_middle_pitch > 10 or base_to_current_RS_middle_pitch < -10:
        print("right shoulder bad (pitch)")
    if base_to_current_RS_middle_roll > 10 or base_to_current_RS_middle_roll < -10:
        print("right shoulder bad (roll)")
    if base_to_current_LS_middle_pitch > 10 or base_to_current_LS_middle_pitch < -10:
        print("left shoulder bad (pitch)")
    if base_to_current_LS_middle_roll > 10 or base_to_current_LS_middle_roll < -10:
        print("left shoulder bad (roll)")

    # checking if a rep has been done (first if bottom was reached and then if top was reached)
    low_pitch = float(base_data[6])
    low_roll = float(base_data[7])
    high_pitch = float(base_data[8])
    high_roll = float(base_data[9])
    # LA
    if hasbeenatbottom_LA == False:
        low_to_current_LA_pitch = calculate_difference(low_pitch, LA_pitch)
        low_to_current_LA_roll = calculate_difference(low_roll, LA_roll)
        if low_to_current_LA_pitch > 5 or low_to_current_LA_pitch < -5 and low_to_current_LA_roll > 5 or low_to_current_LA_roll < -5:
            hasbeenatbottom_LA = True
            hasbeenattop_LA = False
    else:
        high_to_current_LA_pitch = calculate_difference(high_pitch, LA_pitch)
        high_to_current_LA_roll = calculate_difference(high_roll, LA_roll)
        if high_to_current_LA_pitch > 5 or high_to_current_LA_pitch < -5 and high_to_current_LA_roll > 5 or high_to_current_LA_roll < -5:
            hasbeenatbottom_LA = False
            hasbeenattop_LA = True
            LA_reps = LA_reps + 1

    #RA
    if hasbeenatbottom_RA == False:
        low_to_current_RA_pitch = calculate_difference(low_pitch, RA_pitch)
        low_to_current_RA_roll = calculate_difference(low_roll, RA_roll)
        if low_to_current_RA_pitch > 5 or low_to_current_RA_pitch < -5 and low_to_current_RA_roll > 5 or low_to_current_RA_roll < -5:
            hasbeenatbottom_RA = True
            hasbeenattop_RA = False

    else:
        high_to_current_RA_pitch = calculate_difference(high_pitch, RA_pitch)
        high_to_current_RA_roll = calculate_difference(high_roll, RA_roll)
        if high_to_current_RA_pitch > 5 or high_to_current_RA_pitch < -5 and high_to_current_RA_roll > 5 or high_to_current_RA_roll < -5:
            hasbeenatbottom_RA = False
            hasbeenattop_RA = True
            RA_reps = RA_reps + 1

    print(RA_reps, LA_reps)

    # leg squat count
    low_pitch_L = float(base_data[10])
    low_roll_L = float(base_data[11])
    high_pitch_L = float(base_data[12])
    high_roll_L = float(base_data[13])
    current_L_mean_pitch = (RL_pitch + LL_pitch) / 2
    current_L_mean_roll = (RL_roll + LL_roll) / 2

    if hasbeenatbottom_L == False:
        low_to_current_L_pitch = calculate_difference(low_pitch_L, current_L_mean_pitch)
        low_to_current_L_roll = calculate_difference(low_roll_L, current_L_mean_roll)
        if low_to_current_L_pitch > 5 or low_to_current_L_pitch < -5 and low_to_current_L_roll > 5 or low_to_current_L_roll < -5:
            hasbeenatbottom_L = True
            hasbeenattop_L = False
    else:
        high_to_current_L_pitch = calculate_difference(high_pitch_L, current_L_mean_pitch)
        high_to_current_L_roll = calculate_difference(high_roll_L, current_L_mean_roll)
        if high_to_current_L_pitch > 5 or high_to_current_L_pitch < -5 and high_to_current_L_roll > 5 or high_to_current_L_roll < -5:
            hasbeenatbottom_L = False
            hasbeenattop_L = True
            L_reps = L_reps + 1

    print(L_reps)

    #print(left_diff_middle_pitch, "left to middle pitch")
    #print(left_diff_middle_roll, "left to middle roll")
    #print(right_diff_middle_pitch, "right to middle pitch")
    #print(right_diff_middle_roll, "right to middle roll")

    print("-" * 20)  # separates the outputs with dashes
    time.sleep(0.1)
