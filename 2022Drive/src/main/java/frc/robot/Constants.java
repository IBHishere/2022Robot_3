// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// controlers    
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int HELPER_XBOX_CONTROLLER = 1;
// left drive
    public static final int LEFT_MOTOR_CAN1_ID = 1; // change to 1
    public static final int LEFT_MOTOR_CAN2_ID = 2; // change to 2
//right drive
    public static final int RIGHT_MOTOR_CAN1_ID = 3; // change to 3
    public static final int RIGHT_MOTOR_CAN2_ID = 4; // change to 4

    
// Setpoints
public static final double INCHES_FROM_GOAL = 24; // change to 4

// Placeholders
public static final double Zero_for_now = 0; // change to 0

// Speed Regulator
public static final double SPEED_REGULATOR = 0.65;
public static final double DEAD_ZONE_VALUE = 0.03;
public static final double TURN_REGULATOR = 0.45;

// Time of Flight ID
public static final int TIMEOFFLIGHT_ID = 0;

}
 