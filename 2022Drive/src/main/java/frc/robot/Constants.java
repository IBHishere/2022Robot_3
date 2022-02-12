// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int LEFT_MOTOR_CAN1_ID = 1;
    public static final int LEFT_MOTOR_CAN2_ID = 2;
//right drive
    public static final int RIGHT_MOTOR_CAN1_ID = 3;
    public static final int RIGHT_MOTOR_CAN2_ID = 4;
// intake    
    public static final int INTAKE_MOTOR_CAN_ID = 5;
    public static final double INTAKEPULLSPEED = 1;
    public static final double INTAKEPUSHSPEED = -1;
// shooter & queue
    public static final int SHOOTER_MOTOR_CAN_ID = 6;
    public static final int QUEUE_MOTOR_CAN_ID = 7;

    

   
}
