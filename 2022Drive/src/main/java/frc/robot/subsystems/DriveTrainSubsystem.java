// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import java.lang.Math;
import com.revrobotics.RelativeEncoder;


public class DriveTrainSubsystem extends SubsystemBase {

  private DifferentialDrive m_myRobot;
  private CANSparkMax m_leftMotor1;
  private RelativeEncoder m_leftEncoder ;
  private CANSparkMax m_rightMotor1;
  private RelativeEncoder m_rightEncoder; 
  private CANSparkMax m_leftMotor2;
  private RelativeEncoder m_left2Encoder;
  private CANSparkMax m_rightMotor2;
  private RelativeEncoder m_right2Encoder ;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem_2");
  
  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {
    this.init();
  }

  private void init(){
    m_leftMotor1 = new CANSparkMax(Constants.LEFT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(Constants.LEFT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_leftMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftMotor1,m_leftMotor2);
  
    m_rightMotor1 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_rightMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightMotor1,m_rightMotor2);
    


    m_leftEncoder= this.m_leftMotor1.getEncoder();
    m_rightEncoder= this.m_rightMotor1.getEncoder();
    m_left2Encoder = this.m_leftMotor2.getEncoder();
    m_right2Encoder = this.m_rightMotor2.getEncoder();

  m_rightMotor1.setInverted(false);
  m_rightMotor2.setInverted(true);
  m_leftMotor1.setInverted(false);
  m_leftMotor2.setInverted(true);
    m_myRobot = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);
  }

  @Override
  public void periodic() {}
    // This method will be called once per scheduler run

  private double computeActualDriveFromInput(double joystickValue) {
    //TODO: Move this to constants?
    double zeroLimit = 0.2;
    double driveCurvePower = 3.0;
    double driveScalingFactor = 1;
    
    double outputValue  = Math.abs(joystickValue) < zeroLimit ? 0 : Math.pow(joystickValue, driveCurvePower) * driveScalingFactor;

    return outputValue;

  }

  public void tankDrive(double leftJoystickValue, double rightJoystickValue) {
    //log joystick values to network tables
    table.getEntry("motorLeftValue").setDouble(leftJoystickValue);
    table.getEntry("motorRightValue").setDouble(rightJoystickValue);

    //convert joystick values to motor inputs
    double leftMotorValue = this.computeActualDriveFromInput(leftJoystickValue);
    double rightMotorValue = this.computeActualDriveFromInput(rightJoystickValue);

    
    m_myRobot.tankDrive( -leftMotorValue, rightMotorValue);
  }

  public void autoTankDrive(double motorLeftValue, double motorRightValue) {
    m_myRobot.tankDrive(Math.pow(-motorLeftValue,3)*.5, Math.pow(motorRightValue, 3)*.5);
  }
}

