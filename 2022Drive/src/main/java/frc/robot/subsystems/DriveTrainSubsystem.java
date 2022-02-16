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


public class DriveTrainSubsystem extends SubsystemBase {

  private DifferentialDrive m_myRobot;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor2;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem_2");
  
  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {
    this.init();
  }

  private void init(){

    
    m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(Constants.LEFT_MOTOR_CAN2_ID, MotorType.kBrushless);
    
    m_leftMotor.setInverted(false);
    m_leftMotor2.setInverted(true);
  
    table.getEntry("Left is follower").forceSetBoolean(m_leftMotor.isFollower());
    table.getEntry("Left2 is follower").forceSetBoolean(m_leftMotor2.isFollower());
    
    MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor,m_leftMotor2);
    
    
    m_rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_rightMotor.setInverted(false);
    m_rightMotor2.setInverted(true);
    
    table.getEntry("Right is follower").forceSetBoolean(m_rightMotor.isFollower());MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor,m_rightMotor2 );

    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_left, m_right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void tankDrive(double motorLeftValue, double motorRightValue) {
    table.getEntry("leftY").setDouble(motorLeftValue);
    table.getEntry("rightY").setDouble(motorRightValue);
    if ((motorLeftValue > .015 || motorLeftValue < -.015) && (motorRightValue < -.015 || motorRightValue > .015)){
    m_myRobot.tankDrive(Math.pow(-motorLeftValue,3)*.5, Math.pow(-motorRightValue, 3)*.5);
    }
    else{
      m_myRobot.tankDrive(0,0);
    }
    
  }
  public void autoTankDrive(double motorLeftValue, double motorRightValue) {
    
    m_myRobot.tankDrive(Math.pow(-motorLeftValue,3)*.5, Math.pow(-motorRightValue, 3)*.5);
   
    
    
  }
}
