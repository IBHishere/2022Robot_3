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


public class DriveTrainSubsystem extends SubsystemBase {

  private DifferentialDrive m_myRobot;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem_2");
  
  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {
    this.init();
  }

  private void init(){
    m_leftMotor1 = new CANSparkMax(Constants.LEFT_MOTOR1_CAN_ID, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(Constants.RIGHT_MOTOR1_CAN_ID, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(Constants.LEFT_MOTOR2_CAN_ID, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(Constants.RIGHT_MOTOR2_CAN_ID, MotorType.kBrushless);
MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_myRobot = new DifferentialDrive(m_leftMotors, m_rightMotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void tankDrive(double motorLeftValue, double motorRightValue) {
    
    table.getEntry("leftY").setDouble(motorLeftValue);
    table.getEntry("rightY").setDouble(motorRightValue);
    if (Constants.QUADRATICDRIVE == false){
    m_myRobot.tankDrive(-motorLeftValue*Constants.SPEEDMODIFIER, -motorRightValue*Constants.SPEEDMODIFIER);
    } else if (Constants.QUADRATICDRIVE == true){
      m_myRobot.tankDrive(-motorLeftValue*motorLeftValue*Constants.SPEEDMODIFIER, -motorRightValue*motorRightValue*Constants.SPEEDMODIFIER);
    }else {
      m_myRobot.tankDrive(-motorLeftValue, -motorRightValue);
    }
  }
}
