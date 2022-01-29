// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class DriveTrainSubsystem extends SubsystemBase {

  private DifferentialDrive m_myRobot;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem_2");
  
  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {
    this.init();
  }

  private void init(){
    m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void tankDrive(double motorLeftValue, double motorRightValue) {
    
    table.getEntry("leftY").setDouble(motorLeftValue);
    table.getEntry("rightY").setDouble(motorRightValue);
    m_myRobot.tankDrive(-motorLeftValue, -motorRightValue);
  }
}
