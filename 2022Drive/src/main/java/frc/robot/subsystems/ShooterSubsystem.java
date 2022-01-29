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



public class ShooterSubsystem extends SubsystemBase {
 
  private DifferentialDrive m_myRobot;
  private CANSparkMax m_shooterMotor1;
  private CANSparkMax m_shooterMotor2;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem_2");
  

  public ShooterSubsystem() {
    this.init();

  }
  private void init(){
  
      m_shooterMotor2 = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
      m_shooterMotor1 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    m_shooterMotor2.restoreFactoryDefaults();
    m_shooterMotor1.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive( m_shooterMotor2, m_shooterMotor1);

  }
  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void tankDrive(double m_shooterMotor2, double m_shooterMotor1) {
    
    table.getEntry("leftY").setDouble(m_shooterMotor2);
    table.getEntry("rightY").setDouble(m_shooterMotor1);
    m_myRobot.tankDrive(m_shooterMotor2, m_shooterMotor1); 
  }

}
