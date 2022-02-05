// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private DifferentialDrive m_myRobot;
  private CANSparkMax m_intakeMotor1;
 // private CANSparkMax m_intakeMotor2;
//  private CANSparkMax m_intakeMotor3;
 // private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //private NetworkTable table = inst.getTable("DriveTrainSubsystem_2");
  public IntakeSubsystem() {

    this.init();
  }
  private void init(){
  
    m_intakeMotor1 = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    //m_intakeMotor2 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
   // m_intakeMotor3 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

  m_intakeMotor1.restoreFactoryDefaults();
 // m_intakeMotor2.restoreFactoryDefaults();
 // m_intakeMotor3.restoreFactoryDefaults();

  }

  public void startIntake(){

    SparkMaxPIDController intakeController = m_intakeMotor1.getPIDController();
    intakeController.setReference(.2, CANSparkMax.ControlType.kDutyCycle);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
