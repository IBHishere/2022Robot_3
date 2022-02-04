// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax m_shooterMotor1;
  public IntakeSubsystem() {
  
  }
  
  // private void init(){
  //   m_IntakeMotor1 = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  //  m_IntakeMotor1.restoreFactoryDefaults();
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
