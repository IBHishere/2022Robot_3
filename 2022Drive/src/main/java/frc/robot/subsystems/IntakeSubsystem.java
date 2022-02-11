// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax m_intakeMotor1;
  public IntakeSubsystem() {
  
  }
  public void intakePull() {

  }
  public void intakePush() {

  }
  public void intakeStop() {

  }
public void startIntake() {
  this.init();
}
 private void init(){
 m_intakeMotor1 = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  m_intakeMotor1.restoreFactoryDefaults();
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
