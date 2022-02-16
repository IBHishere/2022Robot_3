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
  this.init();
  }
private void init(){

  m_intakeMotor1 = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  m_intakeMotor1.restoreFactoryDefaults();
    }

 public void intakePull() {
// this pulls in balls from the ground into the center of the robot
  m_intakeMotor1.set(1);
  System.out.println("intakePull");

  }
public void intakePush() {
// this pushes out balls incase one of the wrong color is pulled in
m_intakeMotor1.set(-1);
System.out.println("intakePush");

  }
  public void intakeStop() {
// this stops the intake
    m_intakeMotor1.set(0);
    System.out.println("intakeStop");
    
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
