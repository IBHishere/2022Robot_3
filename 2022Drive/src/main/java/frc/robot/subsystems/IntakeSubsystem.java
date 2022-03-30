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
  private BeltSubsystem m_beltSubsystem;


  public IntakeSubsystem(BeltSubsystem beltSubsystem) {
    this.m_beltSubsystem = beltSubsystem;
    this.init();
  }

  public BeltSubsystem getBeltSubsystem() {
    return this.m_beltSubsystem;
  }

  private void init(){
    this.m_intakeMotor1 = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    this.m_intakeMotor1.restoreFactoryDefaults();
  }

  public void intakePull() {
    // this pulls in balls from the ground into the center of the robot
    System.out.println("intakePull");
    this.m_intakeMotor1.set(1.0);
    this.m_beltSubsystem.startBelt(.3);
  }

  public void intakePush() {
  // this pushes out balls incase one of the wrong color is pulled in
    System.out.println("intakePush");
    this.m_intakeMotor1.set(-1.0);
    this.m_beltSubsystem.startBelt(-.1);
  }
  
  public void intakeStop() {
  // this stops the intake
    System.out.println("intakeStop");
    this.m_intakeMotor1.set(0);
    this.m_beltSubsystem.stopBelt();
  }
}
