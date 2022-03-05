// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class BeltSubsystem extends SubsystemBase {

  private CANSparkMax m_queueBeltMotor; 
    
  /** Creates a new BeltSubsystem. */
  public BeltSubsystem() {
    m_queueBeltMotor = new CANSparkMax(Constants.QUEUE_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_queueBeltMotor.restoreFactoryDefaults();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startBelt(double d) {
    this.m_queueBeltMotor.set(d);
  }

  public void stopBelt() {
    this.m_queueBeltMotor.set(0);
  }
}
