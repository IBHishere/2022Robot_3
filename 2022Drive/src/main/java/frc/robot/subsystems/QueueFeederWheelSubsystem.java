// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QueueFeederWheelSubsystem extends SubsystemBase {


  private CANSparkMax m_queueFeederWheerMotor; 
  private static final double QUEUE_MOTOR_POWER = 1;
    
  
  /** Creates a new QueueFeederWheelSubsystem. */
  public QueueFeederWheelSubsystem() {
    m_queueFeederWheerMotor = new CANSparkMax(Constants.QUEUE_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_queueFeederWheerMotor.restoreFactoryDefaults();
    m_queueFeederWheerMotor.setIdleMode(IdleMode.kCoast);
  
  }

  public void  stopQueueFeederWheel(){
    System.out.println("stopQueue2");
    m_queueFeederWheerMotor.set(0); 
    //this stops the entrance to the shooter to allow shooter to be turned on
  }
  public void  startQueueFeederWheel(){
    System.out.println("startQueue2");
    m_queueFeederWheerMotor.set(QueueFeederWheelSubsystem.QUEUE_MOTOR_POWER); 
    

    // this activates the queue motor that feeds balls into the shooter
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
