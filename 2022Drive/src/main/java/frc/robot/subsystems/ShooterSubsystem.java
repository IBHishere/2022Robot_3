// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.security.Principal;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax m_shooterMotor1;  
    private CANSparkMax m_queueMotor1; 
    private CANSparkMax m_queueMotor2; 
    private double m_velocity = .5;
  

  public ShooterSubsystem() {
    this.init();

  }
  private void init(){
    m_shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
    m_queueMotor1 = new CANSparkMax(Constants.QUEUE_MOTOR_CAN_1ID, MotorType.kBrushless);
    m_queueMotor2 = new CANSparkMax(Constants.QUEUE_MOTOR_CAN2_ID, MotorType.kBrushless);

    m_shooterMotor1.restoreFactoryDefaults();
    m_queueMotor1.restoreFactoryDefaults();
    m_queueMotor2.restoreFactoryDefaults();

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void increaseVelocity(double inc) {
    this.m_velocity = Math.min(1, this.m_velocity+inc);
  }

  public void increaseVelocity() {
    this.increaseVelocity(.05);
  }

  public void setSpeed(double velocity) {
    this.m_velocity = velocity;
  }

  public double getVelocity() {
    return this.m_velocity;
  }

  public void startShooter(){
    System.out.println("startShooter");
    m_shooterMotor1.set(this.m_velocity);    


  }
  public void stopShooter(){
    System.out.println("stopShooter");
    m_shooterMotor1.set(0);  
  }

  public void  startQueue(){
    System.out.println("startQueue");
    m_queueMotor1.set(1);  
  }
  public void  stopQueue(){
    System.out.println("stopQueue");
    m_queueMotor1.set(0); 
  }

  //new stuff
  public void  stopQueue2(){
    System.out.println("stopQueue2");
    m_queueMotor2.set(0); 
  }
  public void  startQueue2(){
    System.out.println("startQueue2");
    m_queueMotor2.set(1); 
  }
}

