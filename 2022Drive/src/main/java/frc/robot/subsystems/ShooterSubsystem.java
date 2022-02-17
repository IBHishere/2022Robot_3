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
import java.util.Timer;



public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax m_shooterMotor1;  
    private CANSparkMax m_queueMotor1; 
    private CANSparkMax m_queueMotor2; 
    private double m_velocity = 1;
  

  public ShooterSubsystem() {
    this.init();

  }
  private void init(){
    m_shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
    m_queueMotor1 = new CANSparkMax(Constants.QUEUE_MOTOR_CAN1_ID, MotorType.kBrushless);
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
    // this increases the speed of the shooter motor
  }

  public void increaseVelocity() {
    this.increaseVelocity(.05);
    // see above, by 0.5
  }

  public void setSpeed(double velocity) {
    this.m_velocity = velocity;
    // this is used to set the velocity to the required value
  }

  public double getVelocity() {
    return this.m_velocity;
    // this returns the current velocity
  }

  public void startShooter(){
    System.out.println("startShooter");
    m_shooterMotor1.set(1);    

// this shoots with a speed based on the velocity
  }
  public void stopShooter(){
    System.out.println("stopShooter");
    m_shooterMotor1.set(0);  
    // this stops the shooter
  }

  public void  startQueue(){
    System.out.println("startQueue");
    m_queueMotor1.set(.4);  
    // this starts the belt to bring balls upward
  }
  public void  stopQueue(){
    System.out.println("stopQueue");
    m_queueMotor1.set(0); 
    // this stops bringing balls upward
  }

  //new stuff
  public void  stopQueue2(){
    System.out.println("stopQueue2");
    m_queueMotor2.set(0); 
    //this stops the entrance to the shooter to allow shooter to be turned on
  }
  public void  startQueue2(){
    System.out.println("startQueue2");
    m_queueMotor2.set(.4); 
    // this activates the queue motor that feeds balls into the shooter
  }
}

