// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
//import java.util.Timer;



public class ShooterSubsystem extends SubsystemBase {
    private static final double QUEUE_MOTOR_POWER = .5;
    private CANSparkMax m_shooterMotor;  
    private CANSparkMax m_queueFeederWheerMotor; 
    private double m_velocity = 1.0;

    private BeltSubsystem m_beltSubsystem;

    private boolean m_isShooterOn = false;
    private boolean m_isQueue1On = false;
    private boolean m_isQueue2On = false;

  public ShooterSubsystem(BeltSubsystem beltSubsystem) {
    this.m_beltSubsystem = beltSubsystem;
    this.init();
  }
  
  private void init(){
    m_shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
    m_shooterMotor.restoreFactoryDefaults();
    
    m_queueFeederWheerMotor = new CANSparkMax(Constants.QUEUE_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_queueFeederWheerMotor.restoreFactoryDefaults();
    m_queueFeederWheerMotor.setIdleMode(IdleMode.kBrake);
  
  }

  
  // public void increaseVelocity(double inc) {
  //   this.m_velocity = Math.min(1, this.m_velocity+inc);
  //   // this increases the speed of the shooter motor
  // }

  // public void increaseVelocity() {
  //   this.increaseVelocity(.05);
  //   // see above, by 0.5
  // }

  // public void setSpeed(double velocity) {
  //   this.m_velocity = velocity;
  //   // this is used to set the velocity to the required value
  // }

  // public double getVelocity() {
  //   return this.m_velocity;
  //   // this returns the current velocity
  // }

  public void startShooter(){
    System.out.println("startShooter");
    m_shooterMotor.set(1.0);  
    m_isShooterOn = true;  
      

// this shoots with a speed based on the velocity
  }
  public void stopShooter(){
    System.out.println("stopShooter");
    m_shooterMotor.set(0.0);  
    m_isShooterOn = false;
  }

  // public void  startQueueBelt(){
  //   System.out.println("startQueue");
  //   this.m_beltSubsystem.startBelt(1.0);  
  //   m_isQueue1On = true;
  // }

  
  // public void  stopQueueBelt(){
  //   System.out.println("stopQueue");
  //   this.m_beltSubsystem.stopBelt(); 
  //   m_isQueue1On = false;
  //   // this stops bringing balls upward
  // }

  //new stuff
  public void  stopQueueFeederWheel(){
    System.out.println("stopQueue2");
    m_queueFeederWheerMotor.set(0); 
    m_isQueue2On = false;
    //this stops the entrance to the shooter to allow shooter to be turned on
  }
  public void  startQueueFeederWheel(){
    System.out.println("startQueue2");
    m_queueFeederWheerMotor.set(ShooterSubsystem.QUEUE_MOTOR_POWER); 
    m_isQueue2On = true;


    // this activates the queue motor that feeds balls into the shooter
  }

  public void toggleShooter(){ 
    if(m_isShooterOn == false){
      startShooter();
    }
    else{
      stopShooter(); 
    }
  }

  // public void toggleQueue(){ 
  //   if(m_isQueue1On == false){
  //     startQueueBelt();
  //   }
  //   else{
  //     stopQueueBelt(); 
  //   }
  // }

  public void toggleQueue2(){ 
    if(m_isQueue2On == false){
      startQueueFeederWheel();
    }
    else{
      stopQueueFeederWheel(); 
    }
  }

  public Subsystem getBeltSubsystem() {
    return this.m_beltSubsystem;
  }
}