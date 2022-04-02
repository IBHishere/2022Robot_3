// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import java.lang.Math;
import com.revrobotics.RelativeEncoder;
//import java.util.Timer;



public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax m_shooterMotor;  
    private RelativeEncoder m_shooterEncoder;
    private double m_velocity = 1.0;

    private BeltSubsystem m_beltSubsystem;

    private boolean m_isShooterOn = false;

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterSubsystem");
    
    public enum GoalMode {
      HighGoal,
      LowGoal
    }

    public static final double HighGoalShooterSpeed = 3800; // max speed based n testing
    public static final double LowGoalShooterSpeed = 1500;
    
    public static GoalMode CurrentGoalMode;

    static {
      ShooterSubsystem.setGoalMode(ShooterSubsystem.GoalMode.HighGoal);
    }

    public static void setGoalMode(GoalMode mode) { 
      ShooterSubsystem.CurrentGoalMode = mode;
      ShooterSubsystem.table.getEntry("GoalMode").setString(ShooterSubsystem.CurrentGoalMode.toString());
      ShooterSubsystem.table.getEntry("GoalSpeed").setDouble(ShooterSubsystem.getTargetVelocity());

    }
    public static double getTargetVelocity() {
      return ShooterSubsystem.CurrentGoalMode == GoalMode.HighGoal ? 
      ShooterSubsystem.HighGoalShooterSpeed:
      ShooterSubsystem.LowGoalShooterSpeed;
    }
  
    public static void ToggleGoalMode() {
      ShooterSubsystem.setGoalMode(
      ShooterSubsystem.CurrentGoalMode == GoalMode.HighGoal? 
          GoalMode.LowGoal : GoalMode.HighGoal );
  
  }
  

  public ShooterSubsystem(BeltSubsystem beltSubsystem) {
    this.m_beltSubsystem = beltSubsystem;
    this.init();
  }
  
  private void init(){
    m_shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
    m_shooterMotor.restoreFactoryDefaults();
    m_shooterEncoder= this.m_shooterMotor.getEncoder();
    
   
  }
  public double getVelocity(){
  return this.m_shooterEncoder.getVelocity();
  }
  
  public void increaseVelocity(double inc) {
    this.m_velocity = Math.min(1, this.m_velocity+inc);
    // this increases the speed of the shooter motor
  }

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

  public void runShooter() {
    this.runShooter(m_velocity);
  }

  public void runShooter(double power){
    //System.out.println("startShooter");
    m_shooterMotor.set(power);  
    m_isShooterOn = true;      
  }

  public static double rpmToPower(double rpm, int direction /*1 or -1*/) {
    return direction * rpm/3600;

  }

  //we have some data to show that max RPM of the shooter is about 3600
  
  public void runShooterRmp(double rpm, double control) {
     
     this.runShooter( rpmToPower(rpm, 1)+control);
  }

  public void stopShooter(){
    System.out.println("stopShooter");
    m_shooterMotor.set(0.0);  
    m_isShooterOn = false;
  }

  
  //new stuff
 
  public void toggleShooter(){ 
    if(m_isShooterOn == false){
      runShooter();
    }
    else{
      stopShooter(); 
    }
  }

  
  // public void toggleQueue2(){ 
  //   if(m_isQueue2On == false){
  //     startQueueFeederWheel();
  //   }
  //   else{
  //     stopQueueFeederWheel(); 
  //   }
  // }

  public Subsystem getBeltSubsystem() {
    return this.m_beltSubsystem;
  }

}