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
      MiddlePath,
      LowGoal
    }

    public static final double HighGoalShooterSpeed = 3800*.8; // max speed based n testing
    public static final double MiddlePathShooterSpeed = 3800*.8/*.8*/; // max speed based n testing
    public static final double LowGoalShooterSpeed = 3800*.8/*.4*/;
    
    public static GoalMode CurrentGoalMode =  ShooterSubsystem.GoalMode.HighGoal ;

    static {
      ShooterSubsystem.setGoalMode(ShooterSubsystem.GoalMode.HighGoal);
    }

    public static void setGoalMode(GoalMode mode) { 
      table.getEntry("insideSetGoalMode").setValue(mode.toString());
      ShooterSubsystem.CurrentGoalMode = mode;
      ShooterSubsystem.table.getEntry("GoalMode").setString(ShooterSubsystem.CurrentGoalMode.toString());
      ShooterSubsystem.table.getEntry("GoalSpeed").setDouble(ShooterSubsystem.getTargetVelocity());

    }
    public static double getTargetVelocity() {
      if(ShooterSubsystem.CurrentGoalMode == GoalMode.HighGoal ) return ShooterSubsystem.HighGoalShooterSpeed;
      if(ShooterSubsystem.CurrentGoalMode == GoalMode.MiddlePath ) return ShooterSubsystem.MiddlePathShooterSpeed;

      return ShooterSubsystem.LowGoalShooterSpeed;
    }
  
    public static void ToggleGoalMode() {
      table.getEntry("test").setValue(ShooterSubsystem.CurrentGoalMode == GoalMode.HighGoal);
      
      table.getEntry("before").setValue(ShooterSubsystem.CurrentGoalMode.toString());
      
      if(ShooterSubsystem.CurrentGoalMode == GoalMode.HighGoal ){
        table.getEntry("action").setValue("change to mid");
      
         ShooterSubsystem.setGoalMode(GoalMode.MiddlePath);}

      else if(ShooterSubsystem.CurrentGoalMode == GoalMode.MiddlePath ){
        table.getEntry("action").setValue("change to low");
        ShooterSubsystem.setGoalMode(GoalMode.LowGoal);}

      else if(ShooterSubsystem.CurrentGoalMode == GoalMode.LowGoal ){ 
        table.getEntry("action").setValue("change to high");
      ShooterSubsystem.setGoalMode(GoalMode.HighGoal);}
  
      table.getEntry("after").setValue(ShooterSubsystem.CurrentGoalMode.toString());
      
  
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