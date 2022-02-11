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
  

  public ShooterSubsystem() {
    this.init();

  }
  private void init(){
    m_shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
    m_queueMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);

    m_shooterMotor1.restoreFactoryDefaults();
    m_queueMotor1.restoreFactoryDefaults();


  }
  
 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void  startShooter(){
    m_shooterMotor1.set(1);    


  }
  public void  stopShooter(){
    m_shooterMotor1.set(0);  

  
}
public void  startQueue(){
  m_queueMotor1.set(1);  

}
public void  stopQueue(){
  m_queueMotor1.set(0); 


}
}

