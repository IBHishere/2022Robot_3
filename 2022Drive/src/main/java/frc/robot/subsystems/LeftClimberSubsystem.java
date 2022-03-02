// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import java.lang.Math;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class LeftClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private static final double defaultSpeedLimit = 1;
  
  private CANSparkMax m_climberLeft;
  private RelativeEncoder m_climberLeftEncoder;
  
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("ClimberSubsystem");
  
  
  public LeftClimberSubsystem() {
    this.init();
  }
  
  private void init(){
    m_climberLeft = new CANSparkMax(Constants.CLIMBER_MOTOR_CANLEFT_ID, MotorType.kBrushless);
    m_climberLeft.restoreFactoryDefaults();
    m_climberLeft.setIdleMode(IdleMode.kBrake);
    m_climberLeft.setInverted(true);
   // m_climbGroup = new MotorControllerGroup(m_climberLeft,m_climberRight);
  
    m_climberLeftEncoder= this.m_climberLeft.getEncoder();
  }
  
  public double getLeftPosition() {
    double pos = this.m_climberLeftEncoder.getPosition() ; 
    this.table.getEntry("posclimber").setDouble(pos);
    return  pos;
  }

  public void climbLeft(double speed) {
    this.climbLeft(speed, defaultSpeedLimit);
  }

  public void climbLeft(double speed, double speedLimit){
    speed = Math.signum(speed) * Math.min(speedLimit, Math.abs(speed));
    this.m_climberLeft.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
