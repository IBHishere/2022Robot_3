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

public class RightClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private static final double defaultSpeedLimit = .5;
  
  private CANSparkMax m_climberRight;
  private RelativeEncoder m_climberRightEncoder ;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("ClimberSubsystem");
  
  
  public RightClimberSubsystem() {
    this.init();
  }
  
  private void init(){
    m_climberRight = new CANSparkMax(Constants.CLIMBER_MOTOR_CANRIGHT_ID, MotorType.kBrushless);
    m_climberRight.restoreFactoryDefaults();
    m_climberRight.setIdleMode(IdleMode.kBrake);
    
    m_climberRightEncoder= this.m_climberRight.getEncoder();
  }
  public double getRightPosition() {
    double pos = this.m_climberRightEncoder.getPosition() ; 
    this.table.getEntry("posclimber").setDouble(pos);
    return  pos;
  }

  public void climbRight(double speed) {
    this.climbRight(speed, defaultSpeedLimit);
  }

  public void climbRight(double speed, double speedLimit){
    speed = Math.signum(speed) * Math.min(speedLimit, Math.abs(speed));
    this.m_climberRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
