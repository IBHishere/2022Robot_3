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

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private CANSparkMax m_climber1;
  private RelativeEncoder m_climber1encoder;
  private CANSparkMax m_climber2;
  private RelativeEncoder m_climber2encoder ;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("ClimberSubsystem");
  
  MotorControllerGroup m_climbGroup;
  
  public ClimberSubsystem() {
    this.init();
  }
  
  private void init(){
    m_climber1 = new CANSparkMax(Constants.CLIMBER_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_climber2 = new CANSparkMax(Constants.CLIMBER_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_climber1.restoreFactoryDefaults();
    m_climber2.restoreFactoryDefaults();
    m_climber1.setIdleMode(IdleMode.kBrake);
    m_climber2.setIdleMode(IdleMode.kBrake);
    m_climbGroup = new MotorControllerGroup(m_climber1,m_climber2);
  
    m_climber2encoder= this.m_climber2.getEncoder();
    m_climber1encoder= this.m_climber1.getEncoder();
  }
  public double getPosition() {
    double pos = (this.m_climber1encoder.getPosition() + this.m_climber2encoder.getPosition())/2.0; 
    
    this.table.getEntry("posclimber").setDouble(pos);
    return  pos;
  }
  public void climb(double speed){
    //TODO: we should probably limit the speed so that it doesn't shoot up 
    
    this.m_climbGroup.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
