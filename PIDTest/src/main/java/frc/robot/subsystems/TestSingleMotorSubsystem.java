// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSingleMotorSubsystem extends SubsystemBase {
  private CANSparkMax m_testMotor = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder m_eEncoder = this.m_testMotor.getEncoder();
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("TestSingleMotorSubsystem");

  
  /** Creates a new TestSingleMotorSubsystem. */
  public TestSingleMotorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getMeasurement() {
    double meas = this.m_eEncoder.getPosition();
    table.getEntry("measurement").setDouble(meas);
    return meas;
  }

  public void go(double output) {
    table.getEntry("output").setDouble(output);
    
    this.m_testMotor.setVoltage(output);
  }

  public void setVoltage(double voltage) {
    table.getEntry("output").setDouble(voltage);
    this.m_testMotor.setVoltage(voltage);
  }

  public double getPosition() {
      return this.m_eEncoder.getPosition();
  }

  public double getVelocity() {
      return this.m_eEncoder.getVelocity();
  }
}
