// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class TestPIDSpeedSubsystem extends PIDSubsystem {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("TestPIDSpeedSubsystem");

  static double kP = 1.0/250.0;
  static double kI = 0;
  static double kD = 0;
  public static final double kToleranceRPS = 50;
  public static final double kShooterFreeRPS = 5300;
  public static final double kTargetRPS = 1000;
  public static final double kMaxRPS = 6000;
  public static final double kTargetIncrement = 250;
  public static final double kShooterToleranceRPS = 50;

  public static final double kSVolts = 0.05;
  public static final double kVVoltSecondsPerRotation =
      // Should have value 12V at free speed...
      12.0 / kShooterFreeRPS;

  private double m_target = kTargetRPS;

  private TestSingleMotorSubsystem m_singleMotorSubsystem; 
  /** Creates a new TestPIDSubsytem. */
  public TestPIDSpeedSubsystem(TestSingleMotorSubsystem singleMotorSubsystem) {
    super(
        // The PIDController used by the subsystem
        new PIDController(kP, kI, kD));

    getController().setTolerance(kToleranceRPS);
    setSetpoint(kTargetRPS);

    this.m_singleMotorSubsystem =singleMotorSubsystem;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    double voltage = output;// + m_feedforward.calculate(setpoint);
    this.table.getEntry("output").setDouble(output);
    this.table.getEntry("setpoint").setDouble(setpoint);
    this.table.getEntry("voltage").setDouble(voltage);
    this.m_singleMotorSubsystem.setVoltage(voltage);
  }

  @Override
  public double getMeasurement() {
    double measurement = this.m_singleMotorSubsystem.getVelocity();
    this.table.getEntry("velocity meaurement").setDouble(measurement);
    return measurement;
  }

  public boolean atSetpoint() {
    boolean atS = this.m_controller.atSetpoint();
    this.table.getEntry("atSetpoint").setBoolean(atS);
    
    return atS;
  }

  public void increaseSetpoint() {
      this.m_target += this.m_target < kMaxRPS ? kTargetIncrement : 0;
      this.setSetpoint(this.m_target);
       System.out.println("Speed changed to: " + this.m_target);
  }

  public void decreaseSetpoint() {
    this.m_target += this.m_target > - kMaxRPS ? -kTargetIncrement : 0;
    this.setSetpoint(this.m_target);
    System.out.println("Speed changed to: " + this.m_target);
  }

}
