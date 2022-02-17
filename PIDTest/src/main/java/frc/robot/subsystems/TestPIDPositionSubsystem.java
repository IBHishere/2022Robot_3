// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class TestPIDPositionSubsystem extends PIDSubsystem {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("TestPIDPositionSubsystem");

  static double kP = 1;
  static double kI = 10;
  static double kD = 0;
   public static final double kToleranceR = 50;
  public static final double kTargetIncrement = 1;
  
  private double m_target = 0;
  private TestSingleMotorSubsystem m_singleMotorSubsystem; 
  /** Creates a new TestPIDSubsytem. */
  public TestPIDPositionSubsystem(TestSingleMotorSubsystem singleMotorSubsystem) {
    super(new PIDController(kP, kI, kD));
    getController().setTolerance(kToleranceR);
    setSetpoint(singleMotorSubsystem.getPosition());
    this.m_singleMotorSubsystem = singleMotorSubsystem;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    double voltage = output;// + m_feedforward.calculate(setpoint);
    this.table.getEntry("output").setDouble(output);
    this.table.getEntry("setpoint").setDouble(setpoint);
    this.table.getEntry("voltage").setDouble(voltage);
    //System.out.println("useOutput");
    this.m_singleMotorSubsystem.setVoltage(voltage);
  }

  @Override
  public double getMeasurement() {
    double measurement = this.m_singleMotorSubsystem.getPosition();
    this.table.getEntry("Pos meaurement").setDouble(measurement);
    //System.out.println("measure");
    
    return measurement;
  }

  public boolean atSetpoint() {
    boolean atS = this.m_controller.atSetpoint();

    this.table.getEntry("atSetpoint").setBoolean(atS);
    
    return atS;
  }

  public void increaseSetpoint() {
      this.m_target += kTargetIncrement;
      this.setSetpoint(this.m_target);
      System.out.println("Pos changed to: " + this.m_target);
  }

  public void decreaseSetpoint() {
    this.m_target += -kTargetIncrement;
    this.setSetpoint(this.m_target);
    System.out.println("Pos changed to: " + this.m_target);
  }

}
