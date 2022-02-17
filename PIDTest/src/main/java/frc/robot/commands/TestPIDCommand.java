// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TestSingleMotorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPIDCommand extends PIDCommand {
  public static double TargetSetpoint = 0;
  public static final double TargetIncrement = 50;
  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("TestPIDCommand");

  
  /** Creates a new TestPIDCommand. */
  public TestPIDCommand(TestSingleMotorSubsystem motorSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(1.0/250.0, 0, 0),
        // This should return the measurement
        () -> motorSubsystem.getMeasurement(),
        // This should return the setpoint (can also be a constant)
        () -> TestPIDCommand.TargetSetpoint,
        // This uses the output
        output -> {
          motorSubsystem.go(output);
        });


    addRequirements(motorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // private double getSetpoint() {
  //   return this.m_setpoint;
  // }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void enable() {
    
  }

  public void disable() {

  }

  public static void increaseSetpoint() {
    TargetSetpoint += TargetIncrement;
  }

  public static void decreaseSetpoint() {
    TargetSetpoint -= TargetIncrement;
  }
}
