// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAnglePidCommand extends PIDCommand {
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("TurnAnglePidCommand");

  
  private DriveTrainSubsystem m_dDriveTrainSubsystem;
  private double m_angleToTurn;

  private final static double kP = 1;
  private final static double kI = 1;
  private final static double kD = 0;

  /** Creates a new TurnAnglePidCommand. */
  public TurnAnglePidCommand( DriveTrainSubsystem driveTrainSubsystem, double angleToTurn) {
    super(
        // The controller that the command will use
        new PIDController(kD, kI, kD),
        // This should return the measurement
        () -> driveTrainSubsystem.getAngle(),
        // This should return the setpoint (can also be a constant)
        driveTrainSubsystem.getAngle()+angleToTurn,
        // This uses the output
        output -> {
          driveTrainSubsystem.tankDrive(-output, output);
        });

    this.m_dDriveTrainSubsystem = driveTrainSubsystem;
    this.m_angleToTurn = angleToTurn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_dDriveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    this.table.getEntry("atSetpoint").setBoolean(this.m_controller.atSetpoint());

    return this.m_controller.atSetpoint();
  }

  @Override
  public void execute() {
    this.table.getEntry("setpoint").setDouble( m_setpoint.getAsDouble());
    this.table.getEntry("angle").setDouble(m_measurement.getAsDouble());
    super.execute();
  }
}
