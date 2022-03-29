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

  
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private double m_angleToTurn;
  
  private final static double kP = 1;
  private final static double kI = .2;
  private final static double kD = 0;

  private final static double rotationsPerDegree = 1.0/13.92;


  /** Creates a new TurnAnglePidCommand. */
  public TurnAnglePidCommand( DriveTrainSubsystem driveTrainSubsystem, double angleToTurnInDegrees) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> driveTrainSubsystem.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angleToTurnInDegrees*rotationsPerDegree,
        // This uses the output
        output -> {
          driveTrainSubsystem.tankDrive(output, -output, .3);
        });

        

    this.m_driveTrainSubsystem = driveTrainSubsystem;
    this.m_angleToTurn = angleToTurnInDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
  
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFin = this.m_controller.atSetpoint();
    this.table.getEntry("atSetpoint").setBoolean(isFin);

    return isFin;
  }

  @Override
  public void execute() {
    this.table.getEntry("setpoint").setDouble( m_setpoint.getAsDouble());
    this.table.getEntry("angle").setDouble(m_measurement.getAsDouble());
          
    super.execute();
  }
}
