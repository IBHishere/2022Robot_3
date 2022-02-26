// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AutonomousState;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAnglePidCommand extends PIDCommand {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("TurnAnglePidCommand");

  
  private DriveTrainSubsystem m_dDriveTrainSubsystem;
  private double m_angleToTurn;
  private AutonomousState m_autonomousState;

  private final static double kP = 0.5;
  private final static double kI = .2;
  private final static double kD = 0;

  /** Creates a new TurnAnglePidCommand. */
  public TurnAnglePidCommand( DriveTrainSubsystem driveTrainSubsystem, double angleToTurn, AutonomousState autonomousState) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> driveTrainSubsystem.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> autonomousState.getPriorCommandFinishAngle()+angleToTurn,
        // This uses the output
        output -> {
          TurnAnglePidCommand.table.getEntry("output").setDouble(output);
          driveTrainSubsystem.tankDrive(-output, output);
        });

        

    this.m_dDriveTrainSubsystem = driveTrainSubsystem;
    this.m_angleToTurn = angleToTurn;
    this.m_autonomousState = autonomousState;
    TurnAnglePidCommand.table.getEntry("m_angleToTurn").setDouble(this.m_angleToTurn);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_dDriveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
  
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFin = this.m_controller.atSetpoint();
    TurnAnglePidCommand.table.getEntry("atSetpoint").setBoolean(isFin);

    if(isFin) this.m_autonomousState.setValues(this.m_dDriveTrainSubsystem);

    return isFin;
  }

  @Override
  public void execute() {
    TurnAnglePidCommand.table.getEntry("setpoint").setDouble( m_setpoint.getAsDouble());
    TurnAnglePidCommand.table.getEntry("angle").setDouble(m_measurement.getAsDouble());
    
    System.out.println(m_setpoint.getAsDouble() + ", " + m_measurement.getAsDouble());
          
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    this.m_autonomousState.setValues(this.m_dDriveTrainSubsystem);
    super.end(interrupted);
  }      
}
