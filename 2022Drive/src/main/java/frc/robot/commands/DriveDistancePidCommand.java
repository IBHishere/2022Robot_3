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
public class DriveDistancePidCommand extends PIDCommand {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("DriveDistancePidCommand");

  private DriveTrainSubsystem m_dDriveTrainSubsystem;
  private double m_distanceToDrive;
  private AutonomousState m_autonomousState;

  private final static double kP = 1;
  private final static double kI = .2;
  private final static double kD = 0;
  private final static double rotationTolerance =0.1;

  /** Creates a new DriveDistancePidCommand. */
  public DriveDistancePidCommand( DriveTrainSubsystem driveTrainSubsystem, double distanceToDrive, AutonomousState autonomousState) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> driveTrainSubsystem.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> autonomousState.getPriorCommandFinishPosition() + distanceToDrive,
        // This uses the output
        output -> {
          DriveDistancePidCommand.table.getEntry("output").setDouble(output);
          driveTrainSubsystem.tankDrive(-output, -output);
        });

    this.m_dDriveTrainSubsystem = driveTrainSubsystem;
    this.m_distanceToDrive = distanceToDrive;
    this.m_autonomousState = autonomousState;
    DriveDistancePidCommand.table.getEntry("start setpoint").setDouble(autonomousState.getPriorCommandFinishPosition() + distanceToDrive);
    DriveDistancePidCommand.table.getEntry("m_distanceToDrive").setDouble(this.m_distanceToDrive);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_dDriveTrainSubsystem);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(rotationTolerance);

    DriveDistancePidCommand.table.getEntry("distanceToDrive").setDouble(distanceToDrive);
    DriveDistancePidCommand.table.getEntry("computedSetPoint").setDouble(driveTrainSubsystem.getPosition()+distanceToDrive);
    DriveDistancePidCommand.table.getEntry("controllerSetPoint").setDouble(this.m_controller.getSetpoint());

  }

  @Override
  public void initialize() {
    System.out.print("DriveDistancePidCommand " + m_controller.getSetpoint());
    m_controller.reset();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFin =  this.m_controller.atSetpoint();
    DriveDistancePidCommand.table.getEntry("atSetpoint").setBoolean(isFin);
    if(isFin)  
      this.m_autonomousState.setValues(this.m_dDriveTrainSubsystem);
    return isFin; 
  }

  @Override
  public void execute() {
    DriveDistancePidCommand.table.getEntry("setpoint").setDouble( m_setpoint.getAsDouble());
    DriveDistancePidCommand.table.getEntry("pos").setDouble(m_measurement.getAsDouble());  
    
    System.out.println(m_setpoint.getAsDouble()  + ", " + m_measurement.getAsDouble() );
    
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    this.m_autonomousState.setValues(this.m_dDriveTrainSubsystem);
    super.end(interrupted);
  }      
}
