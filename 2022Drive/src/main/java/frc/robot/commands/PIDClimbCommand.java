// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDClimbCommand extends PIDCommand {
  private  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private  NetworkTable table = inst.getTable("PidClimbCommand");

  private ClimberSubsystem m_climberSubsystem;
  private final static double kP = 3;
  private final static double kI = 1;
  private final static double kD = 0;
  private final static double rotationTolerance =0.1;
  /** Creates a new PIDClimbCommand. 
     **/
  public PIDClimbCommand(ClimberSubsystem m_climberSubsystem, double distance) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> m_climberSubsystem.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          // Use the output here
        m_climberSubsystem.climb(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(rotationTolerance);

    this.table.getEntry("distanceToDrive").setDouble(Constants.CLIMBDISTANCE);
    
    this.table.getEntry("controllerSetPoint").setDouble(this.m_controller.getSetpoint());
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFin = this.m_controller.atSetpoint();
    this.table.getEntry("atSetpoint").setBoolean(isFin);
    return isFin;
  }
}
