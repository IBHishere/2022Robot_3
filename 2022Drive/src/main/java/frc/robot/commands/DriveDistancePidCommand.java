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
public class DriveDistancePidCommand extends PIDCommand {
  private  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private  NetworkTable table = inst.getTable("DriveDistancePidCommand");

  private DriveTrainSubsystem m_dDriveTrainSubsystem;
  private double m_distanceToDrive;

  //The gearing of the motors is 6.6667 to 1.  The diameter of the wheel is 6 inches.
  private final static double empiricalAdjustment = 60.0/62.0;
  private final static double rotationsPerFeet = 6.6667/(Math.PI * 6.0 / 12.0) * empiricalAdjustment;

  private final static double kP = 3;
  private final static double kI = 1;
  private final static double kD = 0;
  private final static double rotationTolerance =0.1;

  /** Creates a new DriveDistancePidCommand. */
  public DriveDistancePidCommand( 
    DriveTrainSubsystem driveTrainSubsystem, 
    double distanceToDriveInFeet) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> driveTrainSubsystem.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> distanceToDriveInFeet*rotationsPerFeet,
        // This uses the output
        output -> {
          //System.out.println("output, " + output);
          driveTrainSubsystem.tankDrive(output, output, .3);
        }); 

    this.m_dDriveTrainSubsystem = driveTrainSubsystem;
    this.m_distanceToDrive = distanceToDriveInFeet;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_dDriveTrainSubsystem);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(rotationTolerance);

    this.table.getEntry("distanceToDriveInFeet").setDouble(distanceToDriveInFeet);
    this.table.getEntry("computedSetPoint").setDouble(distanceToDriveInFeet*rotationsPerFeet);
    this.table.getEntry("controllerSetPoint").setDouble(this.m_controller.getSetpoint());

  }

  @Override
  public void initialize() {
    m_controller.reset();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFin =  this.m_controller.atSetpoint();
    this.table.getEntry("atSetpoint").setBoolean(isFin);
    return isFin; 
  }

  @Override
  public void execute() {
    this.table.getEntry("setpoint").setDouble( m_setpoint.getAsDouble());
    this.table.getEntry("pos").setDouble(m_measurement.getAsDouble());  
    
    super.execute();
  }
}
