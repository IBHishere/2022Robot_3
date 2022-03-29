// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.LinearSetpointTrajectory;
import frc.robot.PidSettings;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDClimbCommand extends PIDCommand {
  private  NetworkTableInstance inst;
  private  NetworkTable table;

  private ClimberSubsystem m_climberSubsystem;
    
  // private final static double kP = .5/(Constants.CLIMBDISTANCE);
  // private final static double kI = 0;
  // private final static double kD = .3;
  private final static double rotationTolerance =3;

  private boolean m_doesItEnd;

  private BooleanSupplier m_isSetpointSettingDone;
  
  /** Creates a new PIDClimbCommand. 
     **/
  public PIDClimbCommand(
      ClimberSubsystem m_climberSubsystem, 
      LinearSetpointTrajectory trajectory,
      double climbSpeed,
      boolean doesItEnd, // if set to false this will never end and will try to hold up
      String logName,
      PidSettings pidSettings)
  {
    this(
      m_climberSubsystem,
      trajectory::getSetpoint,
      climbSpeed,
      doesItEnd,
      logName,
      trajectory::isSetpointSettingDone,
      pidSettings
    );
  }
  
  public PIDClimbCommand(
    ClimberSubsystem m_climberSubsystem, 
    DoubleSupplier setpoint,
    //double distance, 
    double climbSpeed,
    boolean doesItEnd, // if set to false this will never end and will try to hold up
    String logName,
    BooleanSupplier isSetpointSettingDone, 
    PidSettings pidSettings
  ) {
    super(
        // The controller that the command will use
        new PIDController(pidSettings.kP, pidSettings.kI, pidSettings.kD),
        // This should return the measurement
        ()-> { 
          double pos = m_climberSubsystem.getPosition();
          System.out.println("CL-pos, " + pos);
          
          return pos;
        },
        // This should return the setpoint (can also be a constant)
        //() -> distance,
        () -> {
          double s = setpoint.getAsDouble();
          NetworkTableInstance.getDefault().getTable("PidClimbCommand_"+logName).getEntry("setpoint").setDouble(s);
          return s;
        },
        // This uses the output
        output-> { 
          NetworkTableInstance.getDefault().getTable("PidClimbCommand_"+logName).getEntry("output").setDouble(output);
          m_climberSubsystem.climb(output, climbSpeed);
        }
        );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    getController().setTolerance(rotationTolerance);

    this.inst = NetworkTableInstance.getDefault();
    this.table = inst.getTable("PidClimbCommand_"+logName);
    this.m_doesItEnd = doesItEnd;
    this.m_isSetpointSettingDone = isSetpointSettingDone;

    // Configure additional PID options by calling `getController` here.
    
    this.table.getEntry("distanceToDrive").setDouble(Constants.CLIMBDISTANCE);
    
    this.table.getEntry("controllerSetPoint").setDouble(this.m_controller.getSetpoint());
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!this.m_isSetpointSettingDone.getAsBoolean() ) return false;

    boolean isFin = this.m_controller.atSetpoint();
    this.table.getEntry("atSetpoint").setBoolean(isFin);
    return isFin && this.m_doesItEnd;
  }
}

