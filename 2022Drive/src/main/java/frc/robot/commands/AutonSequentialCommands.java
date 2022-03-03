// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonSequentialCommands extends SequentialCommandGroup {
  /** Creates a new AutonSequentialCommands. */
  private DriveTrainSubsystem m_tankDriveSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private LimelightVisionSubsystem m_limelightVisionSubsystem;
  private double feetTotal = 0;
  private double angleTotal = 0;
  public AutonSequentialCommands(DriveTrainSubsystem tankDriveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem limelightVisionSubsystem) {
    m_limelightVisionSubsystem = limelightVisionSubsystem;
    m_tankDriveSubsystem = tankDriveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_tankDriveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_limelightVisionSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    
    addCommands(
      // all commands will go here with commas after them.
      
      TurnLimelightOff(),
      TurnLimelightOn(),
      followlimelight(),
      TurnLimelightOff(),
      drive(1)
      
    
    
    
    
    
      );
    
  }
  public DriveDistancePidCommand  drive(double feet){
    feetTotal += feet;
    System.out.println(feetTotal);
    return new DriveDistancePidCommand(m_tankDriveSubsystem, feetTotal);
  }
  public TurnAnglePidCommand  turn(double angle){
  angleTotal += angle;
  System.out.println(angleTotal);
    return new TurnAnglePidCommand(m_tankDriveSubsystem, angleTotal);
  }
  public FollowLimelightPidCommand followlimelight(){
    
    return new FollowLimelightPidCommand(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
  public InstantCommand TurnLimelightOn(){

  return new InstantCommand(
    ()->{
      
      System.out.println("limelight start");
      m_limelightVisionSubsystem.turnOnLed();
    
  });
  }
  public InstantCommand TurnLimelightOff(){

    return new InstantCommand(
      ()->{
        System.out.println("limelight stop");
        m_limelightVisionSubsystem.turnOffLed();
        // the following lines will be removed later
        m_tankDriveSubsystem.zeroEncoders();
        feetTotal=0;
        angleTotal=0;
    });
    }
  
}

