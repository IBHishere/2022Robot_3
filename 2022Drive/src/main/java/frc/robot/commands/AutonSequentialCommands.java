// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Queue;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.QueueFeederWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonSequentialCommands extends SequentialCommandGroup {
  /** Creates a new AutonSequentialCommands. */
  private DriveTrainSubsystem m_tankDriveSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private BeltSubsystem m_beltSubsystem;
  private LimelightVisionSubsystem m_limelightVisionSubsystem;
  private QueueFeederWheelSubsystem m_queueFeederWheelSubsystem;


  private double feetTotal = 0;
  private double angleTotal = 0;
  public AutonSequentialCommands(
      DriveTrainSubsystem tankDriveSubsystem
      , IntakeSubsystem intakeSubsystem
      , ShooterSubsystem shooterSubsystem
      , LimelightVisionSubsystem limelightVisionSubsystem
      , BeltSubsystem beltSubsystem
      , QueueFeederWheelSubsystem queueFeederWheelSubsystem
      ) {
    m_limelightVisionSubsystem = limelightVisionSubsystem;
    m_tankDriveSubsystem = tankDriveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beltSubsystem = beltSubsystem;
    this.m_queueFeederWheelSubsystem = queueFeederWheelSubsystem;

    addRequirements(m_tankDriveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_limelightVisionSubsystem, m_beltSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    
    addCommands(
      // all commands will go here with commas after them.
      new InstantCommand( ()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kBrake))
      , new InstantCommand(() -> this.m_intakeSubsystem.intakePull())
      , new WaitCommand(.5)
      , drive(-5)   // pick up the second ball 
      , new InstantCommand( ()-> this.m_intakeSubsystem.intakeStop())
      , new WaitCommand(.5)
      , turn(180)  // turn to shoot the second ball // do we need to drive back to the original position
      , new WaitCommand(.5)
      , shootSequence()  // second shot
      , new InstantCommand(()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kCoast))
    
    );
  }

// NOTE: this code is never used, see yellow line
  private void getFirstAuton() {
    addCommands(
      // all commands will go here with commas after them.
      new InstantCommand( ()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kBrake))
      , drive(2.0) // move for the first shot
      , shootSequence() //shoot first shot
      , turn(180)  // turn around to pick up the second ball
      , new WaitCommand(1)
      , new InstantCommand(() -> this.m_intakeSubsystem.intakePull())
      , new WaitCommand(.5)
      , drive(-2.0)   // pick up the second ball //TODO: do we need start the intake?
      , new WaitCommand(.5)
      , new InstantCommand( ()-> this.m_intakeSubsystem.intakeStop())
      , turn(180)  // turn to shoot the second ball // do we need to drive back to the original position
      , drive(-2.0)
      , new WaitCommand(.5)
      , shootSequence()  // second shot
      , new InstantCommand(()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kCoast))
    );
  }

  private DriveDistancePidCommand drive(double feet){
    feetTotal += feet;
    //System.out.println(feetTotal);
    return new DriveDistancePidCommand(m_tankDriveSubsystem, feetTotal);
  }

  public TurnAnglePidCommand  turn(double angle){
    angleTotal += angle;
    //System.out.println(angleTotal);
    return new TurnAnglePidCommand(m_tankDriveSubsystem, angleTotal);
  }


  public FollowLimelightPidCommand followlimelight(){
    return new FollowLimelightPidCommand(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
    
  public FollowLimelightSequence limelightSequence(){
    return new FollowLimelightSequence(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
    
  public InstantCommand TurnLimelightOn(){
    return new InstantCommand(
      ()->{
        //System.out.println("limelight start");
        m_limelightVisionSubsystem.turnOnLed();
      });
  }

  public InstantCommand TurnLimelightOff(){
    return new InstantCommand(
      ()->{
       //System.out.println("limelight stop");
       m_limelightVisionSubsystem.turnOffLed();
        // the following lines will be removed later
        m_tankDriveSubsystem.zeroEncoders();
       feetTotal=0;
       angleTotal=0;
    });
    }

    public ShootSequence shootSequence(){
      return new ShootSequence(
        this.m_shooterSubsystem, this.m_beltSubsystem,
        this.m_queueFeederWheelSubsystem);
    }
    public InstantCommand intakePull(){
      return new InstantCommand(
        ()->{
      this.m_intakeSubsystem.intakePull();
      }, this.m_intakeSubsystem, this.m_beltSubsystem);
    }
    public FollowLimelightSequence followlimelightsequence(){
     return new FollowLimelightSequence(m_tankDriveSubsystem, m_limelightVisionSubsystem);
    }
    public SequentialCommandGroup goback(){
      return turn(0).andThen(drive(0)).andThen(turn(0));
    }
    public SequentialCommandGroup shootWithLimelight(){
      return followlimelightsequence().andThen(goback());
    }
}


