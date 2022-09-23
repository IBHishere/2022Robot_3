// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.LinearSetpointTrajectory;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.QueueFeederWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Timer;

import org.ejml.equation.IntegerSequence;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  /** Creates a new ShootSequence. */
  private ShooterSubsystem m_shooterSubsystem;
  private BeltSubsystem m_beltSubsystem;
  private QueueFeederWheelSubsystem m_queueFeederSubsytem;

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("ShootSequence");

  public ShootSequence(
    ShooterSubsystem shooterSubsystem, 
    BeltSubsystem beltSubsystem,
    QueueFeederWheelSubsystem queueFeederWheelSubsystem
    ) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_beltSubsystem = beltSubsystem;
    this.m_queueFeederSubsytem = queueFeederWheelSubsystem;


    addRequirements(m_shooterSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      ShootSequence1()
      //TestShootSequence()
    );
  }

  private Command TestShootSequence() {
    return new InstantCommand( () -> 
    this.m_queueFeederSubsytem.startQueueFeederWheel() )
    .andThen( new WaitCommand(3))
    .andThen( ()-> this.m_queueFeederSubsytem.stopQueueFeederWheel());

  }

  public Command ShootSequence1()
  {

    return 
        new InstantCommand( 
          ()-> {this.m_shooterSubsystem.runShooter( 
                  ShooterSubsystem.rpmToPower(ShooterSubsystem.getTargetVelocity(), 1));
                  //this.m_queueFeederSubsytem.startQueueFeederWheel(); 
          }
      )
      .andThen( new WaitCommand(1))
      .andThen( ()-> table.getEntry("speed").setDouble(this.m_shooterSubsystem.getVelocity()))
      .andThen(
          new InstantCommand( ()-> {
              this.m_beltSubsystem.startBelt(1);
            }
          )
      .andThen( new WaitCommand(1.0))
          .andThen( new InstantCommand( ()-> {
              this.m_shooterSubsystem.stopShooter();
              this.m_beltSubsystem.stopBelt();
              this.m_queueFeederSubsytem.stopQueueFeederWheel();
            } )) 
        )//end parallel group
      ;
      
    }
     public SequentialCommandGroup onlyShoot(){
       return new InstantCommand (()-> this.m_shooterSubsystem.runShooter() ).andThen(new WaitCommand(3).andThen(new InstantCommand(()-> this.m_shooterSubsystem.runShooter())));

     }
}

