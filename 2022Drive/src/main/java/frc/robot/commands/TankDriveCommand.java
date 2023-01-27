// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveTrainSubsystem;

// public class TankDriveCommand extends CommandBase {
//   /** Creates a new TankDriveCommand. */

//   private DriveTrainSubsystem m_driveTrainSubsystem;
//   private DoubleSupplier m_getLeftY;
//   private DoubleSupplier m_getRightY;
//   private int countCall = 0;

//   // private NetworkTableInstance inst = NetworkTableInstance.getDefault();
//   private NetworkTable table = inst.getTable("TankDriveCommand_1");

//   //public TankDriveCommand(DriveTrainSubsystem driveSubsystem, DoubleSupplier getLeftY, DoubleSupplier getRightY) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(driveSubsystem);
    
//     this.table.getEntry("init").setString("done");
//     this.m_driveTrainSubsystem = driveSubsystem;
//     this.m_getLeftY = getLeftY;
//     this.m_getRightY = getRightY;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     this.table.getEntry("countCall").setNumber(this.countCall++);
//     double test1 = this.m_getLeftY.getAsDouble();
//     this.table.getEntry("m_getLeftY").setNumber(test1);
    
//     this.m_driveTrainSubsystem.tankDrive(this.m_getLeftY.getAsDouble(), this.m_getRightY.getAsDouble());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     this.m_driveTrainSubsystem.tankDrive(0.0, 0.0);
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
