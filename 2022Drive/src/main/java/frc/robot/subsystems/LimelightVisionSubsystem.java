// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
 // comment 

// public class LimelightVisionSubsystem extends SubsystemBase {
//   private NetworkTableInstance m_tableInst = NetworkTableInstance.getDefault();
//   private NetworkTable m_limelightTable = m_tableInst.getTable("limelight");
//   /** Creates a new LimelightVisionSubsystem. */
//   public LimelightVisionSubsystem() {
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     double targetOffsetAngle_Horizontal = m_limelightTable.getEntry("tx").getDouble(0.0);
//     double targetOffsetAngle_Vertical = m_limelightTable.getEntry("ty").getDouble(0.0);
//     double targetArea = m_limelightTable.getEntry("ta").getDouble(0.0);
//     double targetSkew = m_limelightTable.getEntry("ts").getDouble(0.0);
  
//   }
// }
