 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 //comment 

public class LimelightVisionSubsystem extends SubsystemBase {
  private NetworkTableInstance m_tableInst = NetworkTableInstance.getDefault();
  private NetworkTable m_limelightTable = m_tableInst.getTable("limelight");
  private NetworkTable table = m_tableInst.getTable("LimelightVisionSubsystem");
  
  private double m_targetOffsetAngle_Horizontal;
  private double m_targetOffsetAngle_Vertical;
  //private double targetArea;
  //private double targetSkew;
  private boolean m_hasTarget = false;
  
  /** Creates a new LimelightVisionSubsystem. */
  public LimelightVisionSubsystem() {
  }

  public double getHorizontalAngle() {
    System.out.println("angleH, " + this.m_targetOffsetAngle_Horizontal);
    return this.m_targetOffsetAngle_Horizontal;
  }
  public double getVerticalAngle() {
    System.out.println("angleV, " + this.m_targetOffsetAngle_Vertical);
    return this.m_targetOffsetAngle_Vertical;
  }

  public boolean hasTarget() {
    System.out.println("hasTarget, "+this.m_hasTarget);
    return this.m_hasTarget;
  }

  public void turnOnLed() {
    this.m_limelightTable.getEntry("ledMode").setNumber(3);
  }

  public void turnOffLed() {
    this.m_limelightTable.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.m_targetOffsetAngle_Horizontal = m_limelightTable.getEntry("tx").getDouble(0.0);
    this.m_targetOffsetAngle_Vertical = m_limelightTable.getEntry("ty").getDouble(0.0);
    // this.targetArea = m_limelightTable.getEntry("ta").getDouble(0.0);
    // this.targetSkew = m_limelightTable.getEntry("ts").getDouble(0.0);
    this.m_hasTarget = 0.0 == m_limelightTable.getEntry("tv").getDouble(0.0);

    table.getEntry("HorizontalOffsetAngle").setDouble(this.m_targetOffsetAngle_Horizontal);
    table.getEntry("VerticalOffsetAngle").setDouble(this.m_targetOffsetAngle_Vertical);
    
  }
}
