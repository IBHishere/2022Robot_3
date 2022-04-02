// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private UsbCamera m_usbCameraBottom;
  private UsbCamera m_usbCameraTop;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("visionSubsytem");
    
  
  public VisionSubsystem() {
    table.getEntry("lastLog").forceSetString("init");
    m_usbCameraBottom = CameraServer.startAutomaticCapture(0);
    m_usbCameraBottom = CameraServer.startAutomaticCapture(1);
    table.getEntry("lastLog").forceSetString("init2");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
