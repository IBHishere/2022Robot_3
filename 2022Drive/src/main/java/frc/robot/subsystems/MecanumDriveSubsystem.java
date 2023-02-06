// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

import java.lang.Math;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class MecanumDriveSubsystem extends SubsystemBase {

  private MecanumDrive m_myRobot;
  private CANSparkMax m_leftMotorFront;
  private RelativeEncoder m_leftFrontEncoder;
  private CANSparkMax m_rightMotorFront;
  private RelativeEncoder m_rightFrontEncoder;
  private CANSparkMax m_leftMotorBack;
  private RelativeEncoder m_leftBackEncoder;
  private CANSparkMax m_rightMotorBack;
  private RelativeEncoder m_rightBackEncoder;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem");
  public final static double highSpeedLimit = .65;
  public final static double lowSpeedLimit = .50;
  private double currentSpeed = highSpeedLimit;

  private IdleMode m_idleMode = IdleMode.kBrake;
  public boolean doLog = false;

  /** Creates a new DriveTrain. */
  public MecanumDriveSubsystem() {
    this.init();
  }

  public void setIdleMode(IdleMode idleMode) {
    this.m_idleMode = idleMode;
    m_leftMotorFront.setIdleMode(this.m_idleMode);
    m_leftMotorBack.setIdleMode(this.m_idleMode);
    m_rightMotorFront.setIdleMode(this.m_idleMode);
    m_rightMotorBack.setIdleMode(this.m_idleMode);

  }

  public IdleMode getIdleMode() {
    return this.m_idleMode;
  }

  private void init() {
    m_leftMotorFront = new CANSparkMax(Constants.LEFT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_leftMotorBack = new CANSparkMax(Constants.LEFT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_leftMotorFront.setInverted(true);
    m_leftMotorBack.setInverted(true);

    m_rightMotorFront = new CANSparkMax(Constants.RIGHT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_rightMotorBack = new CANSparkMax(Constants.RIGHT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_rightMotorFront.restoreFactoryDefaults();
    m_rightMotorBack.restoreFactoryDefaults();

    m_leftFrontEncoder = this.m_leftMotorFront.getEncoder();
    m_rightFrontEncoder = this.m_rightMotorFront.getEncoder();
    m_leftBackEncoder = this.m_leftMotorBack.getEncoder();
    m_rightBackEncoder = this.m_rightMotorBack.getEncoder();

    zeroEncoders();
    this.setIdleMode(IdleMode.kCoast);
    m_myRobot = new MecanumDrive(m_leftMotorFront, m_leftMotorBack, m_rightMotorFront, m_rightMotorBack);
  }

  public void setSpeed(double speed) {
    this.currentSpeed = speed;
  }

  public void setMode(IdleMode mode) {
    this.setIdleMode(mode);
  }

  @Override
  public void periodic() {

  }
  // This method will be called once per scheduler run

  // private double computeActualDriveFromInput(double joystickValue, double
  // speedScalingFactor) {
  // //TODO: Move this to constants?
  // double zeroLimit = 0.1;
  // double driveCurvePower = 5.0; // 3.0;

  // double outputValue = Math.abs(joystickValue) < zeroLimit ? 0 :
  // Math.pow(joystickValue, driveCurvePower) * speedScalingFactor;

  // return outputValue;
  // }

  // public void MecanumDrive(double leftJoystickValue, double rightJoystickValue)
  // {
  // this.MecanumDrive(leftJoystickValue, rightJoystickValue, this.currentSpeed);
  // }

  public void MecanumDrive(
      double leftJoystickValueY,
      double leftJoystickValueX,
      double rightJoystickValueX) {

    if (doLog)
      System.out.println("joystick-X-Y,  " + leftJoystickValueX + ", " + leftJoystickValueY);

    // limit input to [-1,1] range
    leftJoystickValueX = -Math.pow(
        Math.signum(leftJoystickValueX) * Math.min(1, Math.abs(leftJoystickValueX)) * Constants.SPEED_REGULATOR, 3);
    leftJoystickValueY = Math.pow(
        Math.signum(leftJoystickValueY) * Math.min(1, Math.abs(leftJoystickValueY)) * Constants.SPEED_REGULATOR, 3);

    rightJoystickValueX = Math.pow(
        Math.signum(rightJoystickValueX) * Math.min(1, Math.abs(rightJoystickValueX)) * Constants.TURN_REGULATOR, 3);

    if (Math.abs(leftJoystickValueX) <= Constants.DEAD_ZONE_VALUE) {
      leftJoystickValueX = 0;
    }
    if (Math.abs(leftJoystickValueY) <= Constants.DEAD_ZONE_VALUE) {
      leftJoystickValueY = 0;
    }
    if (Math.abs(rightJoystickValueX) <= Constants.DEAD_ZONE_VALUE) {
      rightJoystickValueX = 0;
    }

    // //log joystick values to network tables
    // table.getEntry("motorLeftValue").setDouble(leftJoystickValue);
    // table.getEntry("motorRightValue").setDouble(rightJoystickValue);

    // //convert joystick values to motor inputs
    // double leftMotorValue = this.computeActualDriveFromInput(leftJoystickValue,
    // speedScalingFactor);
    // double rightMotorValue = this.computeActualDriveFromInput(rightJoystickValue,
    // speedScalingFactor);

    // table.getEntry("computedMotorLeftValue").setDouble(leftMotorValue);
    // table.getEntry("computedMotorRightValue").setDouble(rightMotorValue);

    // if(doLog) System.out.println("drive-l-r, " + leftMotorValue +", " +
    // rightMotorValue);

    m_myRobot.driveCartesian(leftJoystickValueX, leftJoystickValueY, rightJoystickValueX);
    // m_myRobot.driveCartesian(0.1, 0,0);
  }

  private double getLeftFrontPosition() {
    return this.m_leftFrontEncoder.getPosition();
  }

  private double getLeftBackPosition() {
    return this.m_leftBackEncoder.getPosition();
  }

  private double getRightFrontPosition() {
    return this.m_rightFrontEncoder.getPosition();
  }

  private double getRightBackPosition() {
    return this.m_rightBackEncoder.getPosition();
  }

  public void zeroEncoders() {
    this.m_leftFrontEncoder.setPosition(0);
    this.m_leftBackEncoder.setPosition(0);
    this.m_rightFrontEncoder.setPosition(0);
    this.m_rightBackEncoder.setPosition(0);
  }
}