// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second





  // Offset lengths from the center to the wheels
  // Robot orientation is x forward, y left
  float x = 0.2365375f;
  float y = 0.22225f;

  // Rivet's convention is we start from the front right swerve module and go
  // clockwise from there
  // for Encoder inputs and CAN IDs
  private final Translation2d m_frontRightLocation = new Translation2d(x, -y);
  private final Translation2d m_backRightLocation = new Translation2d(-x, -y);
  private final Translation2d m_backLeftLocation = new Translation2d(-x, y);
  private final Translation2d m_frontLeftLocation = new Translation2d(x, y);

  private final SwerveModule m_frontRight = new SwerveModule(1, 2, -1, -1, 0, 1);
  private final SwerveModule m_backRight = new SwerveModule(3, 4, -1, -1, 2, 3);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, -1, -1, 4, 5);
  private final SwerveModule m_frontLeft = new SwerveModule(7, 8, -1, -1, 6, 7);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  // Order of modules starts at front right and goes clockwise
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontRightLocation, m_backRightLocation, m_backLeftLocation, m_frontLeftLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public Drivetrain() {
    addChild("Front Right Turn", m_frontRight.getEncoder());
    addChild("Back Right Turn", m_backRight.getEncoder());
    addChild("Back Left Turn", m_backLeft.getEncoder());
    addChild("Front Left Turn", m_frontLeft.getEncoder());
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontRight.setDesiredState(swerveModuleStates[0]);
    m_backRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_frontLeft.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  // public void updateOdometry() {
  // m_odometry.update(
  // m_gyro.getRotation2d(),
  // m_frontLeft.getState(),
  // m_frontRight.getState(),
  // m_backLeft.getState(),
  // m_backRight.getState());
  // }

  public void control(int SwerveModuleNumber, double drive, double rot) {
    SwerveModule currentswervemodule;
    currentswervemodule = null;

    if (SwerveModuleNumber == 0) {
      currentswervemodule = m_frontRight;
    }

    if (SwerveModuleNumber == 1) {
      currentswervemodule = m_backRight;
    }

    if (SwerveModuleNumber == 2) {
      currentswervemodule = m_backLeft;
    }

    if (SwerveModuleNumber == 3) {
      currentswervemodule = m_frontLeft;
    }

    currentswervemodule.rawInput(drive, rot);

  }

}
