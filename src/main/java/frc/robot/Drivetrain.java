// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

    // Wheel RPMs on the Swerve & Steer page are 800.
    public static final double kWheelRpms = 800;
    public static final double kWheelDiameter = 4.0 * 0.0254; // inches * meters/inch
    // Should be: <motor free speed rpms> / 60 * <wheel diam> * PI

    // Theoretical max speed is about 4.3 m/s, but when running full throttle,
    // we're seeing a max speed of 700 RPMs, so it looks more like 3.7 m/s
    // Set an initial value that is safer - no bumpers yet
    public static final double kMaxSpeed = 2.0;

    // Swerve and Steer has a THEORETICAL max Turn RPM of 90
    // 90 / 60 * 2 * PI = 9.4 Radians/sec
    // We'll start with a max speed much smaller - 2 rads/sec
    public static final double kMaxAngularSpeed = 2.0;

    // Offset lengths from the center to the wheels
    // Robot orientation is x forward, y left, with swerve modules numbered
    //
    // ......................^
    // ......................|
    // .....................(X)
    // ......................|
    // ................[3] front [0]
    // .......<---(Y)--------+------------->
    // ................[2] back_ [1]
    // ......................|
    // ......................|
    // ......................v
    //
    // Our swerve bot frame is 24" square, but the swerve modules position the
    // wheels in a rectangle
    public static final float kXOffset = 0.2365375f; // in meters
    public static final float kYOffset = 0.22225f; // in meters

    // Rivet's convention is we start from the front right swerve module and go
    // clockwise from there
    // for Encoder inputs and CAN IDs
    private final Translation2d m_frontRightLocation = new Translation2d(kXOffset, -kYOffset);
    private final Translation2d m_backRightLocation = new Translation2d(-kXOffset, -kYOffset);
    private final Translation2d m_backLeftLocation = new Translation2d(-kXOffset, kYOffset);
    private final Translation2d m_frontLeftLocation = new Translation2d(kXOffset, kYOffset);

    // For convienence in testing and addressing a single module just using a number
    // 0 to 3
    private final SwerveModule[] m_modules = new SwerveModule[4];

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    // Order of modules starts at front right and goes clockwise
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontRightLocation, m_backRightLocation, m_backLeftLocation, m_frontLeftLocation);

    private final SwerveDriveOdometry m_odometry;

    public Drivetrain() {

        CommandScheduler.getInstance().registerSubsystem(this);

        m_modules[0] = new SwerveModuleCANCoder(
                createMotorController(0), createSteerController(4),
                new WPI_CANCoder(0), new WPI_CANCoder(4), m_frontRightLocation);
        m_modules[1] = new SwerveModuleCANCoder(
                createMotorController(1), createSteerController(5),
                new WPI_CANCoder(1), new WPI_CANCoder(5), m_backRightLocation);
        m_modules[2] = new SwerveModuleCANCoder(
                createMotorController(2), createSteerController(6),
                new WPI_CANCoder(2), new WPI_CANCoder(6), m_backLeftLocation);
        m_modules[3] = new SwerveModuleCANCoder(
                createMotorController(3), createSteerController(7),
                new WPI_CANCoder(3), new WPI_CANCoder(7), m_frontLeftLocation);

        SwerveModulePosition[] positions = new SwerveModulePosition[] {
                m_modules[0].getModulePosition(),
                m_modules[1].getModulePosition(),
                m_modules[2].getModulePosition(),
                m_modules[3].getModulePosition()
        };

        m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(),
                positions);

        m_gyro.reset();
    }

    /**
     * Create the right kind of motor controller to match the hardware
     * 
     * @param canID
     * @return
     */
    private MotorController createMotorController(int canID) {
        return new CANSparkMax(canID, MotorType.kBrushed);
        // return new WPI_TalonSRX(turningEncoderChannelB);
        // return new WPI_VictorSPX(turningEncoderChannelB);
    }

    private MotorController createSteerController(int canID) {
        return new CANSparkMax(canID, MotorType.kBrushed);
        // return new WPI_TalonSRX(turningEncoderChannelB);
        // return new WPI_VictorSPX(turningEncoderChannelB);
    }

    /**
     * Turn a module to a specific angle.
     * 
     * @param module      0 to 3 starting with front right
     * @param turnDegrees The angle in degrees, usually -90 to 90
     */
    public void turnModule(int module, double turnDegrees) {
        System.out.println(String.format("Encoder: %5.1f, Level: %5.1f",
                m_modules[module].getTurnDistance(), turnDegrees));
        m_modules[module].setDesiredTurn(Rotation2d.fromDegrees(turnDegrees));
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

        ChassisSpeeds chassisSpeeds = null;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

        System.out.println("Chassis speeds: " + chassisSpeeds);
        System.out.println("Front right: " + swerveModuleStates[0]);
        System.out.println("Back right: " + swerveModuleStates[1]);
        System.out.println("Back left: " + swerveModuleStates[2]);
        System.out.println("Front right: " + swerveModuleStates[3]);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        m_modules[0].setDesiredState(swerveModuleStates[0]);
        m_modules[1].setDesiredState(swerveModuleStates[1]);
        m_modules[2].setDesiredState(swerveModuleStates[2]);
        m_modules[3].setDesiredState(swerveModuleStates[3]);
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

    public void control(int module, double drive, double rot) {

        System.out.println(String.format("Module/Encoder/turn/drive, %d, %8.4f, %8.2f, %8.2f",
                module,
                m_modules[module].getTurnDistance(),
                rot,
                drive));

        m_modules[module].rawInput(drive, rot);
    }

    // --------------------------------------------------------------------------

    @Override
    public void periodic() {
    }

}
