// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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
    private Translation2d[] m_translations = new Translation2d[] {
            new Translation2d(kXOffset, -kYOffset),
            new Translation2d(-kXOffset, -kYOffset),
            new Translation2d(-kXOffset, kYOffset),
            new Translation2d(kXOffset, kYOffset) };

    private boolean[] m_inverted = new boolean[] { false, true, true, false };

    private int NumModules = 4;

    // For convienence in testing and addressing a single module just using a number
    // 0 to 3
    private final SwerveModule[] m_modules = new SwerveModule[NumModules];

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    // Order of modules starts at front right and goes clockwise
    private final SwerveDriveKinematics m_kinematics;

    private final SwerveDriveOdometry m_odometry;

    public Drivetrain() {

        CommandScheduler.getInstance().registerSubsystem(this);

        int goodModule = 0;
        for (int i = 0; i < NumModules; i++) {
            try {

                m_modules[goodModule] = new SwerveModule(
                        // TODO: Assume if drive is inverted then steer is inverted?
                        createMotorController(i, m_inverted[i]), // Drive motor
                        new CANEncoder(i, m_inverted[i]), // Drive encoder
                        createMotorController(i + 1, m_inverted[i + 1]), // Steer motor
                        new CANEncoder(i + 1, m_inverted[i + 1]), // Steer encoder
                        m_translations[goodModule]);

                // If there were no errors (which jump to catch below), then we get here
                System.out.println("MODULE " + i + ": ONLINE");
                goodModule++;

            } catch (java.lang.RuntimeException error) {
                System.out.println("MODULE " + i + ": OFFLINE     - " + error.getMessage());
            }
        }

        // Now we know how many swerve modules we actually have functional, reset all
        // the arrays to be this new length
        NumModules = goodModule;

        // The SwerveDriveOdometry class uses the SwerveModulePosition class to know
        // where each module is. This class is simply an angle and distance from the
        // center of the robot.
        SwerveModulePosition[] positions = new SwerveModulePosition[NumModules];
        Translation2d[] newTranslations = new Translation2d[NumModules];

        for (int i = 0; i < NumModules; i++) {
            positions[i] = m_modules[i].getModulePosition();
            newTranslations[i] = m_translations[i];
        }

        m_translations = newTranslations;

        m_kinematics = new SwerveDriveKinematics(m_translations);
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), positions);

        m_gyro.reset();
    }

    /**
     * Create the right kind of motor controller to match the hardware
     * 
     * @param canID
     * @return
     */
    private MotorController createMotorController(int canID, boolean inverted) {
        CANSparkMax controller = new CANSparkMax(canID, MotorType.kBrushed);
        controller.setIdleMode(IdleMode.kBrake);
        controller.setInverted(inverted);

        // Alaternative motor controller implementations:
        // return new WPI_TalonSRX(turningEncoderChannelB);
        // return new WPI_VictorSPX(turningEncoderChannelB);

        return controller;
    }

    /**
     * Turn a module to a specific angle.
     * 
     * @param module      0 to 3 starting with front right
     * @param turnDegrees The angle in degrees, usually -90 to 90
     */
    public void turnModule(int module, double turnDegrees) {
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

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        for (int i = 0; i < NumModules; i++) {
            m_modules[i].setDesiredState(swerveModuleStates[i]);
        }
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
        m_modules[module].rawInput(drive, rot);
    }

    // --------------------------------------------------------------------------

    @Override
    public void periodic() {
    }

}
