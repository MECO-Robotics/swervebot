// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * This type of swerve module Uses:
 * 
 * - SparkMax controllers for both drive and steer
 * - CANCoder encoders for drive and steer
 * - Mini CIMs for drive and steer
 */
public class SwerveModule implements Sendable {

    // ------------------------------------------------------------------------
    // Swerve & Steer Constants

    // // Wheel RPMs on the Swerve & Steer page are 800.
    // public static final double kWheelRpms = 800;
    // public static final double kWheelDiameter = 4.0 * 0.0254; // inches *
    // meters/inch
    // // Should be: <motor free speed rpms> / 60 * <wheel diam> * PI

    // // Theoretical max speed is about 4.3 m/s, but when running full throttle,
    // // we're seeing a max speed of 700 RPMs, so it looks more like 3.7 m/s
    // // Set an initial value that is safer - no bumpers yet
    // public static final double kMaxSpeed = 2.0;

    // // Swerve and Steer has a THEORETICAL max Turn RPM of 90
    // // 90 / 60 * 2 * PI = 9.4 Radians/sec
    // // We'll start with a max speed much smaller - 2 rads/sec
    // public static final double kMaxAngularSpeed = 2.0;

    // ------------------------------------------------------------------------
    // SDS MK4i Constants
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module

    // Steer motor ratio is 150/7 (21.4) revolutions of the motor to 1 revolution of
    // the steer direction.
    private static final double kSteerGearRatio = 21.42857142857143;

    // Drive ratio is 8.14 revolutions of the motor to 1 revolution of the drive.
    // This is the slowest of the 3 gearing options available on the MK4i
    // TODO: Confirm we're using the "L1 - Standard" Drive gear ratios
    private static final double kDriveGearRatio = 8.14;

    // Wheels are 4" OD
    private static final double kWheelDiameterMeters = 0.1016;

    // ------------------------------------------------------------------------
    // Mini CIM Constants
    // https://motors.vex.com/vexpro-motors/mini-cim-motor

    // Free speed at 12v: 5840rpms
    private static final double kMaxRpm = 5000;

    // ------------------------------------------------------------------------

    // Max angular velocity converted from RPMs to deg/s
    public static final double kSteerMaxAngularVelocity = (kMaxRpm / kSteerGearRatio) * 360.0 / 60.0;

    // Assume 0 to max RPM in 0.5 seconds, so ...
    private static final double kSteerMaxAngularAcceleration = kSteerMaxAngularVelocity / 0.5;

    // Wheel circumference in meters
    private static final double kWheelCircumference = kWheelDiameterMeters * 3.1415;

    // Max forward speed, in m/s
    public static final double kDriveMaxForwardSpeed = (kMaxRpm / kDriveGearRatio) * kWheelCircumference / 60.0;

    // Conversion factor to multiply degrees to get meters.
    public static final double kDegreesToMeters = kWheelCircumference / 360.0;

    private final Translation2d m_location;
    public final MotorController m_driveMotor; // Either CANSparkMax or WPI_TalonSRX implementation
    public final MotorController m_turningMotor;
    public final IEncoder m_driveEncoder;
    public final IEncoder m_steerEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    // This PID controller turns m/s into motor input (-1 .. 1)
    //
    // Initial kP: Using max forward velocity of 3.27m/s, Initial factor would be
    // 1/32.7 = 0.306
    public final PIDController m_drivePIDController = new PIDController(0.306, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    // Set points and constraints are in degrees
    //
    // Initial kP: Using max angular velocity of 1400 deg/sec, initial factor would
    // be 1/1400 = 1.74e-4
    public final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1.74e-4,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kSteerMaxAngularVelocity, kSteerMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);

    // "volts" here really refers to the input level to the motor of -1.0 to 1.0
    // ks is in volts. Original value: 1
    // kv is volts * seconds / radians. Original value: 0.5
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.1);

    // The steer angle of the module from the last tick
    Rotation2d lastAngle = Rotation2d.fromDegrees(0);

    // --------------------------------------------------------------------------

    /**
     * Constructs a SwerveModule with a drive motor, turning motor.
     *
     */
    public SwerveModule(
            MotorController driveMotorController,
            IEncoder driveEncoder,
            MotorController steerMotorController,
            IEncoder steerEncoder,
            Translation2d location) {

        m_driveMotor = driveMotorController;
        m_turningMotor = steerMotorController;
        m_driveEncoder = driveEncoder;
        m_steerEncoder = steerEncoder;
        m_location = location;

        // Encoder is configured to report position from -180 to 180.
        m_turningPIDController.enableContinuousInput(-180, 180);
    }

    // --------------------------------------------------------------------------

    Translation2d getTranslationFromCenter() {
        return m_location;
    }

    // --------------------------------------------------------------------------

    SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                m_driveEncoder.getDistanceDegrees() * kDegreesToMeters,
                Rotation2d.fromDegrees(m_steerEncoder.getDistanceDegrees()));
    }

    // --------------------------------------------------------------------------

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRateDegreesPerSecond(),
                new Rotation2d(m_steerEncoder.getDistanceDegrees()));
    }

    // --------------------------------------------------------------------------

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // --------------------------------------------------------------------
        // Optimize the reference state to avoid spinning further than 90 degrees
        //
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_steerEncoder.getDistanceDegrees()));

        // ----------------------------------------------------------------
        // Prevent jitter by checking if the drive speed is less than 1%.
        //
        if (Math.abs(state.speedMetersPerSecond) <= (kDriveMaxForwardSpeed * 0.01)) {
            state.angle = lastAngle;
        } else {
            lastAngle = state.angle;
        }

        setDesiredTurn(state.angle);
        setDesiredDrive(state.speedMetersPerSecond);
    }

    // --------------------------------------------------------------------------

    /**
     * Spin the drive motor to achieve a desired speed
     * 
     * @param speedMps The forward speed, in meters per second
     */
    public void setDesiredDrive(double speedMps) {

        // Calculate the drive output from the drive PID controller.
        // This applies PID control, but also effectively changes m/s into % output
        final double driveOutput = m_drivePIDController.calculate(
                m_driveEncoder.getRateDegreesPerSecond() * kDegreesToMeters,
                speedMps);

        final double driveFeedForward = m_driveFeedforward.calculate(speedMps);

        m_driveMotor.set(driveOutput + driveFeedForward);
    }
    // --------------------------------------------------------------------------

    /**
     * Steer the module to an indicated position.
     * 
     * @param turn The desired rotation (-180 to 180)
     */
    public void setDesiredTurn(Rotation2d turn) {

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_steerEncoder.getDistanceDegrees(),
                turn.getDegrees());

        // Use the desired velocity (deg/s) and run through a feedfoward controller,
        // which basically increases the velocity by 50%
        final double turnFeedForward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_turningMotor.set(turnOutput + turnFeedForward);
    }

    // --------------------------------------------------------------------------

    /**
     * Direct control of the swerve module motors, regardless of current state
     *
     * @param drive The input level -1.0 to 1.0 for the driver motor
     * @param turn  The input level -1.0 to 1.0 for the steer motor
     */
    public void rawInput(double drive, double turn) {
        m_driveMotor.set(drive);
        m_turningMotor.set(turn);
    }

    // --------------------------------------------------------------------------

    @Override
    public void initSendable(SendableBuilder builder) {
    }

}
