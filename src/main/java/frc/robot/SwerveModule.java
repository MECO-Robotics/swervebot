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

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final Translation2d m_location;
    private final MotorController m_driveMotor; // Either CANSparkMax or WPI_TalonSRX implementation
    private final MotorController m_turningMotor;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);

    // "volts" here really refers to the input level to the motor of -1.0 to 1.0
    // ks is in volts. Original value: 1
    // kv is volts * seconds / radians. Original value: 0.5
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.1);
    private IEncoder m_driveEncoder;
    private IEncoder m_steerEncoder;

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
    }

    // --------------------------------------------------------------------------

    Translation2d getTranslationFromCenter() {
        return m_location;
    }

    // --------------------------------------------------------------------------

    SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_location.getDistance(new Translation2d()), m_location.getAngle());
    }

    // --------------------------------------------------------------------------

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_steerEncoder.getDistance()));
    }

    // --------------------------------------------------------------------------

    Rotation2d lastAngle = Rotation2d.fromDegrees(0);

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
                new Rotation2d(m_steerEncoder.getDistance()));

        // ----------------------------------------------------------------
        // Prevent jitter by checking if the drive speed is less than 10%.
        //
        if (Math.abs(state.speedMetersPerSecond) <= (Drivetrain.kMaxSpeed * 0.01)) {
            state.angle = lastAngle;
        } else {
            lastAngle = state.angle;
        }

        // Calculate the drive output from the drive PID controller.
        // This applies PID control, but also effectively changes m/s into % output
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(),
                state.speedMetersPerSecond);

        // Need to find what the max revs and gear ratio result in max speed

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // From
        // https://github.com/msdifede/FRCSwerve2022/blob/ed5e07e12625b80e12fc02ac8e514a1bc530df18/src/main/java/frc/robot/SwerveModule.java
        // double angle = (Math.abs(desiredState.speedMetersPerSecond) <=
        // (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle
        // : desiredState.angle.getDegrees(); // Prevent rotating module if speed is
        // less
        // then 1%. Prevents Jittering.

        // Calculate the turning motor output from the turning PID controller.
        double turnOutput = m_turningPIDController.calculate(m_steerEncoder.getDistance(),
                state.angle.getRadians());

        // Use the desired velocity (rad/s) and run through a feedfoward controller,
        // which basically increases the velocity by 50%
        turnOutput = turnOutput +
                m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.set(driveOutput + driveFeedforward);

        m_turningMotor.set(-turnOutput);
    }

    // --------------------------------------------------------------------------

    public void setDesiredTurn(Rotation2d turn) {
        final Rotation2d encoderRotation = new Rotation2d(m_steerEncoder.getDistance());

        final double turnOutput = m_turningPIDController.calculate(m_steerEncoder.getDistance(), turn.getRadians());

        System.out.println(String.format("Encoder/Desired/Output: %6.0f, %6.0f, %5.2f",
                encoderRotation.getDegrees(), turn.getDegrees(), turnOutput));

        m_turningMotor.set(-turnOutput);
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
        // TODO Send a bunch of stuff to the shuffleboard
    }

}
