// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule {

    private static final int kSteerEncoderResolution = 1660; // Originally 4096
    private static final int kDriveEncoderResolution = 1; // NOT USED!

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final IMotorControllerEnhanced m_driveMotor;
    private final IMotorController m_turningMotor;

    private final Encoder m_driveEncoder = null; // not using drive encoders (yet)
    private final Encoder m_turningEncoder;

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

    // --------------------------------------------------------------------------

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorCanId        PWM output for the drive motor.
     * @param turningMotorCanId      PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(
            int driveMotorCanId,
            int turningMotorCanId,
            int driveEncoderChannelA,
            int driveEncoderChannelB,
            int turningEncoderChannelA,
            int turningEncoderChannelB) {

        m_driveMotor = new TalonSRX(driveMotorCanId);
        m_turningMotor = new VictorSPX(turningMotorCanId);

        //
        // Setup the Turning Encoder
        //

        m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
        m_turningEncoder.setReverseDirection(true);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.setDistancePerPulse(2 * Math.PI / kSteerEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        //
        // Setup the Drive Encoder (not currently available)
        //

        // m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
        // kDriveEncoderResolution);
    }

    // --------------------------------------------------------------------------

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    // public SwerveModuleState getState() {
    // return new SwerveModuleState(m_driveEncoder.getRate(), new
    // Rotation2d(m_turningEncoder.get()));
    // }

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
                new Rotation2d(m_turningEncoder.getDistance()));

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
        // final double driveOutput =
        // m_drivePIDController.calculate(m_driveEncoder.getRate(),
        // state.speedMetersPerSecond);

        // For now, without drive encoders, just use the input desired speed as the
        // output speed
        // Need to find what the max revs and gear ratio result in max speed
        // The am-0255 CIM Motor has a max speed of 5310 RPM
        // Gear ratio on the Swerve & Steer is 6.67:1
        // Wheel diameter is 4" (0.1016 m)
        // Max speed = 5310 RPM X 1m/60sec X 1/6.67 X 0.1016*3.1415m/1 rev = 4.23 m/s
        // So, drive output of 1.0 equals a desired speed of 4.23m/s

        final double driveOutput = state.speedMetersPerSecond / 4.23;

        // For now, leaving out the feedforward since it relies on knowing the current
        // speed, which we don't have
        // final double driveFeedforward =
        // m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // From
        // https://github.com/msdifede/FRCSwerve2022/blob/ed5e07e12625b80e12fc02ac8e514a1bc530df18/src/main/java/frc/robot/SwerveModule.java
        // double angle = (Math.abs(desiredState.speedMetersPerSecond) <=
        // (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle :
        // desiredState.angle.getDegrees(); //Prevent rotating module if speed is less
        // then 1%. Prevents Jittering.

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(),
                state.angle.getRadians());

        // Use the desired velocity (rad/s) and run through a feedfoward controller,
        // which basically increases the velocity by 50%
        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        // m_driveMotor.set(ControlMode.PercentOutput, driveOutput + driveFeedforward);

        // for now, just use the linear conversion from m/s to percent output
        m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
        m_turningMotor.set(ControlMode.PercentOutput, turnOutput + turnFeedforward);
    }

    public void setDesiredTurn(Rotation2d turn) {
        final Rotation2d encoderRotation = new Rotation2d(m_turningEncoder.getDistance());

        final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), turn.getRadians());

        System.out.println(String.format("Encoder/Desired/Output: %6.0f, %6.0f, %5.2f",
                encoderRotation.getDegrees(), turn.getDegrees(), turnOutput));

        m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
    }
    // --------------------------------------------------------------------------

    /**
     * Direct control of the swerve module motors, regardless of current state
     * 
     * @param drive The input level -1.0 to 1.0 for the driver motor
     * @param turn  The input level -1.0 to 1.0 for the steer motor
     */
    public void rawInput(double drive, double turn) {
        m_driveMotor.set(ControlMode.PercentOutput, drive);
        m_turningMotor.set(ControlMode.PercentOutput, turn);
    }

    // --------------------------------------------------------------------------

    /**
     * Get the turning encoder
     */
    public Encoder getTurnEncoder() {
        return m_turningEncoder;
    }

}
