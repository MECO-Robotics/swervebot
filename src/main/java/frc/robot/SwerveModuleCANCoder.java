// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
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
public class SwerveModuleCANCoder extends SwerveModule implements Sendable {

    // --------------------------------------------------------------------------

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     */
    public SwerveModuleCANCoder(
        MotorController driveMotorController,
        MotorController steerMotorController,
        WPI_CANCoder driveEncoder,
        WPI_CANCoder steerEncoder,
        Translation2d location) {

            super(driveMotorController, steerMotorController, location);


        //
        // Setup the Turning Encoder
        //

        // m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
        // m_turningEncoder.setReverseDirection(true);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kSteerEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // --------------------------------------------------------------------------

    @Override
    public double getTurnDistance() {
        return 0.0;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        
    }

}
