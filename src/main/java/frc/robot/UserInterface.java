package frc.robot;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UserInterface extends SubsystemBase {

    private final Drivetrain m_drivetrain;

    public UserInterface(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        for (int i = 0; i < m_drivetrain.m_numModules; i++) {
            // Put the PID for throttle and steer

            ShuffleboardTab tab = Shuffleboard.getTab("MODULE " + i);
            // Add things to the tab from the module

            SwerveModule module = m_drivetrain.m_modules[i];

            // PID Controllers can be added as Sendables - fire and forget. We don't need to
            // update these. Changes to the parameters on the UI will update the controllers
            // directly
            tab.add("Drive PID", module.m_drivePIDController);
            tab.add("Steer PID", module.m_turningPIDController);
        }
    }

    @Override
    public void periodic() {

        for (int i = 0; i < m_drivetrain.m_numModules; i++) {
            // Put the PID for throttle and steer

            ShuffleboardTab tab = Shuffleboard.getTab("MODULE " + i);
            // Add things to the tab from the module

            SwerveModule module = m_drivetrain.m_modules[i];

        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
