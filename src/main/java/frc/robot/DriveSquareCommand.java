// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * 
 */
public class DriveSquareCommand extends CommandBase {

    private final Drivetrain m_driveTrain;
    private long commandEndTime;
    private final long COMMAND_TIME = 5000;

    /**
     * Create a new command.
     *
     * @param
     */
    public DriveSquareCommand(Drivetrain driveSubsystem) {
        m_driveTrain = driveSubsystem;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {

        commandEndTime = System.currentTimeMillis() + COMMAND_TIME;
    }

    // ------------------------------------------------------------------------

    final static int k_driveDuration = 5000;       // Drive each side of the square for 5 seconds

    int m_side = 0;     // 0, 1, 2, 3 - 4 sides

    int m_currentTime = 0;

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {

        // First, see if we should keep driving or switch to the next side

        
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally, or
     * when it interrupted/canceled.
     *
     * <p>
     * Do not schedule commands here that share requirements with this command. Use
     * {@link
     * #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.drive(0, 0, 0, false);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will
     * call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > commandEndTime;
    }
}