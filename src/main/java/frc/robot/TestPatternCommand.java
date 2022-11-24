// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * 
 */
public class TestPatternCommand extends CommandBase {

    private final Drivetrain m_driveTrain;
    private long commandEndTime;
    private final long COMMAND_TIME = 5000;

    /**
     * Create a new command.
     *
     * @param
     */
    public TestPatternCommand(Drivetrain driveSubsystem) {
        m_driveTrain = driveSubsystem;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {

        // set the initial desired state on the drive train:
        // drive: 0
        // turn: 90 in radians
        m_driveTrain.drive(0, 0, Math.PI/2f, false);
        commandEndTime = System.currentTimeMillis() + COMMAND_TIME;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {

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