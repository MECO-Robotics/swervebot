// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final XboxController m_controller = new XboxController(0);

    private final Drivetrain m_driveTrain = new Drivetrain();

    private final UserInterface m_userInterface = new UserInterface(m_driveTrain);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // --------------------------------------------------------------------------

    public Robot() {
        super(); // Pass in a number here to change the update rate
        CommandScheduler.getInstance().enable();
    }

    // --------------------------------------------------------------------------
    // Autonomous Mode

    @Override
    public void autonomousInit() {
        // If we're coming from disabled, then the command scheduler is disable.
        // Re-enable it.
        CommandScheduler.getInstance().enable();

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousPeriodic() {

    }

    // --------------------------------------------------------------------------
    // Test Mode

    // Class variables used just for Test mode
    double drive = 0;
    double turn = 0;
    int module;
    long pressTimer = 0;

    @Override
    public void testInit() {

        // If we're coming from disabled, then the command scheduler is disable.
        // Re-enable it.
        CommandScheduler.getInstance().enable();

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

        // final double kTurnIncrement = .05;

        // if (pressTimer > 0) {
        // if (System.currentTimeMillis() < (pressTimer + 500)) {
        // System.out.println("Ignoring input ...");
        // return;
        // } else {
        // System.out.println("resuming input ...");
        // pressTimer = 0;
        // }
        // }

        // // controls for selecting which swerve module you are testing
        // if (m_controller.getYButton()) {
        // drive = 0;
        // turn = 0;
        // m_driveTrain.control(module, drive, turn);
        // module = 0;
        // }

        // if (m_controller.getBButton()) {
        // drive = 0;
        // turn = 0;
        // m_driveTrain.control(module, drive, turn);
        // module = 1;
        // }

        // if (m_controller.getAButton()) {
        // drive = 0;
        // turn = 0;
        // m_driveTrain.control(module, drive, turn);
        // module = 2;
        // }

        // if (m_controller.getXButton()) {
        // drive = 0;
        // turn = 0;
        // m_driveTrain.control(module, drive, turn);
        // module = 3;
        // }

        // // controls for increasing speed and turn

        // if (m_controller.getPOV() == 0) {
        // drive = drive + 0.1;
        // pressTimer = System.currentTimeMillis();
        // }

        // if (m_controller.getPOV() == 90) {
        // turn = turn + kTurnIncrement;
        // pressTimer = System.currentTimeMillis();
        // }

        // if (m_controller.getPOV() == 180) {
        // drive = drive - 0.1;
        // pressTimer = System.currentTimeMillis();
        // }

        // if (m_controller.getPOV() == 270) {
        // turn = turn - kTurnIncrement;
        // pressTimer = System.currentTimeMillis();
        // }

        // // allows for control of the swerve modules
        // m_driveTrain.control(module, drive, turn);

        // // turn a module to a specific number of degrees
        // // m_driveTrain.turnModule(module, turn);

        // //
        // // TEST PATTERN COMMAND
        // //

        // if (m_controller.getRightBumperPressed()) {
        // System.out.println("Turning RIGHT");
        // pressTimer = System.currentTimeMillis();
        // m_allAngle += 180;
        // }

        // if (m_controller.getLeftBumperPressed()) {
        // System.out.println("Turning LEFT");
        // pressTimer = System.currentTimeMillis();
        // m_allAngle -= 180;
        // }

        // m_driveTrain.turnModule(0, m_allAngle);
        // m_driveTrain.turnModule(1, m_allAngle);
        // m_driveTrain.turnModule(2, m_allAngle);
        // m_driveTrain.turnModule(3, m_allAngle);

    }

    double m_allAngle = 0;

    @Override
    public void testExit() {

    }

    // --------------------------------------------------------------------------
    // Teleop

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void teleopPeriodic() {

        driveWithJoystick(false);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * SwerveModule.kDriveMaxForwardSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * SwerveModule.kDriveMaxForwardSpeed;
        ;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                * SwerveModule.kSteerMaxAngularVelocity;

        m_driveTrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    // --------------------------------------------------------------------------
    // Robot - called regardless of mode

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {

        // Regardless of the mode (Autonomous, Teleop, Test, Practice, Disabled) we need
        // to run periodic() on all the registered subsystems and scheduled commands
        CommandScheduler.getInstance().run();

        m_userInterface.periodic();
    }

    // --------------------------------------------------------------------------
    // Disabled

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
    }

    @Override
    public void disabledPeriodic() {
    }

    // --------------------------------------------------------------------------
    // Simulation

    double getSimValue(){
        return (int)Timer.getFPGATimestamp() % 50;
    }
    @Override
    public void simulationInit() {
        Shuffleboard.getTab("SIM DEMO").addDouble("X", this::getSimValue);
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
