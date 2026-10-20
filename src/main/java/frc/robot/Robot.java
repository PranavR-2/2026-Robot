// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is the main robot class. WPILib calls different methods here depending
 * on what mode the robot is in (disabled, autonomous, teleop, test).
 *
 * TimedRobot means WPILib calls our "periodic" methods every 20 milliseconds
 * (50 times per second). That's the robot's heartbeat -- every 20ms it checks
 * for new joystick input, updates motors, reads sensors, etc.
 *
 * The flow is:
 *   1. Robot boots up -> constructor runs -> creates RobotContainer
 *   2. Every 20ms -> robotPeriodic() runs the CommandScheduler
 *   3. When a mode starts (auto, teleop, etc.) -> the corresponding Init method runs
 *   4. While in that mode -> the corresponding Periodic method runs every 20ms
 *
 * You usually don't need to add much code here. Most of the robot logic lives in
 * RobotContainer (button bindings), subsystems (hardware control), and commands
 * (actions the robot performs).
 */
public class Robot extends TimedRobot {
  /** Stores the autonomous command so we can cancel it when teleop starts. */
  private Command m_autonomousCommand;

  /**
   * RobotContainer is where all the subsystems, commands, and button bindings live.
   * We create it once when the robot boots up, and it handles everything from there.
   */
  private final RobotContainer m_robotContainer;

  /**
   * This runs when the robot first powers on. It creates the RobotContainer,
   * which in turn creates all subsystems, sets up button bindings, and configures
   * the autonomous chooser on the dashboard.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * Called every 20ms regardless of mode (disabled, autonomous, teleop, test).
   *
   * The most important thing here is running the CommandScheduler. The scheduler is
   * the engine of the command-based framework -- it:
   *   - Polls buttons to see if new commands should start
   *   - Runs the execute() method of all active commands
   *   - Removes commands that have finished
   *   - Calls periodic() on all registered subsystems
   *
   * Without this line, NOTHING in the command-based framework works.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /**
   * Called once when the robot enters Disabled mode. Disabled mode happens when
   * you disable the robot from the Driver Station. Motors won't move in this mode.
   */
  @Override
  public void disabledInit() {}

  /** Called every 20ms while disabled. Usually empty. */
  @Override
  public void disabledPeriodic() {}

  /**
   * Called once when autonomous mode starts. Gets the selected auto routine from
   * RobotContainer (chosen via the dashboard dropdown) and schedules it to run.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** Called every 20ms during autonomous. Usually empty because commands handle everything. */
  @Override
  public void autonomousPeriodic() {}

  /**
   * Called once when teleop (driver control) starts. Cancels the autonomous command
   * so it doesn't keep running during teleop -- you don't want auto and teleop
   * fighting over the same motors!
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Called every 20ms during teleop. Usually empty because commands handle everything. */
  @Override
  public void teleopPeriodic() {}

  /**
   * Called once when Test mode starts. Cancels everything so you start with a clean slate.
   * Test mode is useful for testing individual subsystems via the Shuffleboard/SmartDashboard.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called every 20ms during Test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when simulation mode starts. Only runs in the simulator, not on the real robot. */
  @Override
  public void simulationInit() {}

  /** Called every 20ms during simulation. Use this for simulation-specific updates. */
  @Override
  public void simulationPeriodic() {}
}
