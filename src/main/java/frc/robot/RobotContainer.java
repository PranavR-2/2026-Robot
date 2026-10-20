package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimbConstants;

import frc.robot.commands.Intake;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shooter;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.commands.AlignToTag;
import frc.robot.subsystems.ClimbMechanism;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AlignAndShoot;
import frc.robot.util.HubShift;

import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This is where everything gets wired together. Think of it as the robot's "control room"
 * where we:
 *   1. Create all subsystems (drivetrain, intake, shooter, climb)
 *   2. Bind controller buttons to commands ("when B is pressed, run the intake")
 *   3. Register named commands for PathPlanner autonomous routines
 *   4. Set up the autonomous mode chooser on the dashboard
 *
 * The pattern is: Subsystems own the hardware. Commands use subsystems to do things.
 * This class connects buttons to commands and commands to subsystems.
 *
 * WANT TO ADD A NEW BUTTON BINDING? Go to configureBindings().
 * WANT TO ADD A NEW SUBSYSTEM? Create the subsystem object here and pass it to commands.
 * WANT TO ADD A NEW AUTO ROUTINE? Register named commands below and create the .auto file
 * in PathPlanner.
 */
public class RobotContainer {

  // ========================
  // SUBSYSTEM INSTANTIATION
  // ========================
  // Each subsystem represents a physical mechanism on the robot.
  // We create them here so they exist for the entire match.

  /** The swerve drivetrain -- controls all 4 swerve modules and handles odometry/vision. */
  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();

  /** The intake mechanism -- single roller motor for sucking in or spitting out game pieces. */
  private final IntakeMechanism m_intakeMech = new IntakeMechanism(IntakeConstants.ioSparkPort);

  /** The shooter mechanism -- dual flywheel motors + kicker for launching game pieces. */
  private final ShooterMechanism m_shooterMech = new ShooterMechanism(ShooterConstants.shooterSparkPort1, ShooterConstants.shooterSparkPort2, ShooterConstants.kickerSparkPort);

  /** The climb mechanism -- single motor for lifting the robot onto the chain. */
  private final ClimbMechanism m_climbMech = new ClimbMechanism(ClimbConstants.climbSparkPort1);

  // ========================
  // CONTROLLER SETUP
  // ========================

  /**
   * Driver controller (port 0): Controls robot movement, shooting, and alignment.
   * Left stick = drive, Right stick = rotate, Triggers = speed reduction.
   */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * Operator controller (port 1): Controls mechanisms (intake, outtake, climb).
   * Separate from the driver so each person can focus on their role.
   */
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // ========================
  // DASHBOARD WIDGETS
  // ========================

  /** Dropdown on the dashboard that lets the drive team choose which auto routine to run. */
  SendableChooser<Command> autoChooser;

  /** Toggle button on the dashboard for enabling/disabling automatic alignment. */
  private GenericEntry autoAlignEntry;

  /**
   * Constructor -- runs once when the robot boots up.
   * Sets up the default drive command, binds buttons, registers auto commands,
   * and configures the dashboard.
   */
  /**
   * Every command name PathPlanner autos may use. A unit test (AutoPathRulesTest) checks
   * that each .auto file only uses these, so an auto can't silently skip a missing command.
   */
  public static final Set<String> NAMED_COMMANDS = Set.of(
      "Intake", "Outtake", "Shoot", "AutoShoot", "AlignToTag", "AlignAndShoot", "Climb");

  /**
   * Warns when the battery is low. Checked only while DISABLED (resting voltage) because
   * under swerve load any battery dips below 12V during a match.
   */
  private final Debouncer lowBatteryDebouncer = new Debouncer(2.0, DebounceType.kRising);
  private final Alert lowBatteryAlert = new Alert("Battery below 12.0V at rest -- swap before the next match", AlertType.kWarning);

  /** Warns when the FMS HUB-shift data is missing in TELEOP (HUB gating then assumes active). */
  private final Alert hubDataAlert = new Alert("No HUB shift data from FMS -- shooter is NOT gating on HUB status", AlertType.kWarning);

  public RobotContainer() {

    // Set the default command for the drivetrain: joystick driving.
    // A "default command" runs whenever no other command is using the drivetrain.
    // This means the robot is always drivable unless another command takes over.
    m_drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      m_drivetrain,
      this::driveForward,                       // Forward/backward
      this::driveStrafe,                        // Left/right strafe
      () -> m_driverController.getRightX(),     // Rotation
      () -> m_driverController.getLeftTriggerAxis() // Speed slider (pull trigger to slow down)
    ));

    // Configure the trigger bindings (buttons -> commands)
    configureBindings();

    // ========================
    // PATHPLANNER NAMED COMMANDS
    // ========================
    // Named commands allow PathPlanner autonomous routines to trigger robot actions.
    // In a .auto file, you can add a "NamedCommand" event with one of these names
    // and the corresponding command will run at that point in the path.
    Map<String, Command> namedCommands = Map.of(
        "Intake", new Intake(m_intakeMech),
        "Outtake", new Outtake(m_intakeMech),
        "Shoot", new Shooter(m_shooterMech),
        "AutoShoot", new AutoShooter(m_shooterMech, m_drivetrain),
        "AlignToTag", new AlignToTag(m_drivetrain, () -> 0.0, () -> 0.0),
        "AlignAndShoot", new AlignAndShoot(m_drivetrain, m_shooterMech).withTimeout(3.0),
        "Climb", new Climb(m_climbMech));
    namedCommands.forEach(NamedCommands::registerCommand);
    if (!namedCommands.keySet().equals(NAMED_COMMANDS)) {
      DriverStation.reportError("RobotContainer.NAMED_COMMANDS is out of sync with registered commands", false);
    }

    // Build the auto chooser dropdown. "MobilityAuto" is the default selection.
    // PathPlanner scans the deploy/pathplanner/autos/ folder for .auto files.
    autoChooser = AutoBuilder.buildAutoChooser("MobilityAuto");

    // ========================
    // SHUFFLEBOARD / DASHBOARD
    // ========================
    // Add widgets to the "Driver" tab so the drive team can see and control things.

    // Auto chooser dropdown -- lets the drive team pick which autonomous to run
    Shuffleboard.getTab("Driver")
        .add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(4, 0)
        .withSize(3, 1);

    // Auto alignment toggle -- a button on the dashboard to enable/disable auto-alignment
    autoAlignEntry = Shuffleboard.getTab("Driver")
        .add("Auto Alignment", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(4, 1)
        .withSize(3, 1)
        .getEntry();

    // Debug flags for development
    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
  }

  /**
   * Configures button-to-command mappings for both controllers.
   *
   * How button bindings work:
   *   - .whileTrue(command) = runs the command as long as the button is held down,
   *     stops when released
   *   - .onTrue(command) = runs the command once when the button is first pressed
   *   - .toggleOnTrue(command) = starts the command on first press, stops on second press
   *
   * DRIVER CONTROLLER (port 0):
   *   X button     = Enable field-relative driving
   *   A button     = Disable field-relative driving (robot-relative)
   *   Y button     = Reset NavX gyro heading (re-zeroes "forward")
   *   B button     = Run shooter at constant speed
   *   Left Bumper  = Aim at HUB + auto-shoot (distance-based speed; only feeds when HUB
   *                  active, in ALLIANCE ZONE, and aimed). Driver still translates.
   *   Right Bumper = Aim at HUB (auto-rotation)
   *   Right Trigger (hold) = Override the auto-shooter's HUB/zone/aim checks
   *
   * Both controllers rumble 3 s before our HUB turns on or off, and when END GAME starts.
   *
   * OPERATOR CONTROLLER (port 1):
   *   B button     = Run intake (suck in game piece)
   *   X button     = Run outtake (spit out game piece)
   *   Left Bumper  = Climb up
   *   Right Bumper = Climb down
   */
  private void configureBindings() {
    // --- DRIVER CONTROLLER BINDINGS ---

    // Field relativity controls: X = field-relative ON, A = field-relative OFF
    // These are one-shot actions with no subsystem requirement, so pressing them never
    // interrupts driving (the old whileTrue commands required the drivetrain).
    m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrain.setFieldRelativity(true)));
    m_driverController.a().onTrue(Commands.runOnce(() -> m_drivetrain.setFieldRelativity(false)));

    // Reset the NavX gyro heading -- makes the robot's current facing direction "forward"
    m_driverController.y().onTrue(Commands.runOnce(m_drivetrain::zeroHeading));

    // Shooter: B = constant speed, Left Bumper = auto-speed based on distance
    m_driverController.b().whileTrue(new Shooter(m_shooterMech));
    // Left Bumper = aim AND shoot in one button: turns toward the HUB while the driver keeps
    // translating, and feeds once aimed (AutoShooter won't feed until within the aim tolerance).
    m_driverController.leftBumper().whileTrue(Commands.parallel(
        new AlignToTag(m_drivetrain, this::driveForward, this::driveStrafe),
        new AutoShooter(m_shooterMech, m_drivetrain, m_driverController.rightTrigger())));

    // --- RUMBLE CUES ---
    // 3 s warning before our HUB turns on (get ready to fire) or off (stop and collect/defend).
    new Trigger(() -> {
      double t = HubShift.secondsUntilChange();
      return t > 0 && t <= 3.0;
    }).onTrue(rumble(0.5));

    // END GAME starts (0:30): both HUBS active, time to think about climbing.
    new Trigger(() -> DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= 30.0)
        .onTrue(rumble(1.0));

    // Align to AprilTag: auto-rotates to face the target while still allowing manual driving
    // Uses the SAME stick inputs as normal driving so the robot doesn't feel reversed while
    // aiming. (Previously these were negated vs. SwerveJoystickCmd -- one of them was wrong.)
    m_driverController.rightBumper().whileTrue(new AlignToTag(
        m_drivetrain, this::driveForward, this::driveStrafe));

    // --- OPERATOR CONTROLLER BINDINGS ---

    // Intake/outtake game pieces
    m_operatorController.b().whileTrue(new Intake(m_intakeMech));
    m_operatorController.x().whileTrue(new Outtake(m_intakeMech));

    // Climbing controls
    m_operatorController.leftBumper().whileTrue(new Climb(m_climbMech));
    m_operatorController.rightBumper().whileTrue(new ClimbDown(m_climbMech));
  }

  /**
   * Returns whether the auto-alignment toggle on the dashboard is enabled.
   * Can be checked by other parts of the code to conditionally enable alignment.
   *
   * @return true if auto-alignment is enabled on the dashboard
   */
  /** Driver translation inputs, shared by every command that lets the driver translate. */
  private double driveForward() {
    return m_driverController.getLeftY();
  }

  private double driveStrafe() {
    return m_driverController.getLeftX();
  }

  /** Rumbles both controllers for the given time. Requires no subsystem, so it never interrupts anything. */
  private Command rumble(double seconds) {
    return Commands.startEnd(
        () -> setRumble(1.0),
        () -> setRumble(0.0))
      .withTimeout(seconds);
  }

  private void setRumble(double value) {
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, value);
    m_operatorController.getHID().setRumble(RumbleType.kBothRumble, value);
  }

  /** Called every loop from Robot.robotPeriodic(): dashboard status and alerts. */
  public void periodic() {
    HubShift.publish();
    // Debounced so brief sags under hard acceleration don't flash the warning
    lowBatteryAlert.set(lowBatteryDebouncer.calculate(
        DriverStation.isDisabled() && RobotController.getBatteryVoltage() < 12.0));
    hubDataAlert.set(DriverStation.isTeleopEnabled() && DriverStation.isFMSAttached()
        && HubShift.redInactiveFirst().isEmpty());
  }

  public boolean isAutoAlignEnabled() {
    return autoAlignEntry.getBoolean(false);
  }

  /**
   * Returns the autonomous command selected on the dashboard dropdown.
   * Called by Robot.java when autonomous mode starts.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
