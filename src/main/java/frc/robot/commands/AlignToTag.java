package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Automatically turns the robot to point the shooter at the CENTER of our HUB while the
 * driver keeps manual control of translation (forward/backward and strafing).
 *
 * How it works:
 *   - We know where the robot is on the field (odometry + Limelight) and where our HUB is
 *     (official AprilTag layout, see FieldConstants).
 *   - The code spins the robot so the shooter faces the HUB center.
 *   - This keeps working when the camera loses sight of the tag, and it doesn't lock onto
 *     the HUB's offset AprilTags (each HUB face has one centered and one offset tag).
 *
 * Used with AutoShooter, which only feeds once this has the robot aimed.
 *
 * Bound to: Driver controller Right Bumper (whileTrue -- aligns while held).
 *
 * WANT TO CHANGE alignment speed? Adjust ShooterConstants.kAimP.
 * Shooter fires out the back? Set ShooterConstants.kShooterHeadingOffsetDeg.
 */
public class AlignToTag extends Command {
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY;
    private final SlewRateLimiter xLimiter, yLimiter;

    /** Controls how the robot spins to face the HUB. Input is the aim error in radians. */
    private final PIDController turnPID;

    private Translation2d hubCenter = FieldConstants.kBlueHubCenter;

    /**
     * Creates a new AlignToTag command.
     *
     * @param drivetrain the swerve drivetrain subsystem
     * @param forwardX   supplier for forward/backward joystick input (manual control)
     * @param forwardY   supplier for left/right strafe joystick input (manual control)
     */
    public AlignToTag(
        SwerveSubsystem drivetrain,
        DoubleSupplier forwardX,
        DoubleSupplier forwardY
    ) {
        this.drivetrain = drivetrain;
        this.forwardX = forwardX;
        this.forwardY = forwardY;

        // Smooth out the manual driving inputs so movement isn't jerky
        this.xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration / DrivetrainConstants.maxVelocity);
        this.yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration / DrivetrainConstants.maxVelocity);

        // Controls how the robot spins to face the HUB. Higher P = spins faster.
        this.turnPID = new PIDController(ShooterConstants.kAimP, 0, 0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        turnPID.reset();
        var alliance = DriverStation.getAlliance();
        hubCenter = FieldConstants.hubCenter(
            alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red);
    }

    /** Stop spinning/driving when alignment ends so the wheels aren't left at the last command. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    /**
     * Runs every 20ms: processes manual driving inputs for translation and turns the
     * shooter toward the HUB center.
     */
    @Override
    public void execute() {
        // 1. Process manual joystick inputs for translation (forward/backward, left/right)
        double xSpeed = applyDeadbandAndLimiter(forwardX.getAsDouble(), OperatorConstants.xDeadband, xLimiter);
        double ySpeed = applyDeadbandAndLimiter(forwardY.getAsDouble(), OperatorConstants.yDeadband, yLimiter);

        // 2. Turn toward the HUB. Positive error = HUB is counter-clockwise from where the
        //    shooter points, which needs a positive (counter-clockwise) spin in WPILib.
        double aimError = AutoShooter.aimErrorRadians(drivetrain.getPose(), hubCenter);
        double rotSpeed = -turnPID.calculate(aimError, 0);

        // Clamp to [-1, 1] range and scale to max angular velocity
        rotSpeed = Math.max(-1, Math.min(1, rotSpeed));
        rotSpeed *= DrivetrainConstants.maxAngularVelocity;

        // 3. Drive the robot: manual translation + auto-rotation, always field-relative
        drivetrain.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            true // Always field relative for ease of use during alignment
        );
    }

    /**
     * Ignores tiny stick movements (deadband) and smooths out acceleration.
     */
    private double applyDeadbandAndLimiter(double value, double deadband, SlewRateLimiter limiter) {
        return limiter.calculate(SwerveJoystickCmd.shapeInput(value, deadband)) * DrivetrainConstants.maxVelocity;
    }
}
