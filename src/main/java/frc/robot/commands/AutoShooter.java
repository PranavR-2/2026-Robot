package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.HubShift;

/**
 * Distance-based auto-speed shooter -- automatically adjusts flywheel speed based on
 * how far the robot is from our HUB.
 *
 * Distance comes from the robot's field position (odometry + Limelight), measured to the
 * CENTER of our HUB. That keeps working when the camera briefly loses the tag, and it
 * isn't fooled by the HUB's offset AprilTags.
 *
 * The flywheels always spin up so we're ready to fire, but game pieces are only fed when
 * EVERY one of these is true (so we never waste FUEL or take a foul):
 *   1. Our HUB is active (Game Manual 6.4.1 -- FUEL in an inactive HUB scores 0 points)
 *   2. Our BUMPERS are in our ALLIANCE ZONE (rule G407 -- otherwise a MAJOR FOUL)
 *   3. The robot is pointed at the HUB (within ShooterConstants.kAimToleranceDeg)
 *   4. The flywheels are up to speed (checked inside ShooterMechanism)
 *
 * The driver can hold the override button (right trigger) to skip checks 1-3, in case the
 * robot's position estimate or the FMS data is wrong.
 *
 * Bound to: Driver controller Left Bumper (whileTrue -- runs while held).
 *
 * WANT TO TUNE? Update ShooterConstants.kDistanceToSpeedMap. Distances are from the robot
 * center to the HUB center (shown on the dashboard as AutoShooter/Distance).
 */
public class AutoShooter extends Command {
    private final ShooterMechanism m_shooter;
    private final SwerveSubsystem m_drive;
    private final BooleanSupplier m_override;

    private boolean isRed = false;
    private Translation2d hubCenter = FieldConstants.kBlueHubCenter;

    /**
     * @param override held = feed even if the HUB/zone/aim checks fail (driver's judgment)
     */
    public AutoShooter(ShooterMechanism shooter, SwerveSubsystem drive, BooleanSupplier override) {
        m_shooter = shooter;
        m_drive = drive;
        m_override = override;
        addRequirements(m_shooter);
    }

    /** No override (used in AUTO, where nobody can press buttons anyway). */
    public AutoShooter(ShooterMechanism shooter, SwerveSubsystem drive) {
        this(shooter, drive, () -> false);
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        hubCenter = FieldConstants.hubCenter(isRed);
    }

    /** Runs every 20ms: picks a speed from distance, spins up, and feeds only when it's a good shot. */
    @Override
    public void execute() {
        Pose2d pose = m_drive.getPose();
        double distance = pose.getTranslation().getDistance(hubCenter);
        double speed = ShooterConstants.kDistanceToSpeedMap.get(distance);

        boolean hubActive = HubShift.isOurHubActive();
        boolean inZone = FieldConstants.inAllianceZone(
            pose.getTranslation(), isRed, DrivetrainConstants.kRobotHalfLength, ShooterConstants.kZoneMargin);
        boolean aimed = Math.abs(aimErrorRadians(pose, hubCenter))
            <= Math.toRadians(ShooterConstants.kAimToleranceDeg);
        boolean override = m_override.getAsBoolean();

        m_shooter.run(speed, speed, override || (hubActive && inZone && aimed));

        SmartDashboard.putNumber("AutoShooter/Distance", distance);
        SmartDashboard.putNumber("AutoShooter/SetSpeed", speed);
        SmartDashboard.putBoolean("AutoShooter/InZone", inZone);
        SmartDashboard.putBoolean("AutoShooter/Aimed", aimed);
        SmartDashboard.putBoolean("AutoShooter/Override", override);
    }

    /**
     * How far (radians) the shooter is from pointing at the target. Shared with AlignToTag
     * so "aimed" means the same thing to both.
     */
    public static double aimErrorRadians(Pose2d pose, Translation2d target) {
        Rotation2d wanted = FieldConstants.headingTo(pose.getTranslation(), target);
        Rotation2d shooterFacing = pose.getRotation()
            .plus(Rotation2d.fromDegrees(ShooterConstants.kShooterHeadingOffsetDeg));
        return wanted.minus(shooterFacing).getRadians();
    }

    /** When the command ends, stop all shooter motors. */
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopIOSpark();
    }
}
