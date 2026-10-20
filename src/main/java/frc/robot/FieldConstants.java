package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

/**
 * Where things are on the 2026 REBUILT field, taken from WPILib's official AprilTag layout
 * so we never hand-type field dimensions (the 2026 field is a different size than 2025's).
 *
 * All positions are in WPILib "blue origin" coordinates: (0, 0) is the corner by the blue
 * ALLIANCE WALL, and +X points toward the red ALLIANCE WALL.
 *
 * NOTE: Welded and AndyMark fields are slightly different (Game Manual Table 5-1/5-2).
 * kDefaultField is the welded field. If competing on an AndyMark field, switch to the
 * AndyMark entry in AprilTagFields.
 */
public final class FieldConstants {
  private FieldConstants() {}

  /** Official AprilTag positions for this season's field. */
  public static final AprilTagFieldLayout kLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double kFieldLength = kLayout.getFieldLength();
  public static final double kFieldWidth = kLayout.getFieldWidth();

  /** ALLIANCE ZONE depth from the ALLIANCE WALL (Game Manual 5.3: 158.6in). */
  public static final double kAllianceZoneDepth = Units.inchesToMeters(158.6);

  /** Center of each HUB: the average position of that HUB's AprilTags (Game Manual 5.11). */
  public static final Translation2d kRedHubCenter = averageTagPosition(ShooterConstants.kRedHubTags);
  public static final Translation2d kBlueHubCenter = averageTagPosition(ShooterConstants.kBlueHubTags);

  /** Returns our HUB's center. */
  public static Translation2d hubCenter(boolean isRed) {
    return isRed ? kRedHubCenter : kBlueHubCenter;
  }

  /**
   * True if the robot's BUMPERS are at least partly inside our ALLIANCE ZONE, which is
   * required to launch FUEL into our HUB (rule G407, MAJOR FOUL).
   *
   * @param robot          robot center position on the field
   * @param isRed          true if we are on the red alliance
   * @param halfLength     distance from robot center to the outside of the BUMPERS (m)
   * @param margin         safety margin (m) so pose error doesn't cause a foul
   */
  public static boolean inAllianceZone(Translation2d robot, boolean isRed, double halfLength, double margin) {
    double reach = kAllianceZoneDepth + halfLength - margin;
    return isRed ? robot.getX() >= kFieldLength - reach : robot.getX() <= reach;
  }

  /** The field-relative heading that points from the robot toward the target. */
  public static Rotation2d headingTo(Translation2d robot, Translation2d target) {
    return target.minus(robot).getAngle();
  }

  private static Translation2d averageTagPosition(int[] ids) {
    Translation2d sum = new Translation2d();
    for (int id : ids) {
      sum = sum.plus(kLayout.getTagPose(id).orElseThrow().toPose2d().getTranslation());
    }
    return sum.div(ids.length);
  }
}
