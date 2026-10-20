package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/** Checks field geometry and the ALLIANCE ZONE check used for rule G407. */
class FieldConstantsTest {
  private static final double HALF = 0.45;
  private static final double ZONE = FieldConstants.kAllianceZoneDepth;
  private static final double LEN = FieldConstants.kFieldLength;
  private static final double MID_Y = FieldConstants.kFieldWidth / 2;

  @Test
  void hubTagListsAreOnTheRightSides() {
    // If this fails, ShooterConstants.kRedHubTags / kBlueHubTags are swapped
    assertTrue(FieldConstants.kRedHubCenter.getX() > LEN / 2, "red HUB should be on the +X (red) side");
    assertTrue(FieldConstants.kBlueHubCenter.getX() < LEN / 2, "blue HUB should be on the -X (blue) side");
  }

  @Test
  void blueAllianceZone() {
    assertTrue(FieldConstants.inAllianceZone(new Translation2d(2.0, MID_Y), false, HALF, 0.0));
    // Robot center just past the line but rear BUMPER still inside: allowed
    assertTrue(FieldConstants.inAllianceZone(new Translation2d(ZONE + HALF - 0.01, MID_Y), false, HALF, 0.0));
    // BUMPERS fully out of the zone: not allowed
    assertFalse(FieldConstants.inAllianceZone(new Translation2d(ZONE + HALF + 0.01, MID_Y), false, HALF, 0.0));
    // Blue robot on the red side: not allowed
    assertFalse(FieldConstants.inAllianceZone(new Translation2d(LEN - 2.0, MID_Y), false, HALF, 0.0));
  }

  @Test
  void redAllianceZoneMirrorsBlue() {
    assertTrue(FieldConstants.inAllianceZone(new Translation2d(LEN - 2.0, MID_Y), true, HALF, 0.0));
    assertFalse(FieldConstants.inAllianceZone(new Translation2d(2.0, MID_Y), true, HALF, 0.0));
    assertFalse(FieldConstants.inAllianceZone(new Translation2d(LEN - ZONE - HALF - 0.01, MID_Y), true, HALF, 0.0));
  }

  @Test
  void marginShrinksTheAllowedArea() {
    Translation2d edge = new Translation2d(ZONE + HALF - 0.05, MID_Y);
    assertTrue(FieldConstants.inAllianceZone(edge, false, HALF, 0.0));
    assertFalse(FieldConstants.inAllianceZone(edge, false, HALF, 0.10));
  }
}
