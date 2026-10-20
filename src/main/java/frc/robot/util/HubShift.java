package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tracks whether our HUB is active (Game Manual 6.4.1). FUEL scored into an inactive HUB
 * is worth ZERO points, so the shooter should only feed while our HUB is active.
 *
 * How the game works (Table 6-2 / 6-3), using TELEOP time remaining:
 *   AUTO                       both HUBS active
 *   TRANSITION  2:20 - 2:10    both HUBS active
 *   SHIFT 1     2:10 - 1:45    alliance that WON AUTO (named by FMS game data) is INACTIVE
 *   SHIFT 2     1:45 - 1:20    statuses swap
 *   SHIFT 3     1:20 - 0:55    swap again
 *   SHIFT 4     0:55 - 0:30    swap again
 *   END GAME    0:30 - 0:00    both HUBS active
 *
 * The FMS sends the alliance that is inactive in SHIFT 1 as game data ('R' or 'B') at the
 * start of TELEOP. CHECK the exact format on the 2026 FRC Control System website.
 *
 * This FAILS OPEN: if game data or match time is unknown (practice, no FMS), we report
 * "active" so the robot never refuses to shoot because of missing data.
 */
public final class HubShift {
  private HubShift() {}

  /** TELEOP time-remaining values (s) where a SHIFT boundary happens, in match order. */
  private static final double[] BOUNDARIES = {130, 105, 80, 55, 30};

  /**
   * Pure version of the HUB rules, for unit tests and for {@link #isOurHubActive()}.
   *
   * @param weAreRed            true if we are the red alliance
   * @param redInactiveFirst    whether red is inactive in SHIFT 1 (empty = unknown)
   * @param isAuto              true during AUTO
   * @param teleopTimeRemaining TELEOP seconds remaining (negative = unknown)
   */
  public static boolean isHubActive(
      boolean weAreRed, Optional<Boolean> redInactiveFirst, boolean isAuto, double teleopTimeRemaining) {
    if (isAuto || teleopTimeRemaining < 0 || redInactiveFirst.isEmpty()) return true;
    double t = teleopTimeRemaining;
    if (t > BOUNDARIES[0] || t <= BOUNDARIES[4]) return true; // TRANSITION or END GAME

    int shift = t > BOUNDARIES[1] ? 1 : t > BOUNDARIES[2] ? 2 : t > BOUNDARIES[3] ? 3 : 4;
    boolean weAreInactiveFirst = weAreRed == redInactiveFirst.get();
    // The SHIFT-1-inactive alliance is inactive in SHIFTS 1 and 3, active in 2 and 4.
    boolean oddShift = shift % 2 == 1;
    return weAreInactiveFirst ? !oddShift : oddShift;
  }

  /**
   * Seconds until our HUB status next changes, or -1 if it won't change again (or unknown).
   * Same inputs as {@link #isHubActive}.
   */
  public static double secondsUntilChange(
      boolean weAreRed, Optional<Boolean> redInactiveFirst, boolean isAuto, double teleopTimeRemaining) {
    if (isAuto || teleopTimeRemaining < 0 || redInactiveFirst.isEmpty()) return -1;
    boolean now = isHubActive(weAreRed, redInactiveFirst, false, teleopTimeRemaining);
    for (double b : BOUNDARIES) {
      if (b < teleopTimeRemaining
          && isHubActive(weAreRed, redInactiveFirst, false, b - 0.01) != now) {
        return teleopTimeRemaining - b;
      }
    }
    return -1;
  }

  /** Reads the FMS game data: is red inactive in SHIFT 1? Empty if no/unknown data. */
  public static Optional<Boolean> redInactiveFirst() {
    String msg = DriverStation.getGameSpecificMessage();
    if (msg == null || msg.isEmpty()) return Optional.empty();
    switch (Character.toUpperCase(msg.charAt(0))) {
      case 'R': return Optional.of(true);
      case 'B': return Optional.of(false);
      default: return Optional.empty();
    }
  }

  private static boolean weAreRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /** Live check using the Driver Station: is our HUB active right now? */
  public static boolean isOurHubActive() {
    return isHubActive(weAreRed(), redInactiveFirst(), DriverStation.isAutonomous(), teleopTime());
  }

  /** Live: seconds until our HUB status changes, or -1. */
  public static double secondsUntilChange() {
    return secondsUntilChange(weAreRed(), redInactiveFirst(), DriverStation.isAutonomous(), teleopTime());
  }

  /** TELEOP time remaining, or -1 when not in TELEOP or unknown. */
  private static double teleopTime() {
    return DriverStation.isTeleop() ? DriverStation.getMatchTime() : -1;
  }

  /** Publishes HUB status for the drive team. Call every loop. */
  public static void publish() {
    SmartDashboard.putBoolean("Hub/Active", isOurHubActive());
    SmartDashboard.putNumber("Hub/SecondsToChange", secondsUntilChange());
    SmartDashboard.putBoolean("Hub/DataKnown", redInactiveFirst().isPresent());
  }
}
