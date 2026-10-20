package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Checks HubShift against Game Manual Table 6-2 / 6-3. */
class HubShiftTest {
  private static final Optional<Boolean> RED_WON_AUTO = Optional.of(true); // red inactive in SHIFT 1
  private static final Optional<Boolean> BLUE_WON_AUTO = Optional.of(false);
  private static final Optional<Boolean> UNKNOWN = Optional.empty();

  // One sample time inside each TELEOP timeframe (TELEOP seconds remaining)
  private static final double TRANSITION = 135, SHIFT1 = 120, SHIFT2 = 95, SHIFT3 = 70, SHIFT4 = 40, ENDGAME = 15;

  private static boolean active(boolean weAreRed, Optional<Boolean> data, double t) {
    return HubShift.isHubActive(weAreRed, data, false, t);
  }

  @Test
  void redScoresMoreInAuto() {
    // Table 6-3, left half: RED inactive in SHIFT 1 and 3, BLUE the opposite
    boolean[] red = {active(true, RED_WON_AUTO, TRANSITION), active(true, RED_WON_AUTO, SHIFT1),
        active(true, RED_WON_AUTO, SHIFT2), active(true, RED_WON_AUTO, SHIFT3),
        active(true, RED_WON_AUTO, SHIFT4), active(true, RED_WON_AUTO, ENDGAME)};
    boolean[] blue = {active(false, RED_WON_AUTO, TRANSITION), active(false, RED_WON_AUTO, SHIFT1),
        active(false, RED_WON_AUTO, SHIFT2), active(false, RED_WON_AUTO, SHIFT3),
        active(false, RED_WON_AUTO, SHIFT4), active(false, RED_WON_AUTO, ENDGAME)};
    assertArrayEquals(new boolean[] {true, false, true, false, true, true}, red);
    assertArrayEquals(new boolean[] {true, true, false, true, false, true}, blue);
  }

  @Test
  void blueScoresMoreInAuto() {
    assertEquals(true, active(true, BLUE_WON_AUTO, SHIFT1));
    assertEquals(false, active(true, BLUE_WON_AUTO, SHIFT2));
    assertEquals(false, active(false, BLUE_WON_AUTO, SHIFT1));
    assertEquals(true, active(false, BLUE_WON_AUTO, SHIFT2));
  }

  @Test
  void shiftBoundariesMatchTimerValues() {
    // SHIFT 1 starts at 2:10, END GAME at 0:30
    assertEquals(true, active(true, RED_WON_AUTO, 130.01)); // still TRANSITION
    assertEquals(false, active(true, RED_WON_AUTO, 130.0)); // SHIFT 1
    assertEquals(true, active(true, RED_WON_AUTO, 105.0)); // SHIFT 2
    assertEquals(true, active(false, RED_WON_AUTO, 30.0)); // END GAME
  }

  @Test
  void autoIsAlwaysActive() {
    assertTrue(HubShift.isHubActive(true, RED_WON_AUTO, true, SHIFT1));
  }

  @Test
  void failsOpenWhenDataOrTimeUnknown() {
    assertTrue(active(true, UNKNOWN, SHIFT1));
    assertTrue(active(true, RED_WON_AUTO, -1));
    assertEquals(-1, HubShift.secondsUntilChange(true, UNKNOWN, false, SHIFT1));
  }

  @Test
  void secondsUntilChange() {
    // Red won AUTO: red goes inactive at 130 (2:10), active again at 105
    assertEquals(5.0, HubShift.secondsUntilChange(true, RED_WON_AUTO, false, 135), 1e-9);
    assertEquals(15.0, HubShift.secondsUntilChange(true, RED_WON_AUTO, false, 120), 1e-9);
    // Blue stays active through TRANSITION and SHIFT 1; next change is at 105
    assertEquals(30.0, HubShift.secondsUntilChange(false, RED_WON_AUTO, false, 135), 1e-9);
    // Red in SHIFT 4 (active) stays active into END GAME: no more changes
    assertEquals(-1, HubShift.secondsUntilChange(true, RED_WON_AUTO, false, 40));
  }
}
