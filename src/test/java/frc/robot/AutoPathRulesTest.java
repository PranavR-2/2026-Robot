package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.Constants.DrivetrainConstants;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Checks every PathPlanner auto against game rules on every build, so a path edit can't
 * sneak a foul into a match.
 *
 * Paths are drawn from the BLUE side (PathPlanner mirrors them for red), so "our side of
 * the CENTER LINE" is x < fieldLength / 2.
 */
class AutoPathRulesTest {
  private static final File PP_DIR = new File("src/main/deploy/pathplanner");
  private static final ObjectMapper JSON = new ObjectMapper();
  private static final double CENTER_LINE = FieldConstants.kFieldLength / 2.0;
  private static final double HALF = DrivetrainConstants.kRobotHalfLength;

  /**
   * G403 (MAJOR FOUL): in AUTO a ROBOT may not COMPLETELY cross the CENTER LINE, i.e. its
   * rear BUMPER may not pass it. Checked on anchors and Bezier control points (the curve
   * always stays inside its control points).
   */
  @Test
  void noPathCompletelyCrossesCenterLine() throws IOException {
    List<String> problems = new ArrayList<>();
    for (File path : files("paths", ".path")) {
      for (JsonNode wp : JSON.readTree(path).get("waypoints")) {
        for (String key : new String[] {"anchor", "prevControl", "nextControl"}) {
          JsonNode p = wp.get(key);
          if (p != null && !p.isNull() && p.get("x").asDouble() - HALF >= CENTER_LINE) {
            problems.add(path.getName() + " " + key + " x=" + p.get("x").asDouble());
          }
        }
      }
    }
    assertTrue(problems.isEmpty(), "Paths that would fully cross the CENTER LINE (G403): " + problems);
  }

  /**
   * Stricter than G403: the FRONT bumper stays on our side at every waypoint, so we never
   * reach into the opponent's half and risk contact (G211-H).
   */
  @Test
  void waypointsKeepFrontBumperOnOurSide() throws IOException {
    List<String> problems = new ArrayList<>();
    for (File path : files("paths", ".path")) {
      for (JsonNode wp : JSON.readTree(path).get("waypoints")) {
        double x = wp.get("anchor").get("x").asDouble();
        if (x + HALF > CENTER_LINE) problems.add(path.getName() + " x=" + x);
      }
    }
    assertTrue(problems.isEmpty(), "Waypoints whose front bumper crosses the CENTER LINE: " + problems);
  }

  /** Every named command used in an auto must be registered, or that step silently does nothing. */
  @Test
  void autosOnlyUseRegisteredNamedCommands() throws IOException {
    List<String> problems = new ArrayList<>();
    for (File auto : files("autos", ".auto")) {
      collectUnregistered(JSON.readTree(auto).get("command"), auto.getName(), problems);
    }
    assertTrue(problems.isEmpty(), "Autos using unregistered named commands: " + problems);
  }

  /** Every path an auto references must exist. */
  @Test
  void autosOnlyUseExistingPaths() throws IOException {
    List<String> problems = new ArrayList<>();
    for (File auto : files("autos", ".auto")) {
      collectMissingPaths(JSON.readTree(auto).get("command"), auto.getName(), problems);
    }
    assertTrue(problems.isEmpty(), "Autos referencing missing paths: " + problems);
  }

  private static void collectUnregistered(JsonNode cmd, String auto, List<String> out) {
    String type = cmd.get("type").asText();
    JsonNode data = cmd.get("data");
    if (type.equals("named")) {
      String name = data.get("name").asText();
      if (!RobotContainer.NAMED_COMMANDS.contains(name)) out.add(auto + ": " + name);
    }
    if (data != null && data.has("commands")) {
      for (JsonNode child : data.get("commands")) collectUnregistered(child, auto, out);
    }
  }

  private static void collectMissingPaths(JsonNode cmd, String auto, List<String> out) {
    JsonNode data = cmd.get("data");
    if (cmd.get("type").asText().equals("path")) {
      String name = data.get("pathName").asText();
      if (!new File(PP_DIR, "paths/" + name + ".path").exists()) out.add(auto + ": " + name);
    }
    if (data != null && data.has("commands")) {
      for (JsonNode child : data.get("commands")) collectMissingPaths(child, auto, out);
    }
  }

  private static File[] files(String dir, String ext) {
    File[] found = new File(PP_DIR, dir).listFiles((d, name) -> name.endsWith(ext));
    if (found == null || found.length == 0) fail("No " + ext + " files in " + new File(PP_DIR, dir));
    return found;
  }
}
