// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * All robot-wide numbers live here. Think of this as the robot's "settings" file.
 *
 * We keep everything in one place so if you need to change a motor ID, PID value,
 * or speed limit, you only change it here -- not scattered across 10 different files.
 *
 * Each inner class groups related constants together (controller stuff, intake stuff,
 * drivetrain stuff, etc.).
 *
 * THIS IS THE FIRST FILE TO LOOK AT when you want to change robot behavior.
 * Most tweaks (speed, sensitivity, motor IDs, PID tuning) happen here.
 *
 * WANT TO CHANGE robot speed? Look in DrivetrainConstants.
 * WANT TO CHANGE controller sensitivity? Look in OperatorConstants.
 * WANT TO CHANGE a motor port? Look in the appropriate mechanism class (IntakeConstants, etc.).
 * WANT TO CHANGE shooter behavior? Look in ShooterConstants.
 */
public final class Constants {

  /*
   * ========================================
   * QUICK REFERENCE: ALL CAN IDs ON THE ROBOT
   * ========================================
   *
   * Device                          CAN ID    Section
   * -----------------------------   ------    -------------------
   * Front Left Drive Motor          4         DrivetrainConstants
   * Front Left Steering Motor       7         DrivetrainConstants
   * Front Left CANcoder             14        DrivetrainConstants
   * Front Right Drive Motor         5         DrivetrainConstants
   * Front Right Steering Motor      3         DrivetrainConstants
   * Front Right CANcoder            13        DrivetrainConstants
   * Back Left Drive Motor           6         DrivetrainConstants
   * Back Left Steering Motor        1         DrivetrainConstants
   * Back Left CANcoder              12        DrivetrainConstants
   * Back Right Drive Motor          8         DrivetrainConstants
   * Back Right Steering Motor       2         DrivetrainConstants
   * Back Right CANcoder             11        DrivetrainConstants
   * Intake Motor                    40        IntakeConstants
   * Shooter Motor 1                 42        ShooterConstants
   * Shooter Motor 2                 43        ShooterConstants
   * Kicker Motor                    44        ShooterConstants
   * Climb Motor                     45        ClimbConstants
   *
   * If you swap a motor controller, update the CAN ID here AND flash the
   * new ID to the controller using REV Hardware Client or Phoenix Tuner.
   */

  /**
   * Settings related to the driver and operator Xbox controllers.
   *
   * "Ports" here are the USB port numbers assigned in the FRC Driver Station.
   * Port 0 is typically the driver controller, port 1 is the operator controller.
   *
   * "Deadbands" prevent tiny accidental joystick movements from moving the robot.
   * If the joystick value is smaller than the deadband, the code treats it as zero.
   * Think of it like a "dead zone" around the center of the joystick.
   *
   * WANT TO CHANGE controller sensitivity? Increase the deadband values to require
   * bigger joystick movements before the robot responds. Decrease them for more
   * sensitive control (but the robot might drift if set too low).
   */
  public static class OperatorConstants {

    /* ================ */
    /* CONTROLLER PORTS */
    /* ================ */

    /** USB port for the driver's Xbox controller (the person who drives the robot). */
    public static final int kDriverControllerPort = 0;

    /** USB port for the operator's Xbox controller (the person who controls mechanisms). */
    public static final int kOperatorControllerPort = 1;

    /* ========= */
    /* Deadbands */
    /* ========= */

    /**
     * Forward/backward deadband. Joystick values smaller than 0.1 (10%) are
     * treated as zero. Prevents the robot from creeping when the stick isn't
     * perfectly centered.
     */
    public static final double xDeadband = 0.1;

    /**
     * Left/right (strafe) deadband. Same idea as xDeadband but for sideways movement.
     */
    public static final double yDeadband = 0.1;

    /**
     * Rotation deadband. Set higher (0.3 = 30%) because small accidental rotations
     * are more disorienting than small translational movements. The robot won't spin
     * unless the driver deliberately pushes the right stick.
     */
    public static final double rDeadband = 0.3;
  }


  /**
   * Settings for the intake mechanism -- the rollers that suck game pieces into the robot.
   *
   * The intake uses a single SparkMax motor controller running a brushless (NEO) motor.
   * Positive speed = intake (suck in), negative speed = outtake (spit out).
   *
   * WANT TO CHANGE intake/outtake speed? Change intakeSpeed / outtakeSpeed below.
   * WANT TO CHANGE the motor's CAN ID? Change ioSparkPort (must match the actual hardware).
   */
  public static class IntakeConstants {

    /** CAN ID of the intake motor controller (SparkMax). Must match the ID flashed to the controller. */
    public static final int ioSparkPort = 40;

    /** Speed to run the intake rollers (0.0 to 1.0). 0.75 = 75% power. */
    public static final double intakeSpeed = 0.75;

    /**
     * Speed to run the outtake (negative = reverse direction).
     * -0.75 = 75% power in reverse, which spits the game piece out.
     */
    public static final double outtakeSpeed = -0.75;

    /**
     * Maximum current the motor is allowed to draw (in Amps).
     * This protects the motor from burning out if it stalls (e.g., game piece jammed).
     * 35A is a safe limit for NEO motors in most FRC applications.
     */
    public static final int currentLimit = 35;
  }

  /**
   * Settings for the climbing mechanism -- the motor that lifts the robot onto the chain
   * at the end of a match.
   *
   * Uses a single SparkMax motor controller.
   *
   * WANT TO CHANGE climb speed? The speed is set directly in the Climb/ClimbDown commands.
   * WANT TO CHANGE the motor's CAN ID? Change climbSparkPort1 below.
   */
  public static class ClimbConstants {

    /** CAN ID of the climb motor controller (SparkMax). */
    public static final int climbSparkPort1 = 45;

    /**
     * Maximum current the climb motor can draw (in Amps).
     * Climbing puts heavy load on the motor, so the current limit protects it.
     */
    public static final int currentLimit = 35;
  }


  /**
   * Settings for the shooter mechanism -- the dual-motor flywheel + kicker that launches
   * game pieces (FUEL) into the HUB.
   *
   * The shooter has THREE motors:
   *   - Two shooter motors (CAN IDs 42 & 43) that spin up a flywheel. Both run at the
   *     same speed to launch the game piece.
   *   - One kicker motor (CAN ID 44) that pushes the game piece into the spinning flywheel.
   *     Think of the kicker as a "feeder" -- the flywheel spins up first, then the kicker
   *     shoves the ball in.
   *
   * The distance-to-speed lookup table (kDistanceToSpeedMap) is the key to auto-shooting:
   * the robot uses the Limelight camera to measure how far away it is from the HUB,
   * then picks the right flywheel speed from this table. If the robot is at a distance
   * between two entries (like 3.0m, which is between 2.5m and 4.0m), the code
   * automatically calculates a speed in between the two closest entries.
   *
   * WANT TO CHANGE shooter speed? Change shooterSpeed for manual shooting, or update the
   * lookup table entries for auto-shooting.
   * WANT TO CHANGE motor CAN IDs? Update shooterSparkPort1, shooterSparkPort2, kickerSparkPort.
   */
  public static class ShooterConstants {

    /** CAN ID of the first shooter flywheel motor (SparkMax). */
    public static final int shooterSparkPort1 = 42;

    /** CAN ID of the second shooter flywheel motor (SparkMax). */
    public static final int shooterSparkPort2 = 43;

    /** CAN ID of the kicker/feeder motor (SparkMax). Pushes game pieces into the flywheel. */
    public static final int kickerSparkPort = 44;

    /** Maximum current any shooter motor can draw (in Amps). */
    public static final int currentLimit = 35;

    /**
     * Default shooter speed for manual (constant-speed) shooting.
     * 0.75 = 75% power. Used by the Shooter command when the driver presses B.
     */
    public static final double shooterSpeed = 0.75;

    /** NEO free speed (RPM). Shooter speeds (0-1) are fractions of this. */
    public static final double kFreeSpeedRPM = 5676.0;

    /** Flywheel P gain, duty-cycle per RPM of error. Feedforward supplies the bulk of the output. */
    public static final double kFlywheelP = 0.0001;

    /** Kicker starts feeding once both flywheels are within this fraction of target speed. */
    public static final double kReadyTolerance = 0.05;

    /** Kicker stops feeding if a flywheel drops beyond this fraction of target (ball dip hysteresis). */
    public static final double kNotReadyTolerance = 0.12;

    /** If flywheels still aren't at speed after this long, the kicker feeds anyway. */
    public static final double kMaxSpinUpSeconds = 1.5;

    /**
     * Safety margin (m) for the ALLIANCE ZONE check (rule G407). We only auto-feed when the
     * BUMPERS are at least this far inside the zone line, so pose error can't cause a foul.
     */
    public static final double kZoneMargin = 0.10;

    /** Auto-shooter only feeds when the robot is pointed within this many degrees of the HUB. */
    public static final double kAimToleranceDeg = 3.0;

    /**
     * Which way the shooter points relative to the robot's front, in degrees.
     * 0 = shooter fires out the front. 180 = out the back. CHECK THIS ON THE ROBOT.
     */
    public static final double kShooterHeadingOffsetDeg = 0.0;

    /**
     * Aim P gain: fraction of max spin speed per radian of heading error.
     * 2.29/rad equals the old 0.04/degree Limelight tx gain.
     */
    public static final double kAimP = 2.29;

    /** Nominal battery voltage that motor outputs are scaled against. */
    public static final double kNominalVoltage = 12.0;

    // --- FRC 2026 REBUILT CONSTANTS ---

    /**
     * AprilTag IDs on the RED alliance's HUB (the central scoring structure).
     * The auto-shooter checks if the camera sees one of these tags to make sure
     * we're aiming at the right goal (not the other alliance's).
     */
    public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

    /**
     * AprilTag IDs located on the BLUE alliance's HUB.
     * Same purpose as kRedHubTags, but for the other alliance.
     */
    public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};

    /**
     * LOOKUP TABLE: Distance (Meters) -> Shooter Speed % (0.0 to 1.0)
     *
     * Distance is from the ROBOT CENTER to the HUB CENTER (from the robot's field position),
     * shown on the dashboard as AutoShooter/Distance. From our ALLIANCE ZONE that's roughly
     * 1.1 m (touching the HUB) to 4.6 m (back at the zone line).
     *
     * This table tells the robot "if you're X meters away, shoot at Y% power."
     * If the robot is at a distance between two entries, it figures out the
     * right speed automatically. For example, at 1.75m (between the 1.0m and
     * 2.5m entries), it picks a speed halfway between 0.35 and 0.55.
     *
     * WANT TO TUNE? Stand at known distances on the field, shoot, and see if
     * it goes in. Adjust the speed values until it's accurate, then update
     * the table. Add more entries for better accuracy at more distances.
     *
     * MEASURE THESE ON THE FIELD! The values below are just starting guesses.
     */
    public static final InterpolatingDoubleTreeMap kDistanceToSpeedMap = new InterpolatingDoubleTreeMap();
    static {
        // Format: .put(Distance_Meters, Speed_Percent);

        kDistanceToSpeedMap.put(1.0, 0.35); // Close range (Fender shot -- right up against the HUB)
        kDistanceToSpeedMap.put(2.5, 0.55); // Mid range
        kDistanceToSpeedMap.put(4.0, 0.75); // Long range
        kDistanceToSpeedMap.put(6.0, 0.95); // Cross-field (maximum distance shot)
    }
  }

  /**
   * Settings for the Limelight vision camera.
   *
   * The Limelight is a smart camera that can spot AprilTags (special black-and-white
   * patterns) on the field. It talks to the robot over the network and tells us things
   * like "the target is 5 degrees to the left" and "you're looking at tag #3."
   *
   * WANT TO CHANGE which Limelight to use? If you renamed your Limelight in the web
   * interface, update kLimelightName to match.
   */
  public static class VisionConstants {

    /**
     * The name of our Limelight camera. This must match what's set in the Limelight's
     * settings page (you can open it in a browser). The default name is "limelight".
     */
    public static final String kLimelightName = "limelight";

    /** Reject vision poses from a single tag farther than this (m). */
    public static final double kMaxSingleTagDistance = 4.0;

    /** Reject vision poses when the average tag distance is farther than this (m). */
    public static final double kMaxTagDistance = 6.0;

    /** Reject vision while turning faster than this (deg/s); camera latency makes it unreliable. */
    public static final double kMaxVisionYawRateDegPerSec = 360.0;

    /** Vision x/y standard deviation = kXYStdBase * (1 + dist^2) / tagCount. Higher = trust vision less. */
    public static final double kXYStdBase = 0.1;

    /** Vision heading std dev with one tag. Huge so the pose estimator trusts the gyro for heading. */
    public static final double kThetaStd = 1.0e6;

    /** Vision heading std dev (rad) with 2+ tags: lets vision slowly fix a wrong heading. */
    public static final double kThetaStdMultiTag = 0.7;
  }

  /**
   * Settings for the swerve drivetrain -- the 4-wheel drive system where each wheel
   * can independently steer and drive.
   *
   * This is the most complex section because swerve drive has a LOT of configuration:
   *   - Physical measurements (wheelbase, track width, wheel size)
   *   - Conversion factors (translating motor rotations into real-world distances)
   *   - Speed limits (how fast the robot can go)
   *   - CAN IDs for all 12 motor controllers and 4 encoders (3 per module x 4 modules)
   *   - PID tuning values for both driving and steering
   *
   * WANT TO CHANGE max speed? Look at maxVelocity and kTeleDriveMaxSpeed.
   * WANT TO CHANGE PID tuning? Look at kPDrive, kPRotation, etc. at the bottom.
   * WANT TO SWAP a motor? Find the correct module (FL/FR/BL/BR) and update its CAN IDs.
   */
  public static class DrivetrainConstants {

    /* ==================== */
    /* PHYSICAL DIMENSIONS  */
    /* ==================== */

    /**
     * Distance between the centers of the front and back wheels (in meters).
     * 21 inches = 0.5334 meters. Measured center-to-center.
     * WANT TO CHANGE? Only if you physically move the wheels on the chassis.
     */
    public static final double wheelBase = Units.inchesToMeters(21);

    /**
     * Distance between the centers of the left and right wheels (in meters).
     * 21 inches = 0.5334 meters. Same as wheelBase for our square chassis.
     */
    public static final double trackWidth = Units.inchesToMeters(21);

    /**
     * Diameter of the drive wheels (in meters). 4-inch wheels are standard for swerve.
     * Used in conversion factor calculations.
     */
    public static final double wheelDiameter = Units.inchesToMeters(4.0);

    /**
     * Tells WPILib where each wheel is on the robot (how far from the center).
     * WPILib uses this to figure out how fast each wheel needs to spin and
     * which direction it needs to point for any given movement.
     *
     * Each Translation2d is an (x, y) position from the robot's center:
     *   - +X = toward the front of the robot
     *   - +Y = toward the left side of the robot
     *
     * The order here must match the order we use everywhere else in the code.
     */
    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front right (+,+)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // back right (+,-)
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // front left (-,+)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // back left (-,-)
    );

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */

    /**
     * Turns motor spin counts into real-world distance (meters).
     *
     * The motor spins many times for each wheel rotation (because of gears).
     * This number accounts for the gear ratio and wheel size so we can ask
     * "how far has the robot actually moved?" instead of "how many times
     * did the motor spin?"
     *
     * If you swap to different wheels or gears, you'll need to recalculate this.
     */
    public static final double drivePositionConversionFactor = 0.0472867872;

    /**
     * Turns motor RPM (spins per minute) into meters per second.
     * Same idea as above, but for speed instead of distance.
     */
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    /**
     * Turns motor spin counts into steering angle (radians).
     * Same idea as the drive conversion, but for the steering motor.
     */
    public static final double rotationPositionConversionFactor = 0.29321531433;

    /** Turns motor RPM into steering speed (radians per second). */
    public static final double rotationVelocityConversionFactor = rotationPositionConversionFactor / 60.0;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    /**
     * Top speed the robot can drive (in meters per second).
     * 4 m/s is about 9 mph. Setting this higher than the motors can actually
     * go won't make it faster. Lowering it caps the speed (good for practice).
     */
    public static final double maxVelocity = 4; // m/s

    /**
     * How quickly the robot can speed up (meters per second squared).
     * Higher = the robot gets to full speed faster but feels more jerky.
     * Lower = smoother acceleration but takes longer to get going.
     */
    public static final double maxAcceleration = 7; // m/s^2

    /**
     * How fast the robot can spin (in radians per second).
     * 2*PI = one full spin per second.
     */
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/s

    /** How quickly the robot can start or stop spinning. */
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/s^2

    /**
     * Maximum teleop driving speed (in meters per second).
     * This is intentionally lower than maxVelocity to give drivers more control.
     * 7.5/4.0 = 1.875 m/s (about 4.2 mph).
     *
     * WANT TO CHANGE? Increase for a faster robot in teleop, decrease for more control.
     */
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0; // meters/sec

    /**
     * Maximum teleop rotation speed (in radians per second).
     * 3 rad/s is about half a rotation per second -- fast enough to turn quickly
     * but not so fast that the driver loses control.
     */
    public static final double kTeleDriveMaxAngularSpeed = 3; // rad/sec

    
    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

    /*
     * Each swerve module has three devices on the CAN bus:
     *   1. Drive motor (SparkMax) -- spins the wheel forward/backward
     *   2. Rotation motor (SparkMax) -- steers the wheel left/right
     *   3. CANcoder -- absolute encoder that always knows the wheel's angle,
     *      even after a power cycle
     *
     * The "offset" is a calibration value. When the wheel is physically pointing
     * straight forward, the CANcoder might read some non-zero value. The offset
     * compensates for this so the code knows "straight" = 0 radians.
     *
     * WANT TO CALIBRATE? Point all wheels straight forward, read each CANcoder's
     * raw value, and put that value as the offset.
     */

    // Front Left Module
    /** CAN ID of the front-left drive motor (SparkMax). */
    public static final int kFLDrive = 4;
    /** CAN ID of the front-left rotation/steering motor (SparkMax). */
    public static final int kFLRotate = 7;
    /** CAN ID of the front-left CANcoder (absolute encoder). */
    public static final int kFLCanCoder = 14;
    /** Offset for the front-left CANcoder (in radians). Calibrate with wheels pointing forward. */
    public static final double kFLOffsetRad = 0.042969 * 2 * Math.PI;
    /** Whether the front-left drive motor is inverted. */
    public static final boolean fLIsInverted = true;

    // Front Right Module
    /** CAN ID of the front-right drive motor (SparkMax). */
    public static final int kFRDrive = 5;
    /** CAN ID of the front-right rotation/steering motor (SparkMax). */
    public static final int kFRRotate = 3;
    /** CAN ID of the front-right CANcoder (absolute encoder). */
    public static final int kFRCanCoder = 13;
    /** Offset for the front-right CANcoder (in radians). */
    public static final double kFROffsetRad = 0.028564 * 2 * Math.PI;
    /** Whether the front-right drive motor is inverted. */
    public static final boolean fRIsInverted = true;
    
    // Back Left Module
    /** CAN ID of the back-left drive motor (SparkMax). */
    public static final int kBLDrive = 6;
    /** CAN ID of the back-left rotation/steering motor (SparkMax). */
    public static final int kBLRotate = 1;
    /** CAN ID of the back-left CANcoder (absolute encoder). */
    public static final int kBLCanCoder = 12;
    /** Offset for the back-left CANcoder (in radians). */
    public static final double kBLOffsetRad = 0.027466 * 2 * Math.PI;
    /** Whether the back-left drive motor is inverted. */
    public static final boolean bLIsInverted = true;

    // Back Right Module
    /** CAN ID of the back-right drive motor (SparkMax). */
    public static final int kBRDrive = 8;
    /** CAN ID of the back-right rotation/steering motor (SparkMax). */
    public static final int kBRRotate = 2;
    /** CAN ID of the back-right CANcoder (absolute encoder). */
    public static final int kBRCanCoder = 11;
    /** Offset for the back-right CANcoder (in radians). */
    public static final double kBROffsetRad = 0.340698 * 2 * Math.PI;
    /** Whether the back-right drive motor is inverted. */
    public static final boolean bRIsInverted = true;
    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */

    /*
     * PID is how we make motors go to the right speed or angle smoothly.
     * Think of it like cruise control in a car:
     *
     * - P (Proportional): "I'm far from my target, so push hard. I'm close, so ease up."
     *   This is the most important one. If the wheel wobbles back and forth, P is too high.
     *   If it's slow to reach the target, P is too low.
     * - I (Integral): "I've been slightly off for a while, let me push a bit harder."
     *   We usually leave this at 0 -- not needed for our drivetrain.
     * - D (Derivative): "I'm getting close fast, better slow down so I don't overshoot."
     *   Helps stop wobbling. A little goes a long way.
     *
     * WANT TO TUNE? Start by only changing P. Increase P until the wheel starts
     * wobbling, then back off a little. Add a tiny D if it still wobbles.
     */

    /** Drive motor P value. Higher = reaches target speed faster but might overshoot. */
    public static final double kPDrive = .21;

    /**
     * Drive feedforward: duty cycle per m/s. 1 / (NEO free speed in m/s at the wheel).
     * Without this the P-only controller settles at roughly half the commanded speed.
     */
    public static final double kVDrive =
        1.0 / (ShooterConstants.kFreeSpeedRPM / 60.0 * drivePositionConversionFactor);

    /** Robot center to the outside of the BUMPERS (m). Matches PathPlanner robotLength 0.9 m. */
    public static final double kRobotHalfLength = 0.45;

    /** Current limits (Amps) for the swerve motors, to avoid brownouts. */
    public static final int driveCurrentLimit = 40;
    public static final int rotationCurrentLimit = 25;
    /** Drive motor I value. Leave at 0 -- we don't need it. */
    public static final double kIDrive = 0.0;
    /** Drive motor D value. Leave at 0 for driving. */
    public static final double kDDrive  = 0.0;

    /**
     * Steering motor P value. Controls how hard the wheel tries to turn to the
     * right angle. 0.57 works well for our modules.
     *
     * If the wheels wobble/jitter back and forth, make this smaller.
     * If the wheels are slow to turn, make this bigger.
     */
    public static final double kPRotation = 0.57;
    /** Steering motor I value. Leave at 0. */
    public static final double kIRotation = 0.0;
    /**
     * Steering motor D value. Helps stop the wheel from wobbling.
     * A tiny amount (0.005) is enough to smooth things out.
     */
    public static final double kDRotation = 0.005;

    /**
     * How close the wheel angle needs to be to the target before we say "close enough."
     * 0.01 radians is about half a degree.
     */
    public static final double kToleranceRotation = 0.01;

      
  }
}
