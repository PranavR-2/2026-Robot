package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * The swerve drivetrain -- this controls how the robot moves.
 *
 * A swerve drive has 4 wheels that can each point in any direction AND spin at any
 * speed. This lets the robot drive in any direction while also spinning. Think of it
 * like a shopping cart where you can control every wheel separately.
 *
 * This class manages:
 *   - Four SwerveModules (one per corner: FL, FR, BL, BR)
 *   - A NavX gyroscope (a sensor that knows which way the robot is facing)
 *   - A Pose Estimator (figures out WHERE the robot is on the field by combining
 *     wheel encoder data + gyro + Limelight readings)
 *   - Limelight (a vision camera that reads AprilTag targets for aiming and position)
 *   - PathPlanner (runs auto routines by driving the robot along paths)
 *
 * Every 20ms, the periodic() method updates the robot's position estimate using
 * both the wheel encoders AND the Limelight camera. Using both together gives us
 * a more accurate position than using either one alone.
 *
 * WANT TO CHANGE how the robot drives? Look at the drive() method.
 * WANT TO CHANGE camera stuff? Look at updateVisionPose().
 * WANT TO CHANGE auto path following? Look at the AutoBuilder.configure() call in the constructor.
 */
public class SwerveSubsystem extends SubsystemBase{

    /**
     * Are we driving field-relative (true) or robot-relative (false)?
     * Field-relative: "up" on the stick always goes toward the far wall.
     * Robot-relative: "up" on the stick goes wherever the robot's front is pointing.
     */
    public static boolean fieldRelativeStatus = true;

    // ========================
    // SWERVE MODULES
    // ========================
    // One module per corner. Each has a drive motor, rotation motor, and CANcoder.
    // The order (FL, FR, BL, BR) must match the order in SwerveDriveKinematics.

    private final SwerveModule frontLeft = new SwerveModule(
        DrivetrainConstants.kFLDrive,
        DrivetrainConstants.kFLRotate,
        DrivetrainConstants.kFLCanCoder,
        DrivetrainConstants.kFLOffsetRad,
        DrivetrainConstants.fLIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule frontRight = new SwerveModule(
        DrivetrainConstants.kFRDrive,
        DrivetrainConstants.kFRRotate,
        DrivetrainConstants.kFRCanCoder,
        DrivetrainConstants.kFROffsetRad,
        DrivetrainConstants.fRIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule backLeft = new SwerveModule(
        DrivetrainConstants.kBLDrive,
        DrivetrainConstants.kBLRotate,
        DrivetrainConstants.kBLCanCoder,
        DrivetrainConstants.kBLOffsetRad,
        DrivetrainConstants.bLIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule backRight = new SwerveModule(
        DrivetrainConstants.kBRDrive,
        DrivetrainConstants.kBRRotate,
        DrivetrainConstants.kBRCanCoder,
        DrivetrainConstants.kBROffsetRad,
        DrivetrainConstants.bRIsInverted,
        DrivetrainConstants.kPRotation
    );

    // ========================
    // SENSORS
    // ========================

    /** NavX gyroscope -- measures the robot's heading (which direction it's facing). */
    private final AHRS navX;

    /**
     * Tracks the robot's position on the field by combining wheel movement data,
     * gyro heading, and Limelight camera readings. More accurate than just
     * counting wheel rotations because the camera corrects for errors over time.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    /** Connection to the Limelight (our vision camera). We read all target data through this. */
    private final NetworkTable limelightTable;

    // Subscribers are created once; looking up entries by name every call is slower.
    private final DoubleSubscriber tvSub;
    private final DoubleSubscriber txSub;
    private final DoubleSubscriber tidSub;
    private final DoubleArraySubscriber botposeSub;
    private final DoubleArraySubscriber targetposeSub;

    /** Limelight heartbeat: increments every processed frame. Used to detect a dead camera. */
    private final DoubleSubscriber heartbeatSub;
    private double lastHeartbeat = -1;
    private double lastHeartbeatTime = 0;

    private final Alert navXAlert = new Alert("NavX disconnected: field-relative driving and odometry won't work", AlertType.kError);
    private final Alert limelightAlert = new Alert("Limelight not updating: no vision pose or auto-aim distance correction", AlertType.kWarning);

    /** Dashboard "Debug Swerve" toggle. Per-module telemetry is only published while it's on. */
    private final BooleanSubscriber debugSub = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard").getBooleanTopic("Debug Swerve").subscribe(false);

    /** Gyro is re-zeroed once, after it has had time to boot, while the robot is still disabled. */
    private boolean gyroZeroed = false;
    private final double constructedTime = Timer.getFPGATimestamp();


    /**
     * Constructor -- sets up all hardware and configures PathPlanner.
     *
     * The NavX reset is done in a separate thread with a 1-second delay because the
     * NavX gyro takes about a second to boot up after power-on. If we tried to reset
     * it immediately, it might not be ready yet.
     */
    public SwerveSubsystem(){
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        // The NavX needs about a second to boot after power-on, so it is zeroed from
        // periodic() once that time has passed (see zeroGyroAfterBoot).

        // Sync up the motor encoders with the CANcoders.
        // CANcoders always know the real wheel angle (even after a reboot), but the
        // motor's built-in encoder forgets when it loses power. This copies the
        // real angle into the motor encoder so they agree.
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        backLeft.initRotationOffset();
        backRight.initRotationOffset();

        // Reset drive encoders to zero. We start measuring distance from here.
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        // --- Limelight Setup ---
        // Connect to the Limelight (our vision camera) so we can read what it sees.
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);
        tvSub = limelightTable.getDoubleTopic("tv").subscribe(0.0);
        txSub = limelightTable.getDoubleTopic("tx").subscribe(0.0);
        tidSub = limelightTable.getDoubleTopic("tid").subscribe(-1.0);
        botposeSub = limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
        targetposeSub = limelightTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[0]);
        heartbeatSub = limelightTable.getDoubleTopic("hb").subscribe(-1.0);

        // AprilTag tracking doesn't need the Limelight's bright LEDs. Keep them off
        // (rule R203-L: high intensity lights only briefly while targeting).
        limelightTable.getEntry("ledMode").setNumber(1);

        // Add the Limelight camera stream to the Shuffleboard "Driver" tab
        // so the drive team can see what the camera sees.
        HttpCamera cameraStream = new HttpCamera("LimelightStream",
            "http://" + VisionConstants.kLimelightName + ".local:5800/stream.mjpg");

        Shuffleboard.getTab("Driver")
            .add("Limelight", cameraStream)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(4, 3);

        // --- Pose Estimator Initialization ---
        // Start with a known position (0,0) facing forward (0 degrees).
        // This gets updated every 20ms with encoder + vision data.
        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SwerveDriveKinematics,
            new Rotation2d(), // Initial gyro angle
            getModulePositions(),
            new Pose2d() // Initial position (0, 0, 0 degrees)
        );

        // --- PathPlanner Configuration ---
        // Load the robot configuration from PathPlanner's GUI settings file.
        // This includes robot mass, moment of inertia, and other physical properties
        // that PathPlanner needs to generate accurate paths.
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
        }

        // Configure PathPlanner's AutoBuilder so it can drive the robot during autonomous.
        // We tell it how to get the robot's pose, how to reset the pose, how to get
        // current speeds, and how to drive the robot.
        AutoBuilder.configure(
            this::getPose, // How to get the robot's current position
            this::resetOdometry, // How to reset the robot's position (used at start of auto)
            this::getRobotChassisSpeeds, // How to get current robot speeds (MUST be robot-relative)
            (speeds, feedforwards) -> driveRobotRelative(speeds), // How to drive the robot
            new PPHolonomicDriveController( // PID controller for path following
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID (driving to position)
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID (turning to angle)
            ),
            config, // Robot physical configuration
            () -> {
              // Should paths be mirrored for the red alliance?
              // Paths are designed from the blue side. If we're on red, flip them.
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // This subsystem -- PathPlanner will "require" it during auto
        );
    }


    /**
     * Stops all four swerve modules (both drive and rotation motors).
     * Called when we want the robot to come to a complete stop.
     */
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Returns where the robot thinks it is on the field (x position, y position, and heading).
     */
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns how fast the robot is currently moving and spinning.
     * Calculated from how fast each wheel is actually going.
     */
    public ChassisSpeeds getRobotChassisSpeeds(){
        return DrivetrainConstants.SwerveDriveKinematics.toChassisSpeeds(getStates());
    }

    /**
     * Drives the robot at the given speeds. Used by PathPlanner during auto mode.
     * Makes sure no wheel goes faster than the max speed.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        setModuleStates(moduleStates);
    }

    /**
     * Returns which direction the robot is facing (from the NavX gyro).
     * We negate it because the NavX and WPILib disagree about which
     * direction is "positive" -- this fixes that.
     */
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    /** Returns the NavX gyro object directly (for advanced use). */
    public AHRS getNavX(){
        return navX;
    }

    /**
     * Resets the gyro so the robot's current direction becomes "forward" (0 degrees).
     * Press this when field-relative driving feels off or at the start of a match.
     */
    public void resetHeading(){
        navX.reset();
    }

    /**
     * Zeroes the gyro AND the pose estimator's heading together, keeping the robot's
     * field position. Use this instead of resetHeading() so odometry and gyro agree.
     *
     * Assumes the robot is facing straight away from our DRIVER STATION. The gyro becomes 0
     * (so field-relative driving is "away from the driver" for both alliances), but the
     * field pose heading is set to the real field direction: 0 on blue (facing +X) and
     * 180 degrees on red (facing -X). Auto-aim and the ALLIANCE ZONE check rely on that.
     */
    public void zeroHeading(){
        navX.reset();
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
        poseEstimator.resetPosition(
            getHeading(),
            getModulePositions(),
            new Pose2d(getPose().getTranslation(), isRed ? Rotation2d.kPi : Rotation2d.kZero));
    }

    /**
     * One-time gyro zero after the NavX has booted, only while disabled so it can't jolt a
     * running robot, and only once the Driver Station has told us our alliance (needed to
     * set the field heading correctly on red).
     */
    private void zeroGyroAfterBoot(){
        if (gyroZeroed || !DriverStation.isDisabled() || DriverStation.getAlliance().isEmpty()) return;
        if (Timer.getFPGATimestamp() - constructedTime < 1.0) return;
        zeroHeading();
        gyroZeroed = true;
    }

    /**
     * Tells the robot "you are HERE on the field." Used at the start of auto
     * so the robot knows its starting position.
     */
    public void resetOdometry(Pose2d pose){
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /**
     * Tells each wheel how fast to spin and which direction to point.
     * Also sends the target angles to the dashboard so we can see them.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);

        // Log the commanded angles for debugging
        if (debugSub.get()) {
            SmartDashboard.putNumber("FL Set Position", desiredStates[0].angle.getRadians());
            SmartDashboard.putNumber("FR Set Position", desiredStates[1].angle.getRadians());
            SmartDashboard.putNumber("BL Set Position", desiredStates[2].angle.getRadians());
            SmartDashboard.putNumber("BR Set Position", desiredStates[3].angle.getRadians());
        }
    }

    /**
     * Returns the current position of each swerve module (distance traveled + angle).
     * Used by the pose estimator to track robot movement.
     *
     * @return array of 4 SwerveModulePositions [FL, FR, BL, BR]
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getCANCoderRad())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getCANCoderRad())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getCANCoderRad())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getCANCoderRad())),
        };
        return positions;
    }

    /**
     * Returns the current state of each swerve module (velocity + angle).
     * Used for kinematics calculations and logging.
     *
     * @return array of 4 SwerveModuleStates [FL, FR, BL, BR]
     */
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    /**
     * Sets whether driving should be field-relative or robot-relative.
     *
     * @param relativity true for field-relative, false for robot-relative
     */
    public void setFieldRelativity(boolean relativity){
        fieldRelativeStatus = relativity;
    }

    /**
     * The main drive method -- this is what actually makes the robot move.
     *
     * Takes three speeds (forward, sideways, and spin) and tells each wheel
     * what to do. Works in either field-relative or robot-relative mode.
     *
     * @param forward  forward/backward speed in m/s (positive = forward)
     * @param strafe   left/right speed in m/s (positive = left)
     * @param rotation spin speed in rad/s (positive = counter-clockwise)
     * @param isFieldRelative true = field-relative, false = robot-relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative){
        ChassisSpeeds speeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
            : new ChassisSpeeds(forward, strafe, rotation);

        // This fixes a problem where the robot drifts sideways when driving and
        // spinning at the same time. It adjusts for the robot moving during calculations.
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Save the speed data so we can replay and analyze it later
        Logger.recordOutput("ChassisSpeeds", speeds);

        // Figure out what each wheel needs to do, and make sure none go over max speed
        SwerveModuleState[] states = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);
        setModuleStates(states);

        // Log commanded speeds for debugging
        if (debugSub.get()) {
            SmartDashboard.putNumber("FL Set Speed", states[0].speedMetersPerSecond);
            SmartDashboard.putNumber("FR Set Speed", states[1].speedMetersPerSecond);
            SmartDashboard.putNumber("BL Set Speed", states[2].speedMetersPerSecond);
            SmartDashboard.putNumber("BR Set Speed", states[3].speedMetersPerSecond);
        }
    }

    /**
     * Runs every 20ms automatically. Updates the robot's position using wheel + gyro
     * data and camera readings, then sends debug info to the dashboard.
     */
    @Override
    public void periodic(){
        super.periodic();

        zeroGyroAfterBoot();
        updateAlerts();

        // Update position using wheel + gyro data (good but drifts over time)
        poseEstimator.update(getHeading(), getModulePositions());

        // Update position using Limelight data (fixes the drift from above)
        updateVisionPose();

        // Log data for AdvantageKit
        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("NavX Heading", getHeading());
        Logger.recordOutput("Odometry Pose", getPose());

        // Publish debug telemetry
        execute();
    }

    /**
     * Uses the Limelight (our vision camera) to update the robot's position on the field.
     *
     * The Limelight figures out where the robot is by looking at AprilTags, and
     * gives us position data. We use the x, y, and rotation to update our position,
     * and subtract the delay to account for the fact that the picture was taken
     * a few milliseconds ago.
     *
     * Does nothing if no target is visible or the data looks bad.
     */
    private void updateVisionPose() {
        // tv = "target valid" -- 1.0 means the Limelight sees an AprilTag
        if (tvSub.get() != 1.0) return;

        double[] botpose = botposeSub.get();
        if (botpose.length < 7) return;

        // Make sure the data isn't garbage (sometimes the Limelight sends bad data)
        for (double v : botpose) {
            if (Double.isNaN(v) || Double.isInfinite(v)) return;
        }

        // MegaTag1 array: [x,y,z,roll,pitch,yaw,latency_ms,tagCount,tagSpan,avgTagDist,avgTagArea]
        int tagCount = botpose.length > 7 ? (int) botpose[7] : 1;
        double avgDist = botpose.length > 9 ? botpose[9] : 0.0;
        if (tagCount < 1) return;

        // Reject poses that are likely wrong: off the field, too far from the tags, or
        // captured while spinning fast (camera latency makes the heading unreliable).
        if (botpose[0] < 0 || botpose[0] > FieldConstants.kFieldLength
            || botpose[1] < 0 || botpose[1] > FieldConstants.kFieldWidth) return;
        if (avgDist > VisionConstants.kMaxTagDistance) return;
        if (tagCount == 1 && avgDist > VisionConstants.kMaxSingleTagDistance) return;
        if (Math.abs(navX.getRate()) > VisionConstants.kMaxVisionYawRateDegPerSec) return;

        // Create a Pose2d from the vision data (x, y, yaw)
        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));

        // Account for the Limelight's delay -- the picture was taken a few ms ago
        double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);

        // Trust vision less the farther away the tags are and the fewer we see. Single-tag
        // heading is noisy, so it's left to the gyro; with 2+ tags vision heading is good
        // enough to slowly correct a wrong field heading (e.g. gyro zeroed facing the wrong way).
        double xyStd = VisionConstants.kXYStdBase * (1.0 + avgDist * avgDist) / tagCount;
        double thetaStd = tagCount >= 2 ? VisionConstants.kThetaStdMultiTag : VisionConstants.kThetaStd;

        // Feed the vision measurement into the pose estimator
        poseEstimator.addVisionMeasurement(
            visionPose, timestamp, VecBuilder.fill(xyStd, xyStd, thetaStd));
    }

    /** Flags hardware problems on the dashboard so the pit crew sees them before a match. */
    private void updateAlerts() {
        navXAlert.set(!navX.isConnected());

        double now = Timer.getFPGATimestamp();
        double hb = heartbeatSub.get();
        if (hb != lastHeartbeat) {
            lastHeartbeat = hb;
            lastHeartbeatTime = now;
        }
        limelightAlert.set(now - lastHeartbeatTime > 1.0);
    }

    /**
     * Returns true if the Limelight currently sees a valid AprilTag target.
     *
     * @return true if a target is visible
     */
    public boolean getLimelightHasTarget() {
        return tvSub.get() == 1.0;
    }

    /**
     * Returns how far left or right the target is from the camera's center (in degrees).
     * Negative = target is to the left, positive = to the right.
     * The AlignToTag command uses this to auto-rotate toward the target.
     */
    public double getLimelightTx() {
        return txSub.get();
    }

    /**
     * Returns which AprilTag number the camera is looking at, or -1 if it doesn't see one.
     * The auto-shooter uses this to make sure we're aiming at our own alliance's goal.
     */
    public int getLimelightTid() {
        return (int) tidSub.get();
    }

    /**
     * Returns how far away the target is (in meters). Uses the camera's 3D data
     * to calculate the straight-line distance. Returns -1.0 if no target is visible.
     */
    public double getLimelightTargetDistanceMeters() {
        double[] targetpose = targetposeSub.get();
        if (targetpose.length < 3) return -1.0;
        return Math.sqrt(targetpose[0] * targetpose[0]
            + targetpose[1] * targetpose[1]
            + targetpose[2] * targetpose[2]);
    }

    /**
     * Publishes encoder values and sensor readings to SmartDashboard for debugging.
     * Called every 20ms from periodic().
     */
    protected void execute() {
        SmartDashboard.putBoolean("Field Realtive", fieldRelativeStatus);

        // Detailed per-module telemetry is only published when "Debug Swerve" is on.
        if (!debugSub.get()) return;

        // Drive encoder velocities (m/s)
        SmartDashboard.putNumber("FL Drive Encoder", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Drive Encoder", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Drive Encoder", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Drive Encoder", backRight.getDriveVelocity());

        // Steering encoder positions (radians)
        SmartDashboard.putNumber("FL Angle Position", frontLeft.getRotationPosition());
        SmartDashboard.putNumber("FR Angle Position", frontRight.getRotationPosition());
        SmartDashboard.putNumber("BL Angle Position", backLeft.getRotationPosition());
        SmartDashboard.putNumber("BR Angle Position", backRight.getRotationPosition());

        // NavX heading (degrees)
        SmartDashboard.putNumber("Yaw", -navX.getYaw());
    }
}
