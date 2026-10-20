package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;

/**
 * Represents a single swerve module -- one corner of the swerve drivetrain.
 *
 * Each swerve module has three components:
 *   1. Drive motor (SparkMax + NEO) -- spins the wheel to move the robot
 *   2. Rotation motor (SparkMax + NEO) -- steers the wheel to point in any direction
 *   3. CANcoder (absolute encoder) -- always knows the exact angle of the wheel,
 *      even after a power cycle
 *
 * Think of each module like a shopping cart wheel that you can independently control
 * both the speed AND the direction. Four of these working together let the robot
 * move in any direction while spinning.
 *
 * How steering works:
 *   The steering motor uses PID to turn the wheel to the right angle. It's smart
 *   enough to know that angles wrap around in a circle, so it always turns the
 *   SHORT way. For example, if the wheel needs to go from 350 degrees to 10 degrees,
 *   it turns 20 degrees forward instead of 340 degrees backward.
 *
 * How the CANcoder helps:
 *   The motor's built-in encoder only counts from when it started -- if the robot
 *   loses power, it forgets where the wheel was pointing. The CANcoder is different:
 *   it ALWAYS knows the real angle, even after a reboot. We use the CANcoder to
 *   tell the motor encoder "this is where you actually are" when the robot starts up.
 *
 * WANT TO CHANGE steering behavior? Adjust kPRotation, kDRotation in DrivetrainConstants.
 * WANT TO CHANGE drive behavior? Adjust kPDrive in DrivetrainConstants.
 */
public class SwerveModule {
    /** SparkMax controller for the drive (forward/backward) motor. */
    private final SparkMax driveMotor;

    /** SparkMax controller for the rotation (steering) motor. */
    private final SparkMax rotationMotor;

    /** Built-in encoder on the drive motor. Tracks distance traveled. */
    private final RelativeEncoder driveEncoder;

    /** Built-in encoder on the rotation motor. Tracks steering angle changes. */
    private final RelativeEncoder rotationEncoder;

    /** Absolute encoder (CANcoder) that always knows the real wheel angle. */
    private final CANcoder canCoder;

    /** Offset to subtract from the CANcoder reading to calibrate "straight ahead" = 0. */
    private final double canCoderOffsetRadians;

    /** PID controller for steering the wheel to the target angle. */
    private final PIDController rotationPIDController;

    /** PID controller for driving the wheel at the target speed. */
    private final PIDController drivePIDController;

    /**
     * Creates a new swerve module with the given hardware IDs and configuration.
     *
     * @param driveMotorID       CAN ID of the drive motor SparkMax
     * @param rotationMotorID    CAN ID of the rotation motor SparkMax
     * @param canCoderID         CAN ID of the CANcoder absolute encoder
     * @param canCoderOffsetRadians  calibration offset for the CANcoder (in radians)
     * @param isDriveInverted    whether the drive motor direction is inverted
     * @param x                  P-gain for the rotation PID controller
     */
    public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int canCoderID,
      double canCoderOffsetRadians,
      boolean isDriveInverted,
      double x
    ) {
      // Create motor controllers. kBrushless because we use NEO motors.
      driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
      rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

      // Configure both motors with appropriate settings:
      // - Brake mode: motors resist movement when not powered (robot doesn't coast)
      // - Conversion factors: translate motor rotations into real-world units (meters, radians)
      configureMotor(driveMotor,
                      isDriveInverted,
                      IdleMode.kBrake,
                      DrivetrainConstants.drivePositionConversionFactor,
                      DrivetrainConstants.driveVelocityConversionFactor,
                      DrivetrainConstants.driveCurrentLimit
      );
      configureMotor(rotationMotor,
                      isDriveInverted,
                      IdleMode.kBrake,
                      DrivetrainConstants.rotationPositionConversionFactor,
                      DrivetrainConstants.rotationVelocityConversionFactor,
                      DrivetrainConstants.rotationCurrentLimit
      );

      // Clear any fault flags from previous runs
      driveMotor.clearFaults();
      rotationMotor.clearFaults();

      // --- PID Controllers ---
      // Drive PID: controls wheel speed to match the target velocity
      drivePIDController = new PIDController(DrivetrainConstants.kPDrive, DrivetrainConstants.kIDrive, DrivetrainConstants.kDDrive);

      // Rotation PID: controls wheel angle to match the target direction
      // The 'x' parameter allows per-module P-gain tuning (passed from SwerveSubsystem)
      rotationPIDController = new PIDController(x, DrivetrainConstants.kIRotation, DrivetrainConstants.kDRotation);
      rotationPIDController.setTolerance(DrivetrainConstants.kToleranceRotation);

      // Tell the PID that angles wrap around in a circle (-180 and +180 are the same).
      // Without this, the wheel might spin the long way around instead of taking a shortcut.
      rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

      // --- Encoders ---
      // CANcoder gives absolute position (survives power cycles)
      canCoder = new CANcoder(canCoderID);
      this.canCoderOffsetRadians = canCoderOffsetRadians;
      configureCanCoder();

      // Relative encoders from the SparkMax (reset to CANcoder position on startup)
      driveEncoder = driveMotor.getEncoder();
      rotationEncoder = rotationMotor.getEncoder();
    }

    /**
     * Applies configuration to a SparkMax motor controller.
     * Sets inversion, idle mode (brake/coast), and encoder conversion factors.
     *
     * @param motorController        the SparkMax to configure
     * @param isInverted             whether to invert the motor direction
     * @param idleMode               kBrake (resist motion) or kCoast (free spin) when not powered
     * @param positionConversionFactor  multiplier to convert motor rotations to real units
     * @param velocityConversionFactor  multiplier to convert motor RPM to real units/sec
     * @param currentLimit           max motor current in Amps (prevents brownouts)
     */
    public void configureMotor(
      SparkMax motorController,
      boolean isInverted,
      IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor,
      int currentLimit
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
          .inverted(isInverted)
          .idleMode(idleMode)
          .smartCurrentLimit(currentLimit)
          .voltageCompensation(ShooterConstants.kNominalVoltage);
        config.encoder
          .positionConversionFactor(positionConversionFactor)
          .velocityConversionFactor(velocityConversionFactor);

          motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      }

    /**
     * Sets up the CANcoder (the sensor that always knows the wheel angle).
     * We configure it to report values from 0 to 1 (one full rotation) and to
     * treat counter-clockwise as the positive direction (to match WPILib).
     */
    public void configureCanCoder(){
      CANcoderConfiguration config = new CANcoderConfiguration();

      // [0, 1) wrap range for calculating radians cleanly
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      // Counter-clockwise is positive (matches WPILib convention)
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      // Apply the configuration to the CANcoder
      canCoder.getConfigurator().apply(config);
    }

    /**
     * Sets the desired state (speed + angle) for this swerve module.
     *
     * If the desired speed is essentially zero (< 0.0001 m/s), the module stops
     * entirely. This prevents the wheels from snapping to an angle when the driver
     * isn't trying to move -- the modules just hold their current position.
     *
     * @param state the desired SwerveModuleState (speed in m/s, angle in radians)
     */
    public void setState(SwerveModuleState state){
      // Dead zone: if speed is basically zero, just stop. This prevents the wheels
      // from jittering to new angles when the robot should be stationary.
      if (Math.abs(state.speedMetersPerSecond) < 0.0001){
        stop();
        return;
      }

      // Take the shortest steer (never more than 90 degrees, reversing the wheel if needed),
      // and slow the wheel while it is still pointing the wrong way.
      Rotation2d currentAngle = new Rotation2d(getCANCoderRad());
      SwerveModuleState optimized = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
      optimized.optimize(currentAngle);
      optimized.cosineScale(currentAngle);

      // Use PID to steer the wheel to the target angle
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), optimized.angle.getRadians()));

      // Feedforward does most of the work (duty cycle proportional to speed); PID trims the error.
      driveMotor.set(
          DrivetrainConstants.kVDrive * optimized.speedMetersPerSecond
          + drivePIDController.calculate(getDriveVelocity(), optimized.speedMetersPerSecond));
    }

    /**
     * Returns the current state of this module (velocity + angle).
     *
     * @return a SwerveModuleState with current velocity (m/s) and angle (radians)
     */
    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
    }

    /**
     * Initializes the relative rotation encoder to match the CANcoder's absolute position.
     * Called once on startup to sync the encoders.
     */
    public void initRotationOffset() {
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /**
     * Returns the wheel's current angle in radians, corrected for the calibration offset.
     * The offset makes it so that "straight ahead" reads as 0.
     */
    public double getCANCoderRad() {
      double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
      double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;
      return angle % (2 * Math.PI);
    }

    /**
     * Commands the rotation motor to turn to a specific angle using PID.
     *
     * @param angleRad the target angle in radians
     */
    public void setRotationMotorAnglePID(double angleRad) {
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), angleRad));
    }

    /**
     * Resets the encoders. Drive encoder goes to 0 (we start measuring distance fresh).
     * Rotation encoder syncs to the CANcoder's current absolute position.
     */
    public void resetEncoder() {
      driveEncoder.setPosition(0);
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /**
     * Returns the drive motor velocity in meters per second.
     * The conversion factor (set during motor configuration) handles the
     * RPM-to-m/s conversion automatically.
     *
     * @return drive velocity in m/s
     */
    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }

    /**
     * Returns the rotation motor velocity in radians per second.
     *
     * @return rotation velocity in rad/s
     */
    public double getRotationVelocity(){
      return rotationEncoder.getVelocity();
    }

    /**
     * Stops both motors (drive and rotation) immediately.
     */
    public void stop(){
      driveMotor.stopMotor();
      rotationMotor.stopMotor();
    }

    /**
     * Returns the drive motor position in meters (total distance traveled).
     *
     * @return distance traveled in meters
     */
    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }

    /**
     * Returns the current rotation angle from the CANcoder in radians.
     * Uses the absolute encoder for accuracy (immune to drift).
     *
     * @return current rotation position in radians
     */
    public double getRotationPosition() {
      return getCANCoderRad();
    }

}
