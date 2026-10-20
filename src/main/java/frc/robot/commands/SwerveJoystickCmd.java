package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The default teleop driving command -- this is what runs whenever the driver is
 * using the joysticks to control the robot.
 *
 * This command reads joystick inputs and converts them into swerve drive commands.
 * It applies several processing steps to make driving feel smooth and controllable:
 *
 *   1. DEADBANDS: Small joystick movements (within the deadband zone) are treated as
 *      zero. This prevents the robot from drifting when the driver's thumbs aren't
 *      perfectly centered on the sticks.
 *
 *   2. SMOOTH ACCELERATION: Limits how quickly the speed can change. Instead of
 *      jumping from 0 to max speed instantly (which would jerk the robot), the speed
 *      ramps up smoothly. Like the difference between slamming the gas pedal vs.
 *      gradually pressing it.
 *
 *   3. SPEED SLIDER: The left trigger acts as a "slow-down" control. Pulling it reduces
 *      ALL movement speeds proportionally. Fully pulled = 80% speed reduction.
 *      This gives the driver fine control for precise alignment.
 *
 * This command never finishes on its own (isFinished returns false) -- it runs
 * continuously as the drivetrain's default command until another command takes over.
 *
 * WANT TO CHANGE driving feel? Adjust deadbands in OperatorConstants, or max speeds
 * in DrivetrainConstants.
 * WANT TO CHANGE the slider range? Modify the sliderVal cap (currently 0.8 = 80% max reduction).
 */
public class SwerveJoystickCmd extends Command {
    public final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY, rotation, slider;

    /** Smooths out acceleration so the robot doesn't jerk when the stick moves fast. */
    private final SlewRateLimiter xLimiter, yLimiter, rLimiter;

    /**
     * Creates the default teleop drive command.
     *
     * @param swerveSubsystem the drivetrain to control
     * @param forwardX       forward/backward input (left stick Y)
     * @param forwardY       left/right input (left stick X)
     * @param rotation       spin input (right stick X)
     * @param slider         speed slider input (left trigger)
     */
    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem,
        DoubleSupplier forwardX,
        DoubleSupplier forwardY,
        DoubleSupplier rotation,
        DoubleSupplier slider
        ) {
        this.drivetrain = swerveSubsystem;
        this.forwardX = forwardX;
        this.forwardY = forwardY;
        this.rotation = rotation;
        this.slider = slider;

        // These limit how fast the speed can change (prevents jerky movement).
        // The limiters work on the normalized (-1 to 1) stick value, so convert the
        // real-unit limits (m/s^2, rad/s^2) into "fraction of full speed per second".
        xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration / DrivetrainConstants.maxVelocity);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration / DrivetrainConstants.maxVelocity);
        rLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularAcceleration / DrivetrainConstants.maxAngularVelocity);

        // Tell the scheduler that this command uses the drivetrain.
        // This prevents other commands from using the drivetrain at the same time.
        addRequirements(swerveSubsystem);
    }

    /** Start smoothing from standstill so stale limiter values don't cause a lurch. */
    @Override
    public void initialize(){
        xLimiter.reset(0);
        yLimiter.reset(0);
        rLimiter.reset(0);
    }

    /**
     * Called every 20ms. Reads joystick values, applies deadband + slew rate limiting,
     * applies the speed slider, and sends the processed speeds to the drivetrain.
     */
    @Override
    public void execute(){
        // Read raw joystick values (-1.0 to 1.0)
        double xSpeed = forwardX.getAsDouble();
        double ySpeed = forwardY.getAsDouble();
        double rot = rotation.getAsDouble();
        double sliderVal = slider.getAsDouble();

        // Apply deadband (ignore tiny movements) and slew rate limiting (smooth acceleration)
        // Then scale to the maximum speed in real units (m/s or rad/s)
        xSpeed = applyDeadbandAndLimiter(xSpeed, OperatorConstants.xDeadband, xLimiter, DrivetrainConstants.maxVelocity);
        ySpeed = applyDeadbandAndLimiter(ySpeed, OperatorConstants.yDeadband, yLimiter, DrivetrainConstants.maxVelocity);
        rot = applyDeadbandAndLimiter(rot, OperatorConstants.rDeadband, rLimiter, DrivetrainConstants.maxAngularVelocity);

        // Speed slider: left trigger (0.0 = not pulled, 1.0 = fully pulled).
        // Cap at 0.8 so the robot never slows to a complete stop from the slider alone.
        // The formula (1 - sliderVal) means: fully released = full speed, pulled = reduced speed.
        sliderVal = Math.min(sliderVal, 0.8);
        xSpeed *= (1 - sliderVal);
        ySpeed *= (1 - sliderVal);
        rot *= (1 - sliderVal);

        // Send the processed speeds to the drivetrain
        drivetrain.drive(
            xSpeed,
            ySpeed,
            rot,
            SwerveSubsystem.fieldRelativeStatus
        );
    }

    /**
     * Ignores tiny stick movements (deadband), smooths out changes, and scales
     * the value to real-world speed units (m/s or rad/s).
     */
    public double applyDeadbandAndLimiter(
        double value,
        double deadband,
        SlewRateLimiter limiter,
        double maxSpeed
    ) {
        return limiter.calculate(shapeInput(value, deadband))*maxSpeed;
    }

    /**
     * Deadband that rescales the remaining range (so there's no jump when the stick leaves
     * the deadband), then squares the value (keeping sign) for finer control at low speeds.
     */
    public static double shapeInput(double value, double deadband) {
        double v = MathUtil.applyDeadband(value, deadband);
        return Math.copySign(v * v, v);
    }

    @Override
    public void end(boolean interrupted){}

    /** Never finishes -- runs continuously as the default command. */
    @Override
    public boolean isFinished(){
        return false;
    }
}
