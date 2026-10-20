package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Controls the shooter mechanism -- a dual-motor flywheel system with a kicker that
 * launches game pieces (FUEL) into the HUB.
 *
 * The shooter has THREE motors working together:
 *   1. Shooter Motor 1 (CAN ID 42) -- one side of the flywheel
 *   2. Shooter Motor 2 (CAN ID 43) -- other side of the flywheel
 *   3. Kicker Motor (CAN ID 44) -- pushes the game piece into the spinning flywheel
 *
 * How it works:
 *   - The two shooter motors spin up a flywheel to the desired speed.
 *   - Once the flywheel is at speed, the kicker motor pushes the game piece
 *     into the flywheel, which launches it toward the HUB.
 *   - The flywheels are speed-controlled (feedforward + P on measured RPM), so shot speed
 *     doesn't sag with battery voltage. The kicker only runs once both flywheels are
 *     up to speed, and keeps running through the small RPM dip when a ball is launched.
 *   - The kicker runs at 50% speed.
 *
 * Both shooter motors receive the same speed value. If you want them to spin at
 * different speeds (e.g., to add spin to the game piece), you'd modify setIOSpark.
 *
 * All motors use brake mode and have current limits to prevent damage.
 *
 * WANT TO CHANGE shooter speed? See ShooterConstants.shooterSpeed in Constants.java.
 * WANT TO CHANGE the distance-to-speed lookup? See ShooterConstants.kDistanceToSpeedMap.
 * WANT TO CHANGE motor CAN IDs? See ShooterConstants in Constants.java.
 */
public class ShooterMechanism extends SubsystemBase {

    /** SparkMax controller for the first shooter flywheel motor. */
    private final SparkMax shooterSpark1;

    /** SparkMax controller for the second shooter flywheel motor. */
    private final SparkMax shooterSpark2;

    /** SparkMax controller for the kicker/feeder motor. */
    private final SparkMax kickerSpark;

    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;
    private final PIDController pid1 = new PIDController(ShooterConstants.kFlywheelP, 0, 0);
    private final PIDController pid2 = new PIDController(ShooterConstants.kFlywheelP, 0, 0);

    /** True while the flywheels are at speed and the kicker is feeding. */
    private boolean ready = false;

    /** Time the current spin-up began, or NaN when the shooter is stopped. */
    private double spinUpStart = Double.NaN;

    /**
     * Creates a new ShooterMechanism with the given motor ports.
     *
     * @param shooterSparkPort1 CAN ID of the first shooter motor
     * @param shooterSparkPort2 CAN ID of the second shooter motor
     * @param kickerSparkPort   CAN ID of the kicker motor
     */
    public ShooterMechanism(
        int shooterSparkPort1,
        int shooterSparkPort2,
        int kickerSparkPort
    ) {
        shooterSpark1 = new SparkMax(shooterSparkPort1, MotorType.kBrushless);
        shooterSpark2 = new SparkMax(shooterSparkPort2, MotorType.kBrushless);
        kickerSpark = new SparkMax(kickerSparkPort, MotorType.kBrushless);

        // Flywheels coast (they spin down without fighting the brake and re-spin faster);
        // the kicker brakes so it stops feeding immediately.
        configureMotor(shooterSpark1, IdleMode.kCoast, ShooterConstants.currentLimit);
        configureMotor(shooterSpark2, IdleMode.kCoast, ShooterConstants.currentLimit);
        configureMotor(kickerSpark, IdleMode.kBrake, ShooterConstants.currentLimit);

        encoder1 = shooterSpark1.getEncoder();
        encoder2 = shooterSpark2.getEncoder();
    }

    /**
     * Configures a SparkMax motor controller with brake mode and a current limit.
     *
     * @param spark        the SparkMax to configure
     * @param idleMode     kBrake or kCoast
     * @param currentLimit maximum current draw in Amps
     */
    public void configureMotor(
        SparkMax spark,
        IdleMode idleMode,
        int currentLimit
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(idleMode)
            .smartCurrentLimit(currentLimit)
            .voltageCompensation(ShooterConstants.kNominalVoltage);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the flywheels and feeds the kicker once both are up to speed. Used by the manual
     * shot, where the driver decides when it's safe to fire.
     */
    public void setIOSpark(double speed1, double speed2) {
        run(speed1, speed2, true);
    }

    /**
     * Runs the flywheels at the given fractions of NEO free speed. The kicker feeds only when
     * {@code allowFeed} is true AND the flywheels are up to speed. Call every loop.
     *
     * @param speed1    target for shooter motor 1 (0.0 to 1.0 of free speed)
     * @param speed2    target for shooter motor 2 (0.0 to 1.0 of free speed)
     * @param allowFeed false keeps the flywheels spinning but holds the game pieces
     *                  (e.g. our HUB is inactive, or we're outside our ALLIANCE ZONE)
     */
    public void run(double speed1, double speed2, boolean allowFeed) {
        double rpm1 = encoder1.getVelocity();
        double rpm2 = encoder2.getVelocity();
        double target1 = speed1 * ShooterConstants.kFreeSpeedRPM;
        double target2 = speed2 * ShooterConstants.kFreeSpeedRPM;

        // Feedforward (duty cycle ~ fraction of free speed) plus P correction on measured RPM
        shooterSpark1.set(speed1 + pid1.calculate(rpm1, target1));
        shooterSpark2.set(speed2 + pid2.calculate(rpm2, target2));

        // Hysteresis: start feeding when close to target, keep feeding through the ball dip
        double tol = ready ? ShooterConstants.kNotReadyTolerance : ShooterConstants.kReadyTolerance;
        ready = target1 > 0 && target2 > 0
            && Math.abs(rpm1 - target1) <= tol * target1
            && Math.abs(rpm2 - target2) <= tol * target2;

        // Safety valve: if the flywheels never reach target (bad tune, mechanical issue),
        // feed anyway after a while rather than never shooting.
        double now = Timer.getFPGATimestamp();
        if (Double.isNaN(spinUpStart)) spinUpStart = now;
        boolean timedOut = now - spinUpStart > ShooterConstants.kMaxSpinUpSeconds;

        kickerSpark.set(allowFeed && (ready || timedOut) ? 0.5 : 0.0);

        SmartDashboard.putNumber("Shooter/RPM1", rpm1);
        SmartDashboard.putNumber("Shooter/RPM2", rpm2);
        SmartDashboard.putBoolean("Shooter/Ready", ready);
    }

    /** True while the flywheels are at speed and the kicker is feeding. */
    public boolean isReady() {
        return ready;
    }

    /**
     * Stops all three motors (both shooter motors and the kicker) immediately.
     * Called when shooter commands end.
     */
    public void stopIOSpark() {
        shooterSpark1.stopMotor();
        shooterSpark2.stopMotor();
        kickerSpark.stopMotor();
        pid1.reset();
        pid2.reset();
        ready = false;
        spinUpStart = Double.NaN;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
