package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Shooter {
    private DcMotorEx shooterMotor;
    private VoltageSensor batteryVoltageSensor;

    // Encoder resolution for the 1:1 goBILDA 5203 motor is 28 Pulses Per Revolution (PPR)

    // Change if a gear or pulley changes how many encoder counts equal one flywheel turn.
    private static final double COUNTS_PER_REV = 28.0;

    // Highest allowed flywheel speed (RPM) for safety and motor limits.
    private static final double MAX_RPM = 5500.0;

    // Base push (kF) in FTC Dashboard to keep the shooter flywheel motor near target RPM.
    public static double kF = 11.7025;

    // Correction push (kP) in FTC Dashboard; raise if flywheel stops short, lower if it overshoots.
    public static double kP = 1.3;

    //  Steady-error push (kI) in FTC Dashboard; keep small or 0.0 to avoid overshoot.
    public static double kI = 0.0;

    // Damping value (kD) in FTC Dashboard; keep at 0.0 unless shooter flywheel speed oscillates.
    public static double kD = 0.0;

    private double lastAppliedP = Double.NaN;
    private double lastAppliedI = Double.NaN;
    private double lastAppliedD = Double.NaN;
    private double lastAppliedF = Double.NaN;

    // Default flywheel target speed (RPM) when the op mode starts.
    private double targetRPM = 3000.0;
    private boolean flywheelEnabled = false;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Use FLOAT so the heavy flywheel spins down naturally without shocking the gearbox
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Flip to REVERSE if your motor spins backward
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        if (!hardwareMap.voltageSensor.entrySet().isEmpty()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
        syncPidIfchanged();
        stop();
    }

    public void runShooter(boolean preset2000Button, boolean preset2300Button, boolean stopButton) {
        if (preset2000Button) {
            spinToRPM(3000.0);
        } else if (preset2300Button) {
            spinToRPM(3200.0);
        }

        if (stopButton) {
            stop();
            return;
        }

        applyFlywheelOutput();
    }

    // Sets a target flywheel speed in RPM and turns the flywheel on.
    public void spinToRPM(double rpm) {
        this.targetRPM = Math.max(-MAX_RPM, Math.min(MAX_RPM, rpm));
        this.flywheelEnabled = true;
        applyFlywheelOutput();
    }

    // Sends the target speed in encoder ticks per second to the flywheel motor.
    private void applyFlywheelOutput() {
        syncPidIfchanged();
        if (flywheelEnabled) {
            double targetTicksPerSec = (targetRPM * COUNTS_PER_REV) / 60.0;
            shooterMotor.setVelocity(targetTicksPerSec);
        } else {
            shooterMotor.setVelocity(0.0);
        }
    }

    // Pushes new kP, kI, kD, and kF values from FTC Dashboard to the shooter flywheel motor when edited.
    private void syncPidIfchanged() {
        if (kP != lastAppliedP || kI != lastAppliedI || kD != lastAppliedD || kF != lastAppliedF) {
            shooterMotor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP, kI, kD, kF)
            );
            lastAppliedP = kP;
            lastAppliedI = kI;
            lastAppliedD = kD;
            lastAppliedF = kF;
        }
    }

    // Checks whether the flywheel is within toleranceRPM of the target speed.
    public boolean isAtSpeed(double toleranceRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) <= toleranceRPM;
    }

    // Turns off the flywheel and sets its speed command to zero.
    public void stop() {
        flywheelEnabled = false;
        shooterMotor.setVelocity(0.0);
    }

    // Returns the current target flywheel speed in RPM.
    public double getTargetRPM() {
        return targetRPM;
    }

    // Returns the flywheel's actual measured speed in RPM from its encoder.
    public double getCurrentRPM() {
        return (shooterMotor.getVelocity() / COUNTS_PER_REV) * 60.0;
    }

    // Returns the current robot battery voltage, or 0.0 if no voltage sensor is found.
    public double getBatteryVoltage() {
        return batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;    }
}