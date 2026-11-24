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

    // to tune Temporarily set kP = 0 and bangBangThresholdRPM = 9999, then spin at 3000 RPM:
    // - If Actual RPM is too low -> increase  kV slightly
    // - If Actual RPM is too high  -> decrease kV slightly
    public static double kV = 0.000167;

    // minimum power to overcome bearing drag
    // If wheel is sluggish to start or settles slightly below target at ALL speeds -> increase
    // If wheel overshoots target at low speeds -> decrease (0.01 - 0.03)
    public static double kS = 0.04;

    // Fine error correction when near target speed
    // If Actual RPM hovers 20-50 RPM away from target -> increase slightly
    // If Actual RPM vibrates/oscillates rapidly up and down -> decrease
    public static double kP = 0.0008;

    // If RPM drops further than this below target after a shot, slam 100% power
    // If 2nd shot still drops short (recovery is too slow) -> lower
    // If wheel surges/clicks randomly while idling with no ball -> raise
    public static double bangBangThresholdRPM = 120.0;

    // two driver preset speeds
    public static double PRESET_1_RPM = 3000.0;
    public static double PRESET_2_RPM = 3200.0;

    private double targetRPM = PRESET_1_RPM;
    private boolean flywheelEnabled = false;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Use FLOAT so the heavy flywheel spins down naturally without shocking the gearbox
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Flip to REVERSE if your motor spins backward
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        if (!hardwareMap.voltageSensor.entrySet().isEmpty()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
        stop();
    }

    public void runShooter(boolean lowerPresetButton, boolean HigherPresetButton, boolean stopButton) {
        if (lowerPresetButton) {
            spinToRPM(PRESET_1_RPM);
        } else if (HigherPresetButton) {
            spinToRPM(PRESET_2_RPM);
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
        if (!flywheelEnabled || targetRPM == 0.0) {
            shooterMotor.setPower(0.0);
            return;
        }

        double currentRPM = getCurrentRPM();
        double error = targetRPM - currentRPM;

        // This is where the bang bang controller activates
        if (error > bangBangThresholdRPM) {
            shooterMotor.setPower(1.0);
            return;
        }

        // Voltage normalization so 3000 RPM shoots the same at 13.5V and 11.8V
        double voltage = getBatteryVoltage();
        double voltageScale = (voltage > 8.0) ? (12.0/voltage) : 1.0;

        double ffPower = (kS * Math.signum(targetRPM) + (kV * targetRPM));
        double pPower = kP * error;

        double totalPower = (ffPower + pPower) * voltageScale;
        shooterMotor.setPower(Math.max(-1.0, Math.min(1.0, totalPower)));

    }

    // Checks whether the flywheel is within toleranceRPM of the target speed.
    public boolean isAtSpeed(double toleranceRPM) {
        return flywheelEnabled && Math.abs(getCurrentRPM() - targetRPM) <= toleranceRPM;
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