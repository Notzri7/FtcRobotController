package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Shooter {
    private DcMotorEx shooterMotor;
    private VoltageSensor batteryVoltageSensor;

    // Encoder resolution for the 1:1 goBILDA 5203 motor is 28 Pulses Per Revolution (PPR)

    // Change if a gear or pulley changes how many encoder counts equal one flywheel turn.
    private static final double COUNTS_PER_REV = 28.0;

    // Highest allowed flywheel speed (RPM) for safety and motor limits.
    private static final double MAX_RPM = 5500.0;

    // Motor no-load speed (RPM) used to estimate baseline speed control.
    private static final double MOTOR_MAX_RPM = 6000.0;

    // Proportional gain multiplier—raise slightly if flywheel stops short, lower if it overshoots.
    private static final double P_TO_F_RATIO = 0.12;

    // Integral gain multiplier—keep small or 0.0 to avoid overshoot or slow recovery.
    private static final double I_TO_P_RATIO = 0.05;

    // Default flywheel target speed (RPM) when the op mode starts.
    private double targetRPM = 2000.0;
    private boolean flywheelEnabled = false;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tune velocity PIDF for a 1:1 6000 RPM (28 PPR) motor to reduce overshoot
        double maxTicksPerSec = (MOTOR_MAX_RPM * COUNTS_PER_REV) / 60.0; // ~2800 ticks/sec
        double fCoeff = 32767.0 / maxTicksPerSec;
        double pCoeff = fCoeff * P_TO_F_RATIO;
        double iCoeff = pCoeff * I_TO_P_RATIO;
        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pCoeff, iCoeff, 0.0, fCoeff)
        );

        // Use FLOAT so the heavy flywheel spins down naturally without shocking the gearbox
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Flip to REVERSE if your motor spins backward
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        if (!hardwareMap.voltageSensor.entrySet().isEmpty()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
        stop();
    }

    public void runShooter(boolean preset2000Button, boolean preset2300Button, boolean stopButton) {
        if (preset2000Button) {
            spinToRPM(2000.0);
        } else if (preset2300Button) {
            spinToRPM(2300.0);
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
        if (flywheelEnabled) {
            double targetTicksPerSec = (targetRPM * COUNTS_PER_REV) / 60.0;
            shooterMotor.setVelocity(targetTicksPerSec);
        } else {
            shooterMotor.setVelocity(0.0);
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