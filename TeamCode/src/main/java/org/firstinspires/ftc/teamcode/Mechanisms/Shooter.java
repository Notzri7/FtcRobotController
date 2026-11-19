package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Shooter {
    private DcMotorEx shooterMotor;
    private VoltageSensor batteryVoltageSensor;

    // Encoder resolution for the 1:1 goBILDA 5203 motor is 28 Pulses Per Revolution (PPR)
    private static final double COUNTS_PER_REV = 28.0;

    // Default starting target RPM (safe and efficient instead of max 6000 RPM)
    private double targetRPM = 2000.0;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Use FLOAT so the heavy flywheel spins down naturally without shocking the gearbox
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Flip to REVERSE if your motor spins backward
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooterMotor.setPower(0.0);

    }

    public void runShooter(boolean preset2000Button, boolean preset2300Button, boolean stopButton, boolean feedButton, Intake intakeSystem) {
        //Adjust 0.85 power up or down based on your distance to the target goal.
        if (preset2000Button) {
            targetRPM = 2000.0;
        } else if (preset2300Button) {
            targetRPM = 2300.0;
        }

        if (stopButton) {
            shooterMotor.setPower(0.0); // Stop flywheel
        } else {
            // Convert target RPM to Encoder Ticks Per Second for REV's built-in PID:
            // Formula: Ticks Per Sec = (Target RPM * Ticks Per Rev) / 60
            double targetTicksPerSec = (targetRPM * COUNTS_PER_REV) / 60.0;
            double voltageScale = 12.0 / batteryVoltageSensor.getVoltage();
            shooterMotor.setVelocity(targetTicksPerSec * voltageScale);
        }

        // When feed button is held, push pollen from transfer into the flywheel
        if (feedButton) {
            intakeSystem.setTransferPower(0.9);
        }
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return (shooterMotor.getVelocity() / COUNTS_PER_REV) * 60.0;
    }
    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }
}
