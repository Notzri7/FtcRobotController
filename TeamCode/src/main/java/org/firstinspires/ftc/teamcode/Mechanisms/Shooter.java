package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor shooterMotor;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        // Use FLOAT so the heavy flywheel spins down naturally without shocking the gearbox
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runShooter(boolean shootButton, boolean stopButton, boolean feedButton, Intake intakeSystem) {
        //Adjust 0.85 power up or down based on your distance to the target goal.
        if (shootButton) {
            shooterMotor.setPower(0.85); // Spin up flywheel to shooting speed
        } else if (stopButton) {
            shooterMotor.setPower(0.0); // Stop flywheel
        }

        // When feed button is held, push pollen from transfer into the flywheel
        if (feedButton) {
            intakeSystem.setTransferPower(0.9);
        }
    }
}
