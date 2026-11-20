package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private DcMotorEx transferMotor;

    // 1:1 goBILDA 5203 = 28 CPR (ticks per motor rev)
    // Change if a gear or belt changes how many encoder counts equal one transfer turn.
    private static final double TRANSFER_TICKS_PER_REV = 28.0;

    // Transfer speed (RPM) when pushing game pieces into the shooter or running alone.
    private static final double TRANSFER_FEED_RPM = 3200.0;

    // Power (-1.0 to 1.0) sent to the front intake roller when pulling game pieces in.
    private static final double INTAKE_COLLECT_POWER = 1.0;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");

        // Flip setDirection to REVERSE if the intake roller or transfer spins backward.
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);

        // Intake has NO encoder
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Transfer HAS an encoder
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    public void update(
            boolean intakeOnly,
            boolean transferOnly,
            boolean bothButton
    ) {
        if (bothButton) {
            intakeMotor.setPower(INTAKE_COLLECT_POWER);
            setTransferRPM(TRANSFER_FEED_RPM);
        } else if (intakeOnly) {
            intakeMotor.setPower(INTAKE_COLLECT_POWER);
            setTransferRPM(0.0);
        } else if (transferOnly) {
            intakeMotor.setPower(0.0);
            setTransferRPM(TRANSFER_FEED_RPM);
        } else {
            stop();
        }
    }

    public void setTransferRPM(double rpm) {
        double ticksPerSec = (rpm * TRANSFER_TICKS_PER_REV) / 60.0;
        transferMotor.setVelocity(ticksPerSec);
    }

    public double getTransferRPM() {
        return (transferMotor.getVelocity() / TRANSFER_TICKS_PER_REV) * 60.0;
    }

    public void stop() {
        intakeMotor.setPower(0.0);
        transferMotor.setVelocity(0.0);
        transferMotor.setPower(0.0);
    }
}
