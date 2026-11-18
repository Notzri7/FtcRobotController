package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private  DcMotor transferMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runIntake(boolean intakeButton, boolean ejectButton) {
        //If items spin backward or spit out when intaking,
        //change the power values to negative or reverse the motor direction in init.
        if (intakeButton) {
            intakeMotor.setPower(1.0); // Run intake forward
            transferMotor.setPower(0.8); // Run transfer forward
        } else if (ejectButton) {
            intakeMotor.setPower(-1.0); // Run intake backward
            transferMotor.setPower(-0.8); // Run transfer backward
        } else {
            intakeMotor.setPower(0.0); // Stop intake
            transferMotor.setPower(0.0); // Stop transfer
        }
    }

    public void setTransferPower(double power) {
        transferMotor.setPower(power);
    }
}
