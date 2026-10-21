package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive;

@TeleOp(name = "Mecanum Field Orientated", group = "TeleOp")
public class MainTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    @Override
    public void init() {

        drive.init(hardwareMap);
    }

    /**
     * Runs repeatedly while this TeleOp is active. Reads the gamepad sticks as forward/backward,
     * left/right strafe, and rotation commands, then displays those values in Driver Station
     * telemetry. Passes the commands to {@link MecanumDrive#driveFieldRelative(double, double,
     * double)}, which uses the IMU yaw to make the movement field-relative and sets the four
     * wheel motor powers. The drive's motors and IMU are configured in {@link #init()}.
     */
    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        telemetry.addData("forward:", forward);
        telemetry.addData("strafe:", strafe);
        telemetry.addData("rotate:", rotate);
        telemetry.update();

        drive.driveFieldRelative(forward,strafe,rotate);
    }
}
