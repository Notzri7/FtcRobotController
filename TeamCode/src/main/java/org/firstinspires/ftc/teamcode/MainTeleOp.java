package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Read raw joystick inputs
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Apply joystick deadbands to eliminate stick drift
        if (Math.abs(forward) < 0.05) {
            forward = 0.0;
        }
        if (Math.abs(strafe) < 0.05) {
            strafe = 0.0;
        }
        if (Math.abs(rotate) < 0.05) {
            rotate = 0.0;
        }

        // Precision / Slow Mode
        if (gamepad1.right_bumper) {
            forward *= 0.4;
            strafe *= 0.4;
        }

        if (gamepad1.back) {
            drive.resetYaw();
        }

        // Telemetry for Debugging & Tuning
        telemetry.addData("Actual Heading (deg)", drive.getYawDegrees());
        telemetry.addData("Target Heading (deg)", drive.getTargetHeadingDegrees());
        telemetry.addData("Heading Error (deg)", drive.getHeadingErrorDegrees());
        //telemetry.addData("Live kP Gain", drive.kP); // Shows current kP (useful if tuning via FTC Dashboard)
        telemetry.addData("Forward / Strafe / Rotate", "%.2f / %.2f / %.2f", forward, strafe, rotate);
        telemetry.update();

        //drive.drive(forward, strafe, rotate); // This is robot centric drive
        //drive.driveFieldRelativeWithHeadingHold(forward, strafe, rotate);
        drive.driveFieldRelative(forward, strafe, rotate);
    }
}
