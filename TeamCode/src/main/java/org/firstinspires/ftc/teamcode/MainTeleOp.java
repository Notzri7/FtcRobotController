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
    private static final double DEADBAND = 0.05;   // ~5% stick = dead zone
    private static final double SLOW_MODE_SCALE = 0.4; // 40% speed in precision

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
        // Zero out tiny stick values so the robot doesn't "crawl"
        // from stick drift. If you find the robot is still creeping,
        // raise DEADBAND to 0.08. If it feels "dead" near center,
        // lower it to 0.03.
        forward = applyDeadband(forward, DEADBAND);
        strafe  = applyDeadband(strafe,  DEADBAND);
        rotate  = applyDeadband(rotate,  DEADBAND);

        // Precision / Slow Mode
        if (gamepad1.right_bumper) {
            forward *= SLOW_MODE_SCALE;
            strafe  *= SLOW_MODE_SCALE;
        }

        if (gamepad1.back) {
            drive.resetYaw();
        }
        //drive.driveFieldRelativeWithHeadingHold(forward, strafe, rotate);
        //drive.drive(forward, strafe, rotate); // This is robot centric drive
        drive.driveFieldRelative(forward, strafe, rotate);

        //telemetry.addData("Target Heading (deg)", drive.getTargetHeadingDegrees());
        //telemetry.addData("Heading Error (deg)", drive.getHeadingErrorDegrees());
        //telemetry.addData("Live kP Gain", drive.kP); // Shows current kP (useful if tuning via FTC Dashboard)

        telemetry.addData("Yaw (deg)",        "%.1f", drive.getYawDegrees());
        telemetry.addData("Fwd / Strf / Rot", "%.2f / %.2f / %.2f", forward, strafe, rotate);
        telemetry.addData("Slow Mode",        gamepad1.right_bumper ? "ON" : "off");
        telemetry.update();

    }
    private double applyDeadband(double value, double threshold) {
        if (Math.abs(value) < threshold) {
            return 0.0;
        }
        double scaled = (Math.abs(value) - threshold) / (1.0 - threshold);
        // Cubing the scaled input creates a true exponential response curve
        return Math.pow(scaled, 2) * Math.signum(value);
    }
}

