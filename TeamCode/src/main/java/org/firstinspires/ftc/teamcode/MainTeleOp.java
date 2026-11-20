package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

@TeleOp(name = "Mecanum Field Orientated", group = "TeleOp")
public class MainTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    double forward, strafe, rotate;

    // Raise if joysticks drift on their own; lower if small stick pushes feel sluggish.
    private static final double DEADBAND = 0.05;   // ~5% stick = dead zone

    // Speed multiplier (0.0 to 1.0) for the four drive wheels while holding right bumper.
    private static final double SLOW_MODE_SCALE = 0.4; // 40% speed in precision

    // How far you must pull the left or right trigger (0.0 to 1.0) to activate eject or feed.
    private static final double TRIGGER_THRESHOLD = 0.1;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);

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
        drive.drive(forward, strafe, rotate); // This is robot centric drive
        //drive.driveFieldRelative(forward, strafe, rotate);

        boolean intakeOnlyBtn = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean transferOnlyBtn = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean bothBtn = gamepad1.left_bumper;
        intake.update(intakeOnlyBtn, transferOnlyBtn, bothBtn);
        // X = 2000 RPM flywheel preset, B = 2300 RPM preset, A = stop flywheel.
        shooter.runShooter(gamepad1.x, gamepad1.b, gamepad1.a);

        // Add target and actual RPM to your telemetry screen to check performance live:
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Actual RPM", "%.0f", shooter.getCurrentRPM());

        //telemetry.addData("Target Heading (deg)", drive.getTargetHeadingDegrees());
        //telemetry.addData("Heading Error (deg)", drive.getHeadingErrorDegrees());
        //telemetry.addData("Live kP Gain", drive.kP); // Shows current kP (useful if tuning via FTC Dashboard)

        telemetry.addData("Yaw (deg)",        "%.1f", drive.getYawDegrees());
        telemetry.addData("Fwd / Strf / Rot", "%.2f / %.2f / %.2f", forward, strafe, rotate);
        telemetry.addData("Battery Voltage",   "%.1fV", shooter.getBatteryVoltage());
        telemetry.addData("Slow Mode",        gamepad1.right_bumper ? "ON" : "off");
        telemetry.update();

    }
    private double applyDeadband(double value, double threshold) {
        if (Math.abs(value) < threshold) {
            return 0.0;
        }
        double scaled = (Math.abs(value) - threshold) / (1.0 - threshold);
        // squaring the scaled input creates a true exponential response curve
        return Math.pow(scaled, 2) * Math.signum(value);
    }
}

