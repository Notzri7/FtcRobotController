package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive;

@Autonomous(name = "Basic Drive Autonomous", group = "Autonomous")
public class EvenMoreSimpleAutonomous extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();

    // Drive power (-1.0 to 1.0) for moving the robot forward a few inches for points.
    private static final double DRIVE_POWER = 0.25;

    // How long (in seconds) the robot drives forward; increase if short, decrease if too far.
    private static final double DRIVE_DURATION_SEC = 1.00;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        telemetry.addLine("Ready for Autonomous (Drive Only)");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("State", "Driving Forward");
            telemetry.addData("Power / Time", "%.2f / %.2fs", DRIVE_POWER, DRIVE_DURATION_SEC);
            telemetry.update();

            timeDrive(DRIVE_POWER, 0.0, 0.0, DRIVE_DURATION_SEC);

            telemetry.addData("State", "Done");
            telemetry.update();
        }
    }

    public void timeDrive(double forward, double strafe, double rotate, double durationSeconds) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < durationSeconds) {
            drive.drive(forward, strafe, rotate);
            idle();
        }

        drive.stop();
    }
}