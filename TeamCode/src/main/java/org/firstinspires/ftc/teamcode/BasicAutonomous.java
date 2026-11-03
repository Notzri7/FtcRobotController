package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive;

@Autonomous(name = "Auto: Park in Zone", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final MecanumDrive drive = new MecanumDrive();
    private DcMotor[] motors;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        motors = drive.getDriveMotors();

        telemetry.addLine("Ready to park");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            encoderDrive(0.4, 24.0, 4.0);
        }
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        try {
            for (DcMotor motor : motors) {
                motor.setTargetPosition(motor.getCurrentPosition() + moveCounts);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            for (DcMotor motor : motors) {
                motor.setPower(Math.abs(speed));
            }

            while (opModeIsActive() && runtime.seconds() < timeoutS && anyMotorIsBusy()) {
                idle();
            }
        } finally {
            for (DcMotor motor : motors) {
                motor.setPower(0.0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    private boolean anyMotorIsBusy() {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }
}
