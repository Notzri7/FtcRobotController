package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {


    // if robot drives too far or too short adjust WHEEL_DIAMETER_INCHES slightly
    //up if short, down if it overshoots
    //to account for foam tile slippage and tire wear
    static final double COUNTS_PER_MOTOR_REV = 537.7; //switch this depending on the motor gear ratio
    static final double DRIVE_GEAR_REDUCTION = 1.0; // set greater if any external gear ratio

    //change this depending on the wheel diameter
    static final double WHEEL_DIAMETER_INCHES = 3.77953;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private final MecanumDrive drive = new MecanumDrive();
    private DcMotor[] motors;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        motors = drive.getDriveMotors();

        telemetry.addLine("Ready for Autonomous");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Speed: increase if too slow, decrease if wheels slip at launch
            // Inches: Change this to match exact inches from distance to parking zone
            // Timeout: Safety stop. if the robot takes longer than 4 seconds it forces movement to stop
            encoderDrive(0.4, 24.0, 4.0);
        }
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        try {
            for (DcMotor motor : motors) {
                // take the current position and add/subtract from target ticks
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
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
        }

        private boolean anyMotorIsBusy() {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                //If one wheel fights backward or spins the wrong way,
                // motor.isBusy() might stay true forever because that motor's encoder wire
                // is unplugged, reversed in configuration, or fighting a mechanical bind
                return true;
            }
        }
        return false;
    }
}
