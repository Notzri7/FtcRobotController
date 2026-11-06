package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.config.Config;


@Config
public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    // Controls how aggressively the robot snaps back to its target heading.
    // Increase if it's sluggish to correct; decrease if it oscillates back and forth rapidly.
    // Making this 'public static' allows live tuning via FTC Dashboard.
    //public static double kP = 0.5;
    //private double targetHeading = 0.0;
    //private double currentHeadingError = 0.0;
    //private boolean wasTurning = false;
    /**
     * Maps the configured drivetrain motors and IMU, applies motor directions and run modes,
     * and initializes the hub orientation used for yaw measurements.
     */
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // RUN_USING_ENCODER enables internal velocity regulation for consistent speed
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // BRAKE mode stops the robot instantly when joysticks are released rather than coasting
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(IMU.class, "imu");

        // Define hub orientation relative to the robot chassis
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

        this.resetYaw();
        //this.targetHeading = getYawRadians();
    }

    /**
     * Sets the current IMU yaw reading as the new zero heading.
     */
    public void resetYaw() {
        imu.resetYaw();
        //this.targetHeading = 0.0;
    }

    public double getYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getYawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    //public double getTargetHeadingDegrees() {
        //return Math.toDegrees(targetHeading);
    //}

    //public double getHeadingErrorDegrees() {
    //    return Math.toDegrees(currentHeadingError);
    //}
    /**
     * Provides an object-array accessor to the drive motors owned by this drivetrain.
     * Autonomous code can borrow these private hardware references without retrieving
     * the same configured devices again or duplicating drivetrain initialization.
     *
     * @return the front-left, back-left, front-right, and back-right motor references
     */
    public DcMotor[] getDriveMotors() {
        return new DcMotor[] {
                frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor
        };
    }

    /**
     * Use this method directly to test robot-centric driving.
     *
     * Mixes forward, strafe, and rotation commands into individual mecanum wheel powers,
     * then scales them proportionally if any individual wheel exceeds magnitude 1.0.
     */
    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxMagnitude = Math.max(1.0,
                Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        double SpeedCap = 0.7;

        frontLeftMotor.setPower((frontLeftPower  / maxMagnitude) * SpeedCap);
        backLeftMotor.setPower((backLeftPower   / maxMagnitude) * SpeedCap);
        frontRightMotor.setPower((frontRightPower / maxMagnitude) * SpeedCap);
        backRightMotor.setPower((backRightPower  / maxMagnitude) * SpeedCap);
    }

    /**
     * Field-Centric drive with Heading Hold.
     * Allows driving relative to the alliance wall while automatically locking heading when not turning.
     */

    /**
    public void driveFieldRelativeWithHeadingHold(double forward, double strafe, double rotate) {
        double botHeading = getYawRadians();

        double angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        // If the driver's rotation stick is pushed past 5%, manual turning takes over.
        // If stick drift causes the heading hold to disengage unintentionally,
        // increase this slightly (e.g., 0.08).
        if (Math.abs(rotate) > 0.05 || Math.abs(angularVelocity) > 0.4) {
            targetHeading = botHeading;
            wasTurning = true;
            currentHeadingError = 0.0;
        } else {
            if (wasTurning) {
                targetHeading = botHeading;
                wasTurning = false;
            }

            // Calculate raw heading error and normalize it to the range [-pi, pi] using atan2
            double error = targetHeading - botHeading;
            currentHeadingError = Math.atan2(Math.sin(error), Math.cos(error));

            if (Math.abs(currentHeadingError) < Math.toRadians(1.0)) {
                currentHeadingError = 0.0;
            }

            rotate = kP * currentHeadingError;

            // Caps maximum automatic rotational correction at 50% power (-0.5 to 0.5).
            // Prevents the heading hold from aggressively overpowering your translation controls.
            rotate = Math.max(-0.5, Math.min(0.5, rotate));
        }

        // Apply a 2D Rotation Matrix to transform field-relative commands
        // into robot-relative coordinates
        double cosAngle = Math.cos(-botHeading);
        double sinAngle = Math.sin(-botHeading);

        double rotStrafe = strafe * cosAngle - forward * sinAngle;
        double rotForward = strafe * sinAngle + forward * cosAngle;

        this.drive(rotForward, rotStrafe, rotate);
    }
}

     */

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double botHeading = getYawRadians();

        // Apply a 2D Rotation Matrix to transform field-relative commands
        // into robot-relative coordinates
        double cosAngle = Math.cos(-botHeading);
        double sinAngle = Math.sin(-botHeading);

        double rotStrafe = strafe * cosAngle - forward * sinAngle;
        double rotForward = strafe * sinAngle + forward * cosAngle;

        this.drive(rotForward, rotStrafe, rotate);
    }
}