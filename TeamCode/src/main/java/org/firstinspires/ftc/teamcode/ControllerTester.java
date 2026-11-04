package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Controller Tester", group = "Diagnostics")
public class ControllerTester extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Status", "Ready to test controller");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Print individual stick axes (-1.0 to 1.0)
        telemetry.addData("Left Stick", "X: %.2f, Y: %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right Stick", "X: %.2f, Y: %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);

        // Print triggers (0.0 to 1.0)
        telemetry.addData("Triggers", "Left: %.2f, Right: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);

        // Check specific buttons (True/False)
        telemetry.addData("Buttons", "A: %b | B: %b | X: %b | Y: %b",
                gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);

        telemetry.addData("Bumpers", "Left: %b | Right: %b",
                gamepad1.left_bumper, gamepad1.right_bumper);

        telemetry.update();
    }
}