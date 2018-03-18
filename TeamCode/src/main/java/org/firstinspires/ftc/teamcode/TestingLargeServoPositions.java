package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Kaden on 3/17/18.
 */
@TeleOp(name="testing large servo positions", group="test")
public class TestingLargeServoPositions extends OpMode {
    private Intake intake;
    public void init() {
        intake = new Intake(this);
        intake.setServoPosition(0.5);
    }
    public void loop() {
        if(gamepad1.a) {
            intake.setServoPosition(intake.getServoPosition() + 0.0002);
        }
        if(gamepad1.b) {
            intake.setServoPosition(intake.getServoPosition() - 0.0002);
        }
        telemetry.addData("Servo position", intake.getServoPosition());
        telemetry.update();
    }
}
