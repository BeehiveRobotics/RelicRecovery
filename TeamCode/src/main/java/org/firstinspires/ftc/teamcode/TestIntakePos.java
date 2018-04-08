package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="test intake pos", group="Test")
public class TestIntakePos extends OpMode{
    private Robot robot;
    public void init() {
        robot = new Robot(this);
        robot.mapRobot();
    }
    public void loop() {
        if(gamepad1.a) {
            robot.intake.setServoPower(robot.intake.getServoPower() + 0.005);
        }
        if(gamepad1.b) {
            robot.intake.setServoPower(robot.intake.getServoPower() - 0.005);
        }
        telemetry.addData("Power", robot.intake.getServoPower());
        telemetry.update();
    }
}
