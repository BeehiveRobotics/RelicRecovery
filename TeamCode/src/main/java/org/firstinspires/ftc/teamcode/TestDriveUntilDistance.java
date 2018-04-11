package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test range sensor values", group = "Test")
public class TestDriveUntilDistance extends LinearOpMode {
    private Robot robot;

    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance", robot.getDistance());
            telemetry.update(); //20.5 is right, 27 is mid, 34 is right
        }
    }
}

