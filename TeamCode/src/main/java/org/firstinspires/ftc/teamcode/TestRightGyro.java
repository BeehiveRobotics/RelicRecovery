package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestRightGyro", group = "test")
public class TestRightGyro extends LinearOpMode {
    private Robot robot;

    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("READY TO START");
        telemetry.update();
        waitForStart();
        robot.calibrateGyro();
        sleep(1000);
        robot.rightGyro(1, -90);
    }
}
