package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "test drive until distance", group = "Test")
public class TestDriveUntilDistance extends LinearOpMode {
    private Robot robot;

    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        waitForStart();
        robot.driveUntilDistance(0.2, 48);
    }
}

