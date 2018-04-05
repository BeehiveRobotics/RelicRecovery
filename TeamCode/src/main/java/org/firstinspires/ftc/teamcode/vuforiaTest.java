package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class vuforiaTest extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.phone.readyVuforia();
        waitForStart();
        RelicRecoveryVuMark pictograph = robot.phone.getMark();
        telemetry.addData("pictograph", pictograph);
        telemetry.update();
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
