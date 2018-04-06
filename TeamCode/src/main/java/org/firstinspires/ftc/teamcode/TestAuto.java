package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;

import java.util.Random;

/**
 * Created by Kaden on 3/27/18.
 */

@Autonomous(name="TestAuto", group="Test")
public class TestAuto extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("READY TO START");
        telemetry.update();
        waitForStart();
        robot.leftGyro(robot.drive.MAX_SPEED, -90);
    }
}
