package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        Random rand = new Random();
        double target;
        while(opModeIsActive()) {
            target = rand.nextInt(360) - 180;
            telemetry.addData("Target", target);
            telemetry.update();
            robot.rightGyro(robot.drive.MAX_SPEED, target);

        }
    }
}
