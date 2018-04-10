package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Left 90", group = "Autonomous")
public class leftTurn extends LinearOpMode {
    private Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("go, my hombre");
        telemetry.update();
        waitForStart();
        robot.calibrateGyro();
        sleep(1000);
        robot.rightGyro(1, -90);

    }

}
