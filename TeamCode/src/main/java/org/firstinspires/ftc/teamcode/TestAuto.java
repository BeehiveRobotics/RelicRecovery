package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Test gyroGoTo", group="Test")
public class TestAuto extends LinearOpMode {
    private Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("READY TO START"); telemetry.update();
        waitForStart();
        robot.gyroGoTo(1, -90);
        robot.gyroGoTo(1, 90);

    }
}
