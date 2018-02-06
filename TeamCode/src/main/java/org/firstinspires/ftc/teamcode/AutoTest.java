package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by root on 2/5/18.
 */
@Autonomous(name = "testing expo", group = "test")
public class AutoTest extends LinearOpMode {
    private AutoDrive drive;
    public void runOpMode() {
        drive = new AutoDrive(hardwareMap, telemetry);
        waitForStart();
        drive.forward(1, 15);
    }
}
