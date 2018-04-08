package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;

import java.util.Random;

/**
 * Created by Kaden on 3/27/18.
 */

@Disabled
@TeleOp(name="Test potentiometer", group="Test")
public class TestAuto extends OpMode {
    Robot robot;
    AnalogInput potentiometer;
    @Override
    public void init() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("READY TO START");
        telemetry.update();
        potentiometer = hardwareMap.analogInput.get("p1");

    }
    public void loop() {
        telemetry.addData("Voltage", potentiometer.getVoltage());
        telemetry.update();
    }
}
