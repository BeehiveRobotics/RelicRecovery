package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Kaden on 10/20/2017.
 */

@TeleOp(name = "MecanumDrive2sticks", group = "linear OpMode")
public class Mecanum2Sticks extends OpMode {
    private DriveMecanum drive;
    @Override
    public void init() {
        drive = new DriveMecanum(
                hardwareMap.dcMotor.get("m1"), //FrontLeft
                hardwareMap.dcMotor.get("m2"), //FrontRight
                hardwareMap.dcMotor.get("m3"), //RearLeft
                hardwareMap.dcMotor.get("m4"), //RearRight
                1.0); //top speed as a decimal
    }
    @Override
    public void loop() {
        drive.driveLeftRight(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
