package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//teleop mode to use if you only have 2 motors

/**
 * Created by BeehiveRobotics-3648 on 7/13/2017.
 */

@TeleOp(name = "1MotorDrive", group = "linear OpMode")
public class Drive1Motor extends OpMode {

    DcMotor Motor;
    double speed = 0;

    @Override


    public void init() {
        Motor = hardwareMap.dcMotor.get("m1");

    }

    @Override
    public void loop() {
        speed = 0;
        if (gamepad1.a){
            speed = 0.5;
        }
        else if (gamepad1.b){
            speed = -0.5;
        }
        Motor.setPower(speed);
    }
}