package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import junit.framework.Test;

//4 motor teleop used by the Preseason robot made by Sam, Jonah, and Kennan.

/**
 * Created by BeehiveRobotics-3648 on 7/20/2017.
 */

@TeleOp(name = "Directdrive4motorteleop", group = "linear OpMode")
public class DirectDrive4MotorTeleOp extends OpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor RearLeft;
    DcMotor RearRight;
    DcMotor OptionalMotor;
    boolean OptionalMotorExists = false;
    double TestVariable;
    @Override


    public void init() {
        FrontLeft = hardwareMap.dcMotor.get("m1");
        FrontRight = hardwareMap.dcMotor.get("m2");
        RearLeft = hardwareMap.dcMotor.get("m3");
        RearRight = hardwareMap.dcMotor.get("m4");
        try {
            OptionalMotor = hardwareMap.dcMotor.get("m5");
            OptionalMotorExists = true;
        } catch (IllegalArgumentException e){
            OptionalMotorExists = false;
        }



        reverseMotor(FrontRight);
        reverseMotor(RearRight);


    }

    @Override
    public void loop() {

        double right = gamepad1.right_stick_y;
        double left = gamepad1.left_stick_y;
        double OptionalMotorSpeed = gamepad1.right_trigger;
        FrontRight.setPower(right);
        FrontLeft.setPower(left);
        RearRight.setPower(right);
        RearLeft.setPower(left);
        if (OptionalMotorExists){
            OptionalMotor.setPower(-OptionalMotorSpeed);
            TestVariable = OptionalMotorSpeed;
        }


        telemetry.addData("OptionalMotorSpeed: ", TestVariable);
        telemetry.update();
    }

    public void reverseMotor(DcMotor motor) {

        motor.setDirection(DcMotor.Direction.REVERSE);
    }
// TODO: When the Drive interface and OpenLoop classes are for sure working, add that implementation here

}