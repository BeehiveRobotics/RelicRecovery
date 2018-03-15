package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by root on 3/15/18.
 */

public class Intake {
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private Servo rightServo;
    private Servo leftServo;

    private Telemetry telemetry;

    private final double ON_SPEED = 1;
    private final double REVERSE_SPEED = -1;

    private final double OUT_POSITION = 0;
    private final double IN_POSITION = 1;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.rightMotor = hardwareMap.dcMotor.get("m7");
        this.leftMotor = hardwareMap.dcMotor.get("m8");
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);

        this.rightServo = hardwareMap.servo.get("s8");
        this.leftServo = hardwareMap.servo.get("s9");
        this.leftServo.setDirection(Servo.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void on() {
        setMotorSpeed(1);
    }

    public void off() {
        setMotorSpeed(0);
    }

    public void reverse() {
        setMotorSpeed(REVERSE_SPEED);
    }

    public void out() {
        setServoPosition(OUT_POSITION);
    }

    public void in() {
        setServoPosition(IN_POSITION);
    }

    private void setMotorSpeed(double speed) {
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
    }

    private void setServoPosition(double position) {
        rightServo.setPosition(position);
        leftServo.setPosition(position);
    }

}
