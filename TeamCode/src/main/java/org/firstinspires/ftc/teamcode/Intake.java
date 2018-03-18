package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by root on 3/15/18.
 */

public class Intake {
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private Servo rightServo;
    private Servo leftServo;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private final double ON_SPEED = 1;
    private final double REVERSE_SPEED = -1;

    private final double OUT_POSITION = 0.5;
    private final double IN_POSITION = 0.57;
    private double servoPosition;

    public boolean isOut = false;
    public boolean isOn = false;

    public Intake(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        this.rightMotor = hardwareMap.dcMotor.get("m7");
        this.leftMotor = hardwareMap.dcMotor.get("m8");
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightServo = hardwareMap.servo.get("s8");
        this.leftServo = hardwareMap.servo.get("s9");
        this.leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void on() {
        setMotorSpeed(1);
        isOn = true;
    }

    public void off() {
        setMotorSpeed(0);
        isOn = false;
    }

    public void reverse() {
        setMotorSpeed(REVERSE_SPEED);
    }

    public void out() {
        setServoPosition(OUT_POSITION);
        isOut = true;
    }

    public void in() {
        setServoPosition(IN_POSITION);
        isOut = false;
    }

    private void setMotorSpeed(double speed) {
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
    }

    public void setServoPosition(double position) { //SET TO PRIVATE ONCE STUFF IS SET
        this.servoPosition = Range.clip(position, 0, 1);
        rightServo.setPosition(position);
        leftServo.setPosition(position);
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
