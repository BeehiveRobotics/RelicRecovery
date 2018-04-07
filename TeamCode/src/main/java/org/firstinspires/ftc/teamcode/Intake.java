package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Kaden on 3/15/18.
 */

public class Intake {
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private CRServo rightServo;
    private CRServo leftServo;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private final double ON_SPEED = 1;
    private final double REVERSE_SPEED = -1;

    public boolean isOut = false;
    public boolean isOn = false;
    private double servoPower;

    public Intake(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        this.rightMotor = hardwareMap.dcMotor.get("m7");
        this.leftMotor = hardwareMap.dcMotor.get("m8");
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightServo = hardwareMap.crservo.get("s8");
        this.leftServo = hardwareMap.crservo.get("s9");
        this.leftServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void on() {
        if(isOut) {
            setMotorSpeed(1);
        }
        isOn = true;
    }

    public void off() {
        setMotorSpeed(0);
        isOn = false;
    }

    public void reverse() {
        if(isOut){setMotorSpeed(REVERSE_SPEED);}
    }

    public void out() {
        isOut = true;
        setServoPower(1);
        Robot.sleep(300);
        stopServos();
    }

    public void in() {
        isOut = false;
        setServoPower(-1);
        Robot.sleep(300);
        stopServos();
    }

    private void setMotorSpeed(double speed) {
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
    }

    private void stopServos() {
        setServoPower(0);
    }
    public void setServoPower(double power) { //SET TO PRIVATE ONCE STUFF IS SET
        power = Range.clip(power, -.85, .85);
        servoPower = power;
        rightServo.setPower(power);
        leftServo.setPower(power);
    }

    public double getServoPower() {
        return servoPower;
    }
}
