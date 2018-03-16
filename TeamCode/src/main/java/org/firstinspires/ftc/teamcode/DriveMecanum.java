package org.firstinspires.ftc.teamcode;

/**
 * Created by Kaden on 11/22/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveMecanum {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearLeft;
    private DcMotor RearRight;
    private double speed = 1;
    private Telemetry telemetry;
    final double D_PAD_SLOW_SPEED = 0.25;
    final double BUMPER_SLOW_SPEED = 0.25;

    public DriveMecanum(HardwareMap hardwareMap, Telemetry telemetry) {
        this.FrontLeft = hardwareMap.dcMotor.get("m1");
        this.FrontRight = hardwareMap.dcMotor.get("m2");
        this.FrontRight.setDirection(DcMotor.Direction.REVERSE);
        this.RearLeft = hardwareMap.dcMotor.get("m3");
        this.RearRight = hardwareMap.dcMotor.get("m4");
        this.RearRight.setDirection(DcMotor.Direction.REVERSE);
        this.telemetry = telemetry;
        //setBRAKE();
        setFLOAT();
    }

    public void driveTranslateRotate(double x, double y, double z) {
        //for positive values: x - strafe right, y - backward, z - spin right
        //for negative values: x - strafe left, y - forward, z - spin left
        driveSpeeds(
                y + -x - z,
                y + x + z,
                y + x - z,
                y + -x + z);
    }
    public void forward(double speed) {
        driveTranslateRotate(0, -Math.abs(speed), 0);
    }

    public void backward(double speed) {
        driveTranslateRotate(0, Math.abs(speed), 0);
    }

    public void strafeRight(double speed) {
        driveTranslateRotate(Math.abs(speed), 0, 0);
    }

    public void strafeLeft(double speed) {
        driveTranslateRotate(-Math.abs(speed), 0, 0);
    }

    public void spinRight(double speed) {
        driveTranslateRotate(0, 0, Math.abs(speed));
    }

    public void spinLeft(double speed) {
        driveTranslateRotate(0, 0, -Math.abs(speed));

    }
    public void driveTranslateRotate(double x, double y, double z, long miliseconds) {
        driveTranslateRotate(x,y,z);
        sleep(miliseconds);
        stop();

    }
    public void driveLeftRight(double xLeft, double xRight, double yLeft, double yRight) {
        //This one is kinda complicated for using in an autonomous program or any linear set of commands. I'd recommend using driveTranslateRotate for things other than driving. However I will explain this one anyway.
        //There are two forces for each side (left/right) of the robot. y controls forward/backward, x controls side-to-side (strafing). For instance if both ys are positive it will move forward. If both xs are positive it will move right.
        driveSpeeds(yLeft - xLeft, yRight + xRight, yLeft + xLeft, yRight - xRight);
    }

    public void swingRight() {
        driveSpeeds(0, 0, -1, 1);
    }

    public void swingLeft() {
        driveSpeeds(0, 0, 1, -1);
    }

    public void driveSpeeds(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed) {
        FrontLeft.setPower(speed * clip(flSpeed));
        FrontRight.setPower(speed * clip(frSpeed));
        RearLeft.setPower(speed * clip(rlSpeed));
        RearRight.setPower(speed * clip(rrSpeed));
    }
    public void stop() {
        driveSpeeds(0,0,0,0);
    }

    public double clip(double value) {
        return Range.clip(value, -1, 1);
    }

    public void sleep(long miliseconds) {try {Thread.sleep(miliseconds);} catch (InterruptedException e) {}}

    private void setBRAKE() {
        this.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void setFLOAT() {
        this.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
}
