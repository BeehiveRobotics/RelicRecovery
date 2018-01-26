package org.firstinspires.ftc.teamcode;

/**
 *Created by Kaden on 11/25/2017.
 **/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoDrive {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearLeft;
    private DcMotor RearRight;
    private REVGyro imu;
    private final double CIRCUMFERENCE_Of_WHEELS = 3.937 * Math.PI;
    private final int CPR = 1120; //Clicks per rotation of the encoder with the NeveRest 40 motors. Please do not edit.
    double heading;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    final double MIN_SPEED = 0.15;
    final double SPIN_ON_BALANCE_BOARD_SPEED = 0.15;
    final double SPIN_ON_BALANCE_BOARD_DISTANCE = 3;
    final double DRIVE_OFF_BALANCE_BOARD_SPEED = 0.4;
    final double STRAFING_PAST_CRYPTOBOX_SPEED = 0.6;
    final double SPIN_TO_CRYPTOBOX_SPEED = 0.5;
    final double DRIVE_INTO_CRYPTOBOX_SPEED = 0.4;
    final double DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_RECOVERY_POSITION = 34.5;
    final double DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION = 15.5;
    final double CYRPTOBOX_COLUMNS_OFFSET = 7.5;
    final double BACK_AWAY_FROM_BLOCK_SPEED = -0.75;
    final double SPIN_TO_CENTER_SPEED = 0.75;
    final double DRIVE_TO_CYRPTOBOX_DISTANCE_FAR = 24;
    final double FIND_VUMARK_DISTANCE = 2;
    final double TRY_AGAIN_JEWEL_DISTANCE = 0.6;

    public AutoDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.FrontLeft = hardwareMap.dcMotor.get("m1");
        this.FrontRight = hardwareMap.dcMotor.get("m2");
        this.FrontRight.setDirection(DcMotor.Direction.REVERSE);
        this.RearLeft = hardwareMap.dcMotor.get("m3");
        this.RearRight = hardwareMap.dcMotor.get("m4");
        this.RearRight.setDirection(DcMotor.Direction.REVERSE);
        this.imu = new REVGyro(hardwareMap.get(BNO055IMU.class, "imu"));
        this.telemetry = telemetry;
        setBRAKE();
    }
    public void driveTranslateRotate(double x, double y, double z, double distance) {
        resetEncoders();
        double clicks = Math.abs(distance * CPR / CIRCUMFERENCE_Of_WHEELS);
        double fl = clip(-y + -x - z);
        double fr = clip(-y + x + z);
        double rl = clip(-y + x - z);
        double rr = clip(-y + -x + z);
        double flSpeed = fl;
        double frSpeed = fr;
        double rlSpeed = rl;
        double rrSpeed = rr;
        double[] list = {fl, fr, rl, rr};
        double highestSpeed = findHigh(list);
        double flTarget = Math.abs(fl/highestSpeed*clicks);
        double frTarget = Math.abs(fr/highestSpeed*clicks);
        double rlTarget = Math.abs(rl/highestSpeed*clicks);
        double rrTarget = Math.abs(rr/highestSpeed*clicks);
        driveSpeeds(fl, fr, rl, rr);
        ElapsedTime time = new ElapsedTime();
        time.start();
        while (!(isMotorAtTarget(FrontLeft, flTarget)) && (!(isMotorAtTarget(FrontRight, frTarget))) && (!(isMotorAtTarget(RearLeft, rlTarget))) && (!(isMotorAtTarget(RearRight, rrTarget))) && time.getElapsedTime() <= 1.2*clicks/420/highestSpeed*100){
            driveSpeeds(calculateSpeed(FrontLeft, flTarget, fl), calculateSpeed(FrontRight, frTarget, fr), calculateSpeed(RearLeft, rlTarget, rl), calculateSpeed(RearRight, rrTarget, rr));
            telemetrizeSpeeds();
            telemetry.update();
        }
        stopMotors();
    }
    public void forward(double speed, double distance) {
        driveTranslateRotate(0, Math.abs(speed), 0, distance);
    }
    public void backward(double speed, double distance) {
        driveTranslateRotate(0, -Math.abs(speed), 0, distance);
    }
    public void strafeRight(double speed, double distance) {
        driveTranslateRotate(Math.abs(speed), 0, 0, distance);
    }
    public void strafeLeft(double speed, double distance) {
        driveTranslateRotate(-Math.abs(speed), 0, 0, distance);
    }
    public void spinRight(double speed, double distance) {
        driveTranslateRotate(0, 0, Math.abs(speed), distance);
    }
    public void spinLeft(double speed, double distance) {
        driveTranslateRotate(0,0, -Math.abs(speed), distance);
    }

    private void driveSpeeds(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed) {
        FrontLeft.setPower(clip(flSpeed));
        FrontRight.setPower(clip(frSpeed));
        RearLeft.setPower(clip(rlSpeed));
        RearRight.setPower(clip(rrSpeed));
    }

    private double clip(double value) {
        return Range.clip(value, -1, 1);
    }

    private double findHigh(double[] nums) {
        double high = 0;
        for (int i=0; i < nums.length; i++) {
            if (Math.abs(nums[i]) > high) {
                high = Math.abs(nums[i]);
            }
        }
        return high;
    }

    private void resetEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopMotors() {
        driveSpeeds(0, 0, 0, 0);
    }

    private int getCurrentPosition(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    private boolean isMotorAtTarget(DcMotor motor, double target) {return Math.abs(getCurrentPosition(motor)) >= Math.abs(target);}

    public void rightGyro(double x, double y, double z, double target) {
        heading = getHeading();
        double derivative = 0;
        double fl = clip(-y + -x - z);
        double fr = clip(-y + x + z);
        double rl = clip(-y + x - z);
        double rr = clip(-y + -x + z);
        driveSpeeds(fl, fr, rl, rr);
        if(heading < target) {
            while(derivative <= 0) {
                derivative = getHeading() - heading;
                heading = getHeading();
            }
        }
        while(heading >= target) {
            heading = getHeading();
        }
        stopMotors();
    }

    public void leftGyro(double x, double y, double z, double target) {
        heading = getHeading();
        double derivative = 0;
        double fl = clip(-y + -x - z);
        double fr = clip(-y + x + z);
        double rl = clip(-y + x - z);
        double rr = clip(-y + -x + z);
        driveSpeeds(fl, fr, rl, rr);
        if (target < heading) {
          while(derivative >= 0) {
            derivative = getHeading() - heading;
            heading = getHeading();
          }
        }
        while(heading <= target) {
            heading = getHeading();
        }
        stopMotors();
    }

    public void init() {
        imu.calibrate();
        heading = getHeading();
    }
    public double getHeading() {
        return imu.getHeading();
    }

    private void telemetrizeGyro() {
        telemetry.addData("Current heading: ", getHeading());
        telemetry.update();
    }
    private void telemetrizeEncoders() {
        telemetry.addData("First motor: ", FrontLeft.getCurrentPosition());
        telemetry.addData("Second motor: ", FrontRight.getCurrentPosition());
        telemetry.addData("third motor: ", RearLeft.getCurrentPosition());
        telemetry.addData("fourth motor: ", RearRight.getCurrentPosition());
    }
    private void telemetrizeSpeeds() {
        telemetry.addLine("Motor speeds:");
        telemetry.addData("Front left motor", FrontLeft.getPower());
        telemetry.addData("Front right motor", FrontRight.getPower());
        telemetry.addData("Back left motor", RearLeft.getPower());
        telemetry.addData("Back right motor", RearRight.getPower());
    }

    private void setBRAKE() {
        this.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private double calculateSpeed(DcMotor motor, double targetClicks, double defaultSpeed) {
        if (defaultSpeed>0) {
            return Range.clip(defaultSpeed*(targetClicks-Math.abs(motor.getCurrentPosition()))/targetClicks, MIN_SPEED, 1);
        }
        else if (defaultSpeed<0) {
            return Range.clip(defaultSpeed*(targetClicks-Math.abs(motor.getCurrentPosition()))/targetClicks, -1, -MIN_SPEED);
        }
        else {
            return 0;
        }
    }
}
