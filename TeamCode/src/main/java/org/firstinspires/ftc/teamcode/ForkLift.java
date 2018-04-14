package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ForkLift {
    private Servo rightClaw;
    private Servo leftClaw;
    public DcMotor motor;
    private DigitalChannel topButton;
    private DigitalChannel bottomButton;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private final double CLAW_GRAB_POSITION = 0.525;
    private final double CLAW_PUSH_IN_BLOCK_POSITION = 0.85;
    private final double CLAW_OPEN_POSITION = 0; //.375 for intake
    private final double CLAW_OPEN_ALL_THE_WAY_POSITION = 0;
    private final double CLAW_CLOSE_POSITION = 1;
    public boolean isClosed = false;

    public ForkLift(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.rightClaw = hardwareMap.servo.get("s5");
        this.rightClaw.setDirection(Servo.Direction.REVERSE);
        this.leftClaw = hardwareMap.servo.get("s6");
        this.motor = hardwareMap.dcMotor.get("m6");
        this.topButton = hardwareMap.digitalChannel.get("b0");
        this.bottomButton = hardwareMap.digitalChannel.get("b1");
        this.telemetry = opMode.telemetry;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoder();
    }

    public void init() {
        openClaw();
        moveUntilDown();
    }
    public void autoInit() {
        openClaw();
        init();
        closeClaw();
        sleep(350);
        moveMotor(1, 200);
    }

    public void closeClaw() {
        setClawPosition(CLAW_GRAB_POSITION);
        isClosed = true;
    }

    public void openClaw() {
        setClawPosition(CLAW_OPEN_POSITION);
        isClosed = false;
    }

    public void setClawPositionPushInBlock() {
        setClawPosition(CLAW_PUSH_IN_BLOCK_POSITION);
        isClosed = true;
    }

    public void openAllTheWay() {
        setClawPosition(CLAW_OPEN_ALL_THE_WAY_POSITION);
        isClosed = false;
    }

    public void moveMotor(double speed) {
        if (speed < 0 && !bottomButton.getState()) {
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            speed = 0;
        }
        if (speed > 0 && !topButton.getState()) {
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            speed = 0;
        }
        if (!opMode.opModeIsActive()) {
            speed = 0;
        }
        motor.setPower(speed);
    }

    private void setClawPosition(double position) {
        rightClaw.setPosition(position);
        leftClaw.setPosition(position);
    }

    private void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveMotor(double speed, long miliseconds) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        while(runTime.milliseconds()<miliseconds){
            moveMotor(speed);
        }
        stop();
    }
    public void moveUntilDown(double speed) {
        while (bottomButton.getState()) {
            moveMotor(-Math.abs(speed));
            if(!topButton.getState()) {
              moveMotor(Math.abs(speed));
            }
        }
        stop();
    }
    public void moveUntilDown() {
        moveUntilDown(0.6);
    }
    public void moveUntilUp(double speed) {
        while (topButton.getState()) {
            moveMotor(Math.abs(speed));
        }
        stop();
    }
    public void moveUntilUp() {
        moveUntilUp(1);
    }
    public void  stop() {
        moveMotor(0);
    }

    private void sleep(long time) {try {Thread.sleep(time);} catch (InterruptedException e) {}}
    public void closeAllTheWay() {setClawPosition(CLAW_CLOSE_POSITION);}
}