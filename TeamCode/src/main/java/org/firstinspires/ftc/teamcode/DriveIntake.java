/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "vroom vroom", group = "test")
public class DriveIntake extends OpMode {
    private Robot robot;
    boolean xPrev = false;
    boolean yPrev = false;
    boolean aPrev = false;
    boolean bPrev = false;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.mapRobot();
    }

    @Override
    public void loop() {
        //drive
        if (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad2.left_stick_x) + Math.abs(gamepad2.left_stick_y) + Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y)) {
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                robot.drive.driveLeftRight(gamepad1.left_stick_x * robot.drive.BUMPER_SLOW_SPEED, gamepad1.right_stick_x * robot.drive.BUMPER_SLOW_SPEED, gamepad1.left_stick_y * robot.drive.BUMPER_SLOW_SPEED, gamepad1.right_stick_y * robot.drive.BUMPER_SLOW_SPEED);
            } else {
                robot.drive.driveLeftRight(gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y);
            }
        } else {
            if (gamepad1.dpad_up) {
                robot.drive.forward(robot.drive.D_PAD_SLOW_SPEED);
            }
            else if (gamepad1.dpad_left) {
                robot.drive.strafeLeft(robot.drive.D_PAD_SLOW_SPEED);
            }
            else if (gamepad1.dpad_down) {
                robot.drive.backward(robot.drive.D_PAD_SLOW_SPEED);
            }
            else if (gamepad1.dpad_right) {
                robot.drive.strafeRight(robot.drive.D_PAD_SLOW_SPEED);
            }
            else {
                robot.drive.stopMotors();
            }
        }
        if(gamepad1.a && !(aPrev)) {
            if(robot.forkLift.isClosed) {
                robot.forkLift.openClaw();
            }
            else {
                robot.forkLift.closeClaw();
            }
        }
        if(gamepad1.b && !(bPrev)) {
            if(robot.intake.isOn) {
                robot.intake.off();
            } else {
                robot.intake.on();
            }
        }
        if(gamepad1.x && !(xPrev)) {
            if(robot.intake.isOut) {
                robot.intake.in();
            } else {
                robot.intake.out();
            }
        }
        robot.forkLift.moveMotor(gamepad1.right_trigger - gamepad1.left_trigger);
        aPrev = gamepad1.a;
        bPrev = gamepad1.b;
        xPrev = gamepad1.x;
        yPrev = gamepad1.y;
        telemetry.addData("isOut", robot.intake.isOut);
        telemetry.update();
    }
}
*/