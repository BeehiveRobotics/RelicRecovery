package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Red Far", group = "Autonomous")
public class RedFar extends LinearOpMode {
    private Robot robot;
    private RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;
    private double DISTANCE_TO_RIGHT_COLUMN = 20.5;
    private double DISTANCE_TO_CENTER_COLUMN = 27;
    private double DISTANCE_TO_LEFT_COLUMN = 34;
    private double OVERSHOOT_OFFSET = 1;
    public void runOpMode() {
        telemetry.addLine("DO NOT PRESS PLAY YET");
        telemetry.update();
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.phone.readyVuforia();
        robot.calibrateGyro();
        telemetry.addLine("NOW YOU CAN PRESS PLAY");
        telemetry.update();
        ElapsedTime runTime = new ElapsedTime();
        waitForStart();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        runTime.reset();
        robot.relicClaw.openClaw();
        robot.relicClaw.down();
        robot.forkLift.openClaw();
        robot.jewelArm.down();
        robot.jewelArm.middle();
        pictograph = robot.phone.getMark();
        robot.forkLift.moveUntilDown();
        robot.forkLift.closeClaw();
        robot.jewelArm.knockJewel(AllianceColor.RED);
        robot.forkLift.moveMotor(1, 200);
        robot.jewelArm.up();
        boolean isDistanceSane = true;
        //isDistanceSane = 12 < robot.getDistance() && robot.getDistance() < 19;
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) {
            pictograph = RelicRecoveryVuMark.LEFT;
        }
        robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, robot.drive.DRIVE_OFF_BALANCE_BOARD_DISTANCE);
        robot.drive.forward(robot.drive.MAX_SPEED, 5);
        switch (pictograph) {
            case LEFT:
                if (isDistanceSane) {
                    robot.driveUntilDistance(0.1, DISTANCE_TO_LEFT_COLUMN - OVERSHOOT_OFFSET);
                    break;
                } else {
                    robot.drive.strafeLeft(0.1, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_FAR + 3);
                    break;
                }
            case CENTER:
                if (isDistanceSane) {
                    robot.driveUntilDistance(0.1, DISTANCE_TO_CENTER_COLUMN - OVERSHOOT_OFFSET);
                    break;
                } else {
                    robot.drive.strafeLeft(0.1, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION + 2.5);
                    break;
                }
            case RIGHT:
                if (isDistanceSane) {
                    robot.driveUntilDistance(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, DISTANCE_TO_RIGHT_COLUMN - OVERSHOOT_OFFSET);
                    break;
                } else {
                    robot.drive.strafeLeft(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_FAR + 3);
                    break;
                }
        }
        robot.drive.forward(robot.drive.DRIVE_INTO_CRYPTOBOX_SPEED, 6);
        robot.pushInBlock();
        robot.drive.backward(robot.drive.MAX_SPEED, 7);
        robot.leftGyro(robot.drive.MAX_SPEED, 120);
        robot.forkLift.openClaw();
        robot.forkLift.moveUntilDown();
        sleep(1000);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}