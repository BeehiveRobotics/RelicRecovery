package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue Far", group = "Autonomous")
public class BlueFar extends LinearOpMode {
    private Robot robot;
    private RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;
    private final double DISTANCE_OFFSET = 3;

    public void runOpMode() {
        telemetry.addLine("DO NOT PRESS PLAY YET");
        telemetry.update();
        robot = new Robot(this);
        robot.mapRobot();
        robot.calibrateGyro();
        telemetry.addLine("NOW YOU CAN PRESS PLAY");
        telemetry.update();
        waitForStart();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        robot.forkLift.autoInit();
        robot.jewelArm.knockJewel(AllianceColor.BLUE);
        pictograph = robot.phone.getMark();
        robot.drive.backward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, robot.drive.DRIVE_TO_CYRPTOBOX_DISTANCE_FAR + 2);
        boolean isDistanceSane = 12 < robot.getDistance() && robot.getDistance() < 19;
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) {
            pictograph = RelicRecoveryVuMark.RIGHT;
        }
        switch (pictograph) {
            case LEFT:
                if (isDistanceSane) {
                    robot.strafeUntilDistance(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DISTANCE_TO_CLOSE_COLUMN + DISTANCE_OFFSET);
                } else {
                    robot.drive.strafeLeft(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_FAR + 2);
                }
            case CENTER:
                if (isDistanceSane) {
                    robot.strafeUntilDistance(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DISTANCE_TO_CENTER_COLUMN + DISTANCE_OFFSET);
                } else {
                    robot.drive.strafeLeft(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION);
                }
            case RIGHT:
                if (isDistanceSane) {
                    robot.strafeUntilDistance(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DISTANCE_TO_FAR_COLUMN + DISTANCE_OFFSET);
                } else {
                    robot.drive.strafeLeft(robot.drive.STRAFING_PAST_CRYPTOBOX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_FAR - 3);
                }
        }
        robot.rightGyro(robot.drive.MAX_SPEED, -180);
        robot.forkLift.moveMotor(-1, 250);
        robot.drive.forward(robot.drive.DRIVE_INTO_CRYPTOBOX_SPEED, 3);
        robot.pushInBlock();
        robot.drive.backward(robot.drive.MAX_SPEED, 6);
        //drive.leftGyro(-drive.SPIN_TO_CENTER_SPEED, 30);
        robot.forkLift.openClaw();
        robot.forkLift.moveUntilDown();
        sleep(1000);
    }
}