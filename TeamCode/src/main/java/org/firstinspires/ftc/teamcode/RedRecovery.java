package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Point;

@Autonomous(name = "Red Recovery", group = "Autonomous")
public class RedRecovery extends LinearOpMode {
    private Robot robot;
    private RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;
    private final double MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY = 31;

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
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) pictograph = RelicRecoveryVuMark.CENTER;
        robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, robot.drive.DRIVE_OFF_BALANCE_BOARD_DISTANCE);
        switch (pictograph) {
            case LEFT:
                robot.drive.forward(robot.drive.MAX_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY - robot.drive.DRIVE_OFF_BALANCE_BOARD_DISTANCE);
                break;
            case CENTER:
                robot.drive.forward(robot.drive.MAX_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY - robot.drive.DRIVE_OFF_BALANCE_BOARD_DISTANCE);
                break;
            case RIGHT:
                robot.drive.forward(robot.drive.MAX_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY - robot.drive.DRIVE_OFF_BALANCE_BOARD_DISTANCE + 1);
                break;
        }
        robot.rightGyro(robot.drive.MAX_SPEED, -90);
        robot.forkLift.moveMotor(-1, 200);
        robot.phone.faceFront();
        robot.drive.forward(robot.drive.MAX_SPEED, 3);
        robot.forkLift.openClaw();
        robot.drive.forward(robot.drive.MAX_SPEED, 5);
        robot.drive.backward(robot.drive.MAX_SPEED , 6);
        robot.getMoreGlyphsRecovery(pictograph, runTime);
    }
}