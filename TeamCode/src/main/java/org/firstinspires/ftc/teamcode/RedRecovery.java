package org.firstinspires.ftc.teamcode;

/**
 * Created by BeehiveRobotics-3648 on 11/28/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Point;

@Autonomous(name = "Red Recovery", group = "Autonomous")
public class RedRecovery extends LinearOpMode {
    private Robot robot;
    private RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;
    private final double MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY = 32.5;

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("DO NOT PRESS PLAY YET");
        telemetry.update();
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("NOW YOU CAN PRESS PLAY");
        telemetry.update();
        waitForStart();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        robot.forkLift.autoInit();
        robot.jewelArm.findJewel(Color.RED);
        pictograph = robot.phone.getMark();
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) {
            pictograph = RelicRecoveryVuMark.CENTER;
        }
        switch (pictograph) {
            case LEFT:
                robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
                break;
            case CENTER:
                robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY);
                break;
            case RIGHT:
                robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
                break;
        }
        robot.rightGyro(robot.drive.SPIN_TO_CRYPTOBOX_SPEED, -90);
        robot.forkLift.moveMotor(-1, 200);
        robot.drive.forward(robot.drive.DRIVE_INTO_CRYPTOBOX_SPEED, 5);
        robot.pushInBlock();
        robot.drive.backward(robot.drive.BACK_AWAY_FROM_BLOCK_SPEED, 8);
        robot.rightGyro(robot.drive.SPIN_TO_CENTER_SPEED, 90);
        robot.drive.backwardTime(robot.drive.MAX_SPEED, 500);
        switch (pictograph) {
            case LEFT:
                robot.drive.strafeLeft(robot.drive.MULTI_GLYPH_STRAFE_SPEED, robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY*robot.drive.STRAFING_DISTANCE_CONSTANT);
            case RIGHT:
                robot.drive.strafeRight(robot.drive.MULTI_GLYPH_STRAFE_SPEED, robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY*robot.drive.STRAFING_DISTANCE_CONSTANT);
        }
        robot.gyroGoTo(robot.drive.MAX_SPEED, 90);
        robot.setUpMultiGlyph();
        ElapsedTime findGlyphTime = new ElapsedTime();
        findGlyphTime.reset();
        double xOffSet = 0, yPos = 0, size = 0, bestGlyphSize = 0;
        Point bestGlyphPos = new Point(AutoGlyphs.DEFAULT_X_POS_VALUE, 0);
        sleep(750);
        while (findGlyphTime.seconds() < 3.5) {
            xOffSet = robot.glyphDetector.getXOffset();
            yPos = robot.glyphDetector.getYPos();
            size = robot.glyphDetector.getSize();
            if ((xOffSet != AutoGlyphs.DEFAULT_X_POS_VALUE) && (size < 125) && (size > 60) && (yPos < 40) && (yPos > -170)) {
                bestGlyphPos.x = xOffSet;
                bestGlyphPos.y = yPos;
                bestGlyphSize = size;
                break;
            }
        }
        if (findGlyphTime.seconds() <= 3.5) {
            telemetry.addData("Glyph Position", bestGlyphPos.toString());
            telemetry.addData("Size", bestGlyphSize);
        } else {
            telemetry.addData("Would be glyph position", xOffSet + ", " + yPos);
            telemetry.addData("Would be glyph size", size);
        }
        telemetry.update();
        robot.glyphDetector.disable();
        robot.forkLift.openClaw();
        if (bestGlyphPos.x == AutoGlyphs.DEFAULT_X_POS_VALUE) {
            bestGlyphPos.x = 0;
        }
        double distanceToStrafe = bestGlyphPos.x * robot.STRAFING_DAMPEN_FACTOR_FOR_MULTI_GLYPH;
        robot.strafeForMultiGlyph(distanceToStrafe);
        robot.drive.forward(robot.drive.DRIVE_INTO_GLYPH_PIT_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE);
        robot.drive.forward(robot.drive.DRIVE_INTO_GLYPHS_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE);
        robot.phone.faceSideways();
        robot.forkLift.closeClaw();
        sleep(300);
        robot.forkLift.moveMotor(1, 75);
        if (pictograph == RelicRecoveryVuMark.CENTER) {
            robot.forkLift.moveMotor(1, 450);
        }
        robot.drive.backward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE - 4);
        robot.leftGyro(robot.drive.MAX_SPEED, -90);
        robot.strafeForMultiGlyph(distanceToStrafe * 1.125 - 3);
        robot.drive.forward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE + 4);
        robot.drive.forwardTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 400);
        robot.drive.strafeLeftTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 350);
        robot.forkLift.openClaw();
        sleep(300);
        robot.drive.forwardTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 700);
        robot.drive.backward(robot.drive.MAX_SPEED, 5);
        robot.leftGyro(robot.drive.SPIN_TO_CENTER_SPEED, 90);
        robot.forkLift.openClaw();
        robot.forkLift.moveUntilDown();
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}