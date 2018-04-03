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
        telemetry.addLine("DO NOT PRESS PLAY YET"); telemetry.update();
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("NOW YOU CAN PRESS PLAY"); telemetry.update();
        waitForStart();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        robot.forkLift.autoInit();
        //JewelArm.findJewel(Color.RED);
        //pictograph = phone.getMark();
        if(pictograph == RelicRecoveryVuMark.UNKNOWN) {
            pictograph = RelicRecoveryVuMark.RIGHT;
        }
        switch (pictograph) {
            case LEFT:
                robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
            case CENTER:
                robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY);
            case RIGHT:
                robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
            }
        robot.rightGyro(robot.drive.SPIN_TO_CRYPTOBOX_SPEED, -90);
        robot.forkLift.moveMotor(-1, 200);
        robot.drive.forward(robot.drive.DRIVE_INTO_CRYPTOBOX_SPEED, 5);
        robot.pushInBlock();
        robot.drive.backward(robot.drive.BACK_AWAY_FROM_BLOCK_SPEED, 6);
        robot.rightGyro(robot.drive.SPIN_TO_CENTER_SPEED, 90);
        robot.setUpMultiGlyph();
        ElapsedTime findGlyphTime = new ElapsedTime();
        findGlyphTime.reset();
        double xOffSet;
        double yPos;
        double decisionPoint = 0;
        double size;
        Point bestGlyphPos = new Point(AutoGlyphs.DEFAULT_X_POS_VALUE, 0);
        double bestGlyphSize = 0;
        while (findGlyphTime.seconds() < 3.5) {
            xOffSet = robot.glyphDetector.getXOffset();
            yPos = robot.glyphDetector.getYPos();
            size = robot.glyphDetector.getSize();
            if ((Math.abs(xOffSet) < Math.abs(bestGlyphPos.x)) && (xOffSet != AutoGlyphs.DEFAULT_X_POS_VALUE) && (size < 105) && (size > 40)) {// && (yPos < 60)) {
                bestGlyphPos.x = xOffSet;
                bestGlyphPos.y = yPos;
                bestGlyphSize = size;
                decisionPoint = findGlyphTime.seconds();
            }
        }
        telemetry.addData("Glyph Position", bestGlyphPos.toString());
        telemetry.addData("Decision made at", decisionPoint);
        telemetry.addData("Size", bestGlyphSize);
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
        sleep(2000);
        robot.phone.faceSideways();
        robot.forkLift.closeClaw();
        sleep(300);
        robot.forkLift.moveMotor(1, 50);
        if(pictograph == RelicRecoveryVuMark.CENTER) {
            robot.forkLift.moveMotor(1, 250);
        }
        robot.drive.backward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE);
        robot.leftGyro(robot.drive.MAX_SPEED, -90);
        switch (pictograph) {
            case LEFT:
                robot.strafeForMultiGlyph(distanceToStrafe + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
            case CENTER:
                robot.strafeForMultiGlyph(distanceToStrafe);
            case RIGHT:
                robot.strafeForMultiGlyph(distanceToStrafe - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
        }
        sleep(2000);
        robot.drive.forward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE);
        sleep(2000);
        robot.drive.forwardTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 400);
        sleep(2000);
        robot.drive.strafeLeftTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 400);
        sleep(2000);
        robot.forkLift.openClaw();
        sleep(300);
        robot.drive.forwardTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 200);
        robot.drive.backward(robot.drive.MAX_SPEED,5);
        stop();
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}
