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
    private final double MOVE_TOWARDS_CRYPTOBOX_DISTANCE_RED_RECOVERY = 32.5;

    public void runOpMode() throws InterruptedException {
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
        robot.forkLift.openClaw();
        robot.jewelArm.down();
        robot.jewelArm.middle();
        pictograph = robot.phone.getMark();
        robot.forkLift.moveUntilDown();
        robot.forkLift.closeClaw();
        robot.jewelArm.knockJewel(JewelColor.RED);
        robot.forkLift.moveMotor(1, 200);
        robot.jewelArm.up();
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) pictograph = RelicRecoveryVuMark.CENTER;
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
        robot.jewelArm.left();
        robot.rightGyro(robot.drive.SPIN_TO_CRYPTOBOX_SPEED, -90);
        robot.forkLift.moveMotor(-1, 200);
        robot.phone.faceFront();
        robot.drive.forward(robot.drive.MAX_SPEED, 3);
        robot.forkLift.openClaw();
        robot.drive.forward(robot.drive.MAX_SPEED, 3);
        robot.drive.backward(robot.drive.BACK_AWAY_FROM_BLOCK_SPEED, 6);
        robot.rightGyro(robot.drive.SPIN_TO_CENTER_SPEED, 90);
        robot.drive.backwardTime(robot.drive.MAX_SPEED, 200);
        switch (pictograph) {
            case LEFT:
                robot.drive.strafeLeft(robot.drive.MULTI_GLYPH_STRAFE_SPEED, robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY * robot.drive.STRAFING_DISTANCE_CONSTANT);
            case RIGHT:
                robot.drive.strafeRight(robot.drive.MULTI_GLYPH_STRAFE_SPEED, robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY * robot.drive.STRAFING_DISTANCE_CONSTANT);
        }
        robot.gyroGoTo(robot.drive.MAX_SPEED, 90);
        robot.setUpMultiGlyph();
        ElapsedTime findGlyphTime = new ElapsedTime();
        findGlyphTime.reset();
        double xOffSet = 0, yPos = 0, size = 0, bestGlyphSize = 0;
        Point bestGlyphPos = new Point(AutoGlyphs.DEFAULT_X_POS_VALUE, 0);
        while (findGlyphTime.seconds() < 3.5) {
            xOffSet = robot.glyphDetector.getXPos();
            yPos = robot.glyphDetector.getYPos();
            size = robot.glyphDetector.getSize();
            if ((xOffSet != AutoGlyphs.DEFAULT_X_POS_VALUE) && (size < 125) && (size > 60) && (yPos < 40) && (yPos > -170) && (Math.abs(xOffSet) < 70)) {
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
        if (bestGlyphPos.x == AutoGlyphs.DEFAULT_X_POS_VALUE) bestGlyphPos.x = 0;
        double distanceToStrafe = bestGlyphPos.x * robot.STRAFING_DAMPEN_FACTOR_FOR_MULTI_GLYPH + robot.phone.PHONE_DISTANCE_OFFSET;
        robot.strafeForMultiGlyph(distanceToStrafe);
        robot.drive.forward(robot.drive.DRIVE_INTO_GLYPH_PIT_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE);
        robot.drive.forward(robot.drive.DRIVE_INTO_GLYPHS_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE);
        robot.phone.faceSideways();
        robot.forkLift.closeClaw();
        sleep(300);
        robot.forkLift.moveMotor(1, 150);
        if (pictograph == RelicRecoveryVuMark.CENTER) robot.forkLift.moveMotor(1, 450);
        robot.drive.backward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE - 7);
        robot.leftGyro(robot.drive.MAX_SPEED, -90);
        robot.strafeForMultiGlyph(distanceToStrafe * 1.1 - 3);
        robot.drive.forward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE + 4);
        robot.drive.forwardTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 350);
        robot.drive.strafeLeftTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 300);
        robot.forkLift.openClaw();
        sleep(300);
        robot.drive.forwardTime(robot.drive.MAX_SPEED, 300);
        robot.drive.backward(robot.drive.MAX_SPEED, 5);
        //start second try
        if(runTime.seconds() > 23) return;
        robot.rightGyro(robot.drive.MAX_SPEED, 90);
        robot.forkLift.moveUntilDown();
        robot.gyroGoTo(robot.drive.MAX_SPEED, 90);
        robot.forkLift.openClaw();
        robot.drive.forward(robot.drive.DRIVE_INTO_GLYPH_PIT_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE);
        robot.drive.forward(robot.drive.DRIVE_INTO_GLYPHS_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE);
        robot.phone.faceSideways();
        robot.forkLift.closeClaw();
        sleep(300);
        robot.forkLift.moveMotor(1, 150);
        if (pictograph == RelicRecoveryVuMark.CENTER) robot.forkLift.moveMotor(1, 450);
        robot.drive.backward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE - 7);
        robot.leftGyro(robot.drive.MAX_SPEED, -90);
        robot.drive.forward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE + 4);
        robot.drive.forwardTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 350);
        robot.drive.strafeRightTime(robot.drive.DRIVE_INTO_GLYPHS_SPEED, 300);
        robot.forkLift.openClaw();
        sleep(300);
        robot.drive.forwardTime(robot.drive.MAX_SPEED, 300);
        robot.drive.backward(robot.drive.MAX_SPEED, 5);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}