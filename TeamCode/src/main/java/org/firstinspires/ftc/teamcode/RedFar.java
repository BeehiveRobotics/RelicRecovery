package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Point;

@Autonomous(name = "Red Far", group = "Autonomous")
public class RedFar extends LinearOpMode {
    private Robot robot;
    private RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;
    private double DISTANCE_TO_RIGHT_COLUMN = 20.5;
    private double DISTANCE_TO_CENTER_COLUMN = 27;
    private double DISTANCE_TO_LEFT_COLUMN = 34;
    private double OVERSHOOT_OFFSET = 3;
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
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) {
            pictograph = RelicRecoveryVuMark.LEFT;
        }
        robot.drive.forward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, robot.drive.DRIVE_OFF_BALANCE_BOARD_DISTANCE);
        robot.drive.forward(robot.drive.MAX_SPEED, 5);
        robot.gyroGoTo(robot.drive.HALF_SPEED, 0);
        switch (pictograph) {
            case LEFT:
                if (isDistanceSane) {
                    robot.driveUntilDistance(robot.drive.MAX_SPEED, DISTANCE_TO_LEFT_COLUMN - OVERSHOOT_OFFSET - 0.5 );
                    break;
                } else {
                    robot.drive.strafeLeft(robot.drive.MAX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_FAR + 3);
                    break;
                }
            case CENTER:
                if (isDistanceSane) {
                    robot.driveUntilDistance(robot.drive.MAX_SPEED, DISTANCE_TO_CENTER_COLUMN - OVERSHOOT_OFFSET);
                    break;
                } else {
                    robot.drive.strafeLeft(robot.drive.MAX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION + 2.5);
                    break;
                }
            case RIGHT:
                if (isDistanceSane) {
                    robot.driveUntilDistance(robot.drive.MAX_SPEED, DISTANCE_TO_RIGHT_COLUMN - OVERSHOOT_OFFSET + 0.5);
                    break;
                } else {
                    robot.drive.strafeLeft(robot.drive.MAX_SPEED, robot.drive.DEFAULT_MOVING_TOWARDS_CRYPTOBOX_DISTANCE_FAR_POSITION - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_FAR + 3);
                    break;
                }
        }
        robot.gyroGoTo(robot.drive.HALF_SPEED, 0);
        robot.drive.forward(robot.drive.DRIVE_INTO_CRYPTOBOX_SPEED, 2);
        robot.forkLift.moveMotor(-1, 200);
        robot.pushInBlock();
        robot.drive.backward(robot.drive.MAX_SPEED, 4);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        switch(pictograph) {
            case RIGHT:
                robot.drive.strafeLeft(robot.drive.MAX_SPEED, 2 * robot.drive.CRYPTOBOX_COLUMNS_OFFSET + 2);
                break;
            case CENTER:
                robot.drive.strafeLeft(robot.drive.MAX_SPEED, robot.drive.CRYPTOBOX_COLUMNS_OFFSET + 2);
                break;
            case LEFT:
                robot.drive.strafeLeft(robot.drive.MAX_SPEED, 2);
        }
        robot.forkLift.closeAllTheWay();
        robot.phone.faceFront();
        robot.leftGyro(robot.drive.MAX_SPEED, 165);
        robot.setUpMultiGlyph();
        ElapsedTime findGlyphTime = new ElapsedTime();
        findGlyphTime.reset();
        double xPos = 0, yPos = 0, size = 0, bestGlyphSize = 0, distanceToStrafe = 0;
        Point bestGlyphPos = new Point(AutoGlyphs.DEFAULT_X_POS_VALUE, 0);
        while (findGlyphTime.seconds() < 3.5) {
            xPos = robot.glyphDetector.getXPos();
            yPos = robot.glyphDetector.getYPos();
            size = robot.glyphDetector.getSize();
            if ((xPos != AutoGlyphs.DEFAULT_X_POS_VALUE) && (size < 125) && (size > 60) && (yPos < 40) && (yPos > -170) && (Math.abs(xPos) < 70)) {
                bestGlyphPos.x = xPos;
                bestGlyphPos.y = yPos;
                bestGlyphSize = size;
                distanceToStrafe = (bestGlyphPos.x * robot.STRAFING_DAMPEN_FACTOR_FOR_MULTI_GLYPH) + robot.phone.PHONE_DISTANCE_OFFSET;
                break;
            }
        }
        if (distanceToStrafe == 0) {
            telemetry.addData("Would be glyph position", xPos + ", " + yPos);
            telemetry.addData("Would be glyph size", size);
        } else {
            telemetry.addData("Glyph Position", bestGlyphPos.toString());
            telemetry.addData("Size", bestGlyphSize);
            telemetry.addData("Found at", findGlyphTime.seconds());
        }
        telemetry.update();
        robot.glyphDetector.disable();
        robot.forkLift.openClaw();
        if (bestGlyphPos.x == AutoGlyphs.DEFAULT_X_POS_VALUE) bestGlyphPos.x = 0;
        robot.drive.forward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE);
        robot.strafeForMultiGlyph(distanceToStrafe);
        robot.phone.faceSideways();
        robot.drive.forward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPHS_DISTANCE);
        robot.forkLift.closeClaw();
        sleep(225);
        robot.forkLift.moveMotor(1, 550);
        robot.strafeForMultiGlyph(-distanceToStrafe);
        robot.drive.backward(robot.drive.MAX_SPEED, robot.drive.DRIVE_INTO_GLYPH_PIT_DISTANCE + robot.drive.DRIVE_INTO_GLYPHS_DISTANCE);
        robot.rightGyro(robot.drive.MAX_SPEED, -15);
        robot.drive.forward(robot.drive.MAX_SPEED, 5);
        robot.forkLift.openClaw();
        robot.drive.forwardTime(robot.drive.MAX_SPEED, 250);
        robot.drive.backward(robot.drive.MAX_SPEED, 5);
        robot.leftGyro(robot.drive.MAX_SPEED, 165);

    }
}