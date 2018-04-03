package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;

import java.util.Random;

/**
 * Created by Kaden on 3/27/18.
 */

@Autonomous(name="TestAuto", group="Test")
public class TestAuto extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.mapRobot();
        robot.drive.setBRAKE();
        robot.calibrateGyro();
        telemetry.addLine("READY TO START");
        telemetry.update();
        waitForStart();


        robot.setUpMultiGlyph(GlyphDetector.GlyphDetectionSpeed.VERY_SLOW);
        ElapsedTime findGlyphTime = new ElapsedTime();
        findGlyphTime.reset();
        double xOffSet;
        double yPos;
        double decisionPoint = 0;
        double size;
        Point bestGlyphPos = new Point(AutoGlyphs.DEFAULT_X_POS_VALUE, 0);
        double bestGlyphSize = 0;
        while (findGlyphTime.seconds() < 20) {
            xOffSet = robot.glyphDetector.getXOffset();
            yPos = robot.glyphDetector.getYPos();
            size = robot.glyphDetector.getSize();
            telemetry.addData("Glyph Position", "{"+xOffSet+", " +yPos+"}");
            telemetry.addData("Glyph Size", size);
            telemetry.update();
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
    }
}
