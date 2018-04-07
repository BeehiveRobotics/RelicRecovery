package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.opencv.core.Point;

public class AutoGlyphs extends GlyphDetector {
    double xPos = DEFAULT_X_POS_VALUE;
    private static final double X_HIGH_POS = 384;
    private static final double X_CENTER = X_HIGH_POS / 2;
    private static final double Y_HIGH_POS = 216;
    private static final double Y_CENTER = Y_HIGH_POS / 2;
    private static final double X_POSITION_OFFSET = 28;
    static final double DEFAULT_X_POS_VALUE = 1000;

    public AutoGlyphs(OpMode opMode) {
        this(opMode, GlyphDetectionSpeed.VERY_FAST, 0);
    }

    public AutoGlyphs(OpMode opMode, GlyphDetectionSpeed speed, CameraDirection cameraDirection) {
        this(opMode, speed, cameraDirection.direction);
    }

    public AutoGlyphs(OpMode opMode, GlyphDetectionSpeed speed, int cameraDirection) {
        super();
        super.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), cameraDirection);
        this.minScore = 0.5;
        this.downScaleFactor = 0.3;
        this.speed = speed;
    }

    public void enable() {
        super.enable();
    }

    public void disable() {
        super.disable();
    }

    public double getXOffset() {
        return getXPos() + X_POSITION_OFFSET;
    }

    public double getXPos() {
        return super.getChosenGlyphPosition().y - Y_CENTER;
    }

    public double getYPos() {
        return super.getChosenGlyphPosition().x - X_CENTER;
    }

    public double getSize() {return super.getSize();}

    public Point getPoint() {
        return new Point(getXOffset(), getYPos());
    }
}
