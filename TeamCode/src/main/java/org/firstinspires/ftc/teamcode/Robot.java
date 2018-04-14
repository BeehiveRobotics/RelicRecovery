package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Point;

public class Robot {
    private static final double SLOW_OFFSET = 10;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public Drive drive;
    public AutoGlyphs glyphDetector;
    public ForkLift forkLift;
    public JewelArm jewelArm;
    public RelicClaw relicClaw;
    public Phone phone;
    private REVGyro imu;
    private double heading;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private static Robot instance;

    static final double STRAFING_DAMPEN_FACTOR_FOR_MULTI_GLYPH = 0.1;
    static final double GYRO_OFFSET = 2.75; //When the one min speed is .75, GYRO_OFFSET shouldl be 2.75
    private boolean isBRAKE;

    Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void mapRobot() {
        drive = new Drive(opMode);
        forkLift = new ForkLift(opMode);
        jewelArm = new JewelArm(opMode);
        relicClaw = new RelicClaw(opMode);
        phone = new Phone(opMode);
        imu = new REVGyro(opMode);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "d1");

    }


    public static Robot getInstance(LinearOpMode linearOpModeInstance) {
        if (instance == null) instance = new Robot(linearOpModeInstance);
        return instance;
    }

    //public static boolean opModeIsActive() {return opMode.opModeIsActive();}

    public void pushInBlock() {
        forkLift.openClaw();
        sleep(150);
        drive.forwardTime(drive.DRIVE_INTO_CRYPTOBOX_SPEED, 400);
    }

    public void grabSecondGlyph() {
        boolean wasBRAKE = drive.isBRAKE();
        drive.setBRAKE();
        forkLift.openClaw();
        drive.backward(0.6, 4);
        forkLift.moveUntilDown();
        drive.forward(0.6, 5);
        forkLift.closeClaw();
        sleep(300);
        forkLift.moveMotor(1, 300);
        if (!wasBRAKE) {
            drive.setFLOAT();
        }
    }

    public void stopAll() {
        drive.stopMotors();
        forkLift.moveMotor(0);
        relicClaw.moveMotor(0);
    }

    public void strafeForMultiGlyph(double distanceToStrafe) {
        if (distanceToStrafe > 0) {
            drive.strafeRight(drive.MAX_SPEED, distanceToStrafe);
        } else if (distanceToStrafe < 0) {
            drive.strafeLeft(drive.MAX_SPEED, distanceToStrafe);
        }
    }

    public void setUpMultiGlyph() {
        glyphDetector = new AutoGlyphs(opMode);
        glyphDetector.enable();
        forkLift.closeAllTheWay();
        phone.faceFront();

    }

    public void setUpMultiGlyph(GlyphDetector.GlyphDetectionSpeed glyphDetectionSpeed) {
        glyphDetector = new AutoGlyphs(opMode, GlyphDetector.GlyphDetectionSpeed.VERY_SLOW, 0);
        glyphDetector.enable();
        forkLift.openAllTheWay();
        phone.faceFront();
    }

    public void rightGyro(double x, double y, double z, double target, double finalSpeed) {
        double Adjustedtarget = target + GYRO_OFFSET + SLOW_OFFSET;
        double finalTarget = target + GYRO_OFFSET;
        heading = getHeading();
        double derivative = 0;
        double fl = clip(-y + -x - z);
        double fr = clip(-y + x + z);
        double rl = clip(-y + x - z);
        double rr = clip(-y + -x + z);
        drive.driveSpeeds(fl, fr, rl, rr);
        double current = heading;
        double last = heading;
        if (heading < target) {
            while (derivative <= 180 && opMode.opModeIsActive()) {
                current = getHeading();
                derivative = current - last;
                last = current;
            }
        }
        double start = getHeading();
        double distance = Adjustedtarget - start;
        heading = start;
        while (heading > Adjustedtarget && opMode.opModeIsActive()) {
            heading = getHeading();
            double remaining = heading - start;
            double proportion = (1 - (Math.abs((remaining) / distance))) * 0.25 + 0.75;
            drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
        }
        drive.stopMotors();
        while (heading > finalTarget && opMode.opModeIsActive()) {
            heading = getHeading();
            drive.driveSpeeds(fl * finalSpeed, fr * finalSpeed, rl * finalSpeed, rr * finalSpeed);
        }
        drive.stopMotors();

    }

    public void rightGyro(double x, double y, double z, double target) {
        rightGyro(x, y, z, target, 0.2);
    }

    public void rightGyro(double speed, double target) {
        rightGyro(0, 0, Math.abs(speed), target);
    }

    public void leftGyro(double x, double y, double z, double target, double finalSpeed) {
        double adjustedTarget = target - GYRO_OFFSET - SLOW_OFFSET;
        double finalTarget = target - GYRO_OFFSET;
        heading = getHeading();
        double derivative = 0;
        double fl = clip(-y + -x - z);
        double fr = clip(-y + x + z);
        double rl = clip(-y + x - z);
        double rr = clip(-y + -x + z);
        drive.driveSpeeds(fl, fr, rl, rr);
        double current = heading;
        double last = heading;
        if (target < heading) {
            while (derivative >= -180 && opMode.opModeIsActive()) {
                current = getHeading();
                derivative = current - last;
                last = current;
            }
        }
        double start = getHeading();
        double distance = adjustedTarget - start;
        heading = start;
        while (heading < adjustedTarget && opMode.opModeIsActive()) {
            heading = getHeading();
            double remaining = heading - start;
            double proportion = (1 - (Math.abs((remaining) / distance))) * 0.5 + 0.5;
            drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
        }
        drive.stopMotors();
        while (heading < finalTarget && opMode.opModeIsActive()) {
            heading = getHeading();
            drive.driveSpeeds(fl * finalSpeed, fr * finalSpeed, rl * finalSpeed, rr * finalSpeed);
        }
        drive.stopMotors();

    }

    public void leftGyro(double x, double y, double z, double target) {
        leftGyro(x, y, z, target, 0.2);
    }

    public void leftGyro(double speed, double target) {
        leftGyro(0, 0, -Math.abs(speed), target);
    }

    public void gyroGoTo(double speed, double target) {
        heading = getHeading();
        speed = -Math.abs(speed);
        if (heading - 0.5 > target) { //RIGHT TURN
            double Adjustedtarget = target + GYRO_OFFSET;
            double fl = +speed;
            double fr = -speed;
            double rl = +speed;
            double rr = -speed;
            drive.driveSpeeds(fl, fr, rl, rr);
            double start = getHeading();
            double distance = Adjustedtarget - start;
            while (heading > Adjustedtarget && opMode.opModeIsActive()) {
                heading = getHeading();
                double proportion = Range.clip(1 - (Math.abs((heading - start) / distance)), 0.01, 1);
                drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
            }
            drive.stopMotors();
        } else if (heading + 0.5 < target) { //LEFT TURN
            double adjustedTarget = target - GYRO_OFFSET;
            double fl = -speed;
            double fr = +speed;
            double rl = -speed;
            double rr = +speed;
            drive.driveSpeeds(fl, fr, rl, rr);
            double start = heading;
            double distance = adjustedTarget - start;
            while (heading < adjustedTarget && opMode.opModeIsActive()) {
                heading = getHeading();
                double proportion = Range.clip(1 - (Math.abs((heading - start) / distance)), 0.01, 1);
                drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
            }
            drive.stopMotors();
        }
    }

    public void strafeUntilDistance(double speed, double endDistance) {
        double derivative = 1;
        double distance = getDistance();
        double current = distance;
        double last = current;
        while ((derivative != 0) && (current < 40) && (current > 8)) {
            current = getDistance();
            derivative = current - last;
            last = current;
        }
        distance = current;
        telemetry.addData("Initial Distance", distance);
        telemetry.update();
        if (distance > endDistance) {
            speed = Math.abs(speed); //Move right
        } else if (distance < endDistance) {
            speed = -Math.abs(speed);
        } else {
            return;
        }
        double fl = clip(-speed);
        double fr = clip(speed);
        double rl = clip(speed);
        double rr = clip(-speed);
        if (distance < endDistance) {
            while (distance < endDistance && opMode.opModeIsActive()) {
                drive.driveSpeeds(drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, fl)), drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, fr)), drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, rl)), drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, rr)));
                current = getDistance();
                derivative = Math.abs(current - last);
                if ((derivative >= 0) && (derivative < 6)) {
                    distance = current;
                }
                last = distance;
            }
            drive.stopMotors();
        } else if (distance > endDistance) {
            while (distance > endDistance && opMode.opModeIsActive()) {
                drive.driveSpeeds(drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, fl)), drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, fr)), drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, rl)), drive.clipStrafeSpeed(drive.calculateSpeedLog(distance, endDistance, rr)));
                current = getDistance();
                derivative = Math.abs(current - last);
                if ((derivative >= 0) && (derivative < 6)) {
                    distance = current;
                }
                last = distance;
            }
            drive.stopMotors();

        }
    }

    private double getDistance(DistanceUnit unit) {
        return rangeSensor.getDistance(unit);
    }

    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    public void calibrateGyro() {
        imu.calibrate();
        heading = getHeading();
    }

    private double getHeading() {
        return imu.getHeading();
    }

    private void telemetrizeGyro() {
        telemetry.addData("Current heading: ", heading);
    }

    private void telemetrizeDistance() {
        telemetry.addData("Distance", getDistance());
    }

    public static void sleep(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException ignored) {
        }
    }

    private double clip(double value) {
        return Range.clip(value, -1, 1);
    }

    public void getMoreGlyphsRecovery(RelicRecoveryVuMark pictograph, ElapsedTime runTime) {
        leftGyro(drive.MAX_SPEED, 90);
        forkLift.closeAllTheWay();
        drive.backwardTime(drive.MAX_SPEED, 125);
        switch (pictograph) {
            case LEFT:
                drive.strafeLeft(drive.MAX_SPEED, (2 * drive.CRYPTOBOX_COLUMNS_OFFSET) - 3);
                break;
            case CENTER:
                drive.strafeLeft(drive.MAX_SPEED, drive.CRYPTOBOX_COLUMNS_OFFSET - 3);
                break;
            case RIGHT:
                drive.strafeRight(drive.MAX_SPEED, drive.CRYPTOBOX_COLUMNS_OFFSET - 3);
                break;
        }
        setUpMultiGlyph();
        gyroGoTo(0.5, 90);
        ElapsedTime findGlyphTime = new ElapsedTime();
        findGlyphTime.reset();
        double xPos = 0, yPos = 0, size = 0, bestGlyphSize = 0, distanceToStrafe = 0;
        Point bestGlyphPos = new Point(AutoGlyphs.DEFAULT_X_POS_VALUE, 0);
        while (findGlyphTime.seconds() < 3.5) {
            xPos = glyphDetector.getXPos();
            yPos = glyphDetector.getYPos();
            size = glyphDetector.getSize();
            if ((xPos != AutoGlyphs.DEFAULT_X_POS_VALUE) && (size < 125) && (size > 60) && (yPos < 40) && (yPos > -170) && (Math.abs(xPos) < 70)) {
                bestGlyphPos.x = xPos;
                bestGlyphPos.y = yPos;
                bestGlyphSize = size;
                distanceToStrafe = (bestGlyphPos.x * STRAFING_DAMPEN_FACTOR_FOR_MULTI_GLYPH) + phone.PHONE_DISTANCE_OFFSET;
                break;
            }
        }
        if (distanceToStrafe != 0) {
            telemetry.addData("Glyph Position", bestGlyphPos.toString());
            telemetry.addData("Size", bestGlyphSize);
            telemetry.addData("Found at", findGlyphTime.seconds());
        } else {
            telemetry.addData("Would be glyph position", xPos + ", " + yPos);
            telemetry.addData("Would be glyph size", size);
        }
        telemetry.update();
        glyphDetector.disable();
        forkLift.openClaw();
        if (bestGlyphPos.x == AutoGlyphs.DEFAULT_X_POS_VALUE) bestGlyphPos.x = 0;
        strafeForMultiGlyph(distanceToStrafe);
        phone.faceSideways();
        drive.forward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPH_PIT_DISTANCE + drive.DRIVE_INTO_GLYPHS_DISTANCE);
        forkLift.closeClaw();
        sleep(200);
        forkLift.moveMotor(1, 550);
        gyroGoTo(0.5, 90);
        drive.backward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPH_PIT_DISTANCE - 7);
        if (distanceToStrafe > 7) leftGyro(drive.MAX_SPEED, -90);
        else rightGyro(drive.MAX_SPEED, -90);
        strafeForMultiGlyph(distanceToStrafe - drive.CRYPTOBOX_COLUMNS_OFFSET + 6);
        drive.forward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPHS_DISTANCE + 4);
        drive.forwardTime(drive.MAX_SPEED, 250);
        gyroGoTo(0.5, -90);
        drive.strafeLeftTime(drive.DRIVE_INTO_GLYPHS_SPEED, 300);
        forkLift.openClaw();
        drive.forwardTime(drive.MAX_SPEED, 300);
        sleep(200);
        drive.backward(drive.MAX_SPEED, 6);
        forkLift.closeAllTheWay();
        forkLift.moveMotor(-1, 500);
        leftGyro(drive.MAX_SPEED, 90);
        forkLift.openClaw();
        phone.faceSideways();
        drive.forward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPH_PIT_DISTANCE + drive.DRIVE_INTO_GLYPHS_DISTANCE - 11);
        drive.forward(drive.DRIVE_INTO_GLYPHS_SPEED, drive.DRIVE_INTO_GLYPHS_DISTANCE);
        forkLift.closeClaw();
        sleep(300);
        forkLift.moveMotor(1, 550);
        gyroGoTo(0.5, 90);
        drive.backward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPH_PIT_DISTANCE - 12);
        leftGyro(drive.MAX_SPEED, -90);
        drive.strafeRight(drive.MAX_SPEED, drive.CRYPTOBOX_COLUMNS_OFFSET + 6);
        drive.forward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPHS_DISTANCE + 4);
        drive.forwardTime(drive.MAX_SPEED, 250);
        gyroGoTo(0.5, -90);
        drive.strafeLeftTime(drive.DRIVE_INTO_GLYPHS_SPEED, 350);
        forkLift.openClaw();
        drive.forwardTime(drive.MAX_SPEED, 300);
        forkLift.moveMotor(1, 150);
        drive.backward(drive.MAX_SPEED, 6);
    }
}
