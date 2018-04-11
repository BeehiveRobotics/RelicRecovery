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
    public Intake intake;
    private REVGyro imu;
    public double heading;
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
        intake = new Intake(opMode);
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
            while (derivative <= 180) {
                current = getHeading();
                derivative = current - last;
                last = current;
            }
        }
        double start = getHeading();
        double distance = Adjustedtarget - start;
        heading = start;
        while (heading > Adjustedtarget) {
            heading = getHeading();
            double remaining = heading - start;
            double proportion = (1 - (Math.abs((remaining) / distance))) * 0.25 + 0.75;
            drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
        }
        drive.stopMotors();
        while (heading > finalTarget) {
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
            while (derivative >= -180) {
                current = getHeading();
                derivative = current - last;
                last = current;
            }
        }
        double start = getHeading();
        double distance = adjustedTarget - start;
        heading = start;
        while (heading < adjustedTarget) {
            heading = getHeading();
            double remaining = heading - start;
            double proportion = (1 - (Math.abs((remaining) / distance))) * 0.5 + 0.5;
            drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
        }
        drive.stopMotors();
        while (heading < finalTarget) {
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
        if (heading - 1 > target) { //RIGHT TURN
            double Adjustedtarget = target + GYRO_OFFSET;
            double fl = +speed;
            double fr = -speed;
            double rl = +speed;
            double rr = -speed;
            drive.driveSpeeds(fl, fr, rl, rr);
            double start = getHeading();
            double distance = Adjustedtarget - start;
            while (heading > Adjustedtarget) {
                heading = getHeading();
                double proportion = Range.clip(1 - (Math.abs((heading - start) / distance)), 0.01, 1);
                drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
            }
            drive.stopMotors();
        } else if (heading + 1 < target) { //LEFT TURN
            double adjustedTarget = target - GYRO_OFFSET;
            double fl = -speed;
            double fr = +speed;
            double rl = -speed;
            double rr = +speed;
            drive.driveSpeeds(fl, fr, rl, rr);
            double start = heading;
            double distance = adjustedTarget - start;
            while (heading < adjustedTarget) {
                heading = getHeading();
                double proportion = Range.clip(1 - (Math.abs((heading - start) / distance)), 0.01, 1);
                drive.driveSpeeds(drive.clipSpinSpeed(fl * proportion), drive.clipSpinSpeed(fr * proportion), drive.clipSpinSpeed(rl * proportion), drive.clipSpinSpeed(rr * proportion));
            }
            drive.stopMotors();
        }
    }

    public void driveUntilDistance(double x, double y, double z, double endDistance) {
        double fl = clip(-y + -x - z);
        double fr = clip(-y + x + z);
        double rl = clip(-y + x - z);
        double rr = clip(-y + -x + z);
        drive.driveSpeeds(fl, fr, rl, rr);
        double start = getDistance();
        double distanceToTravel = endDistance - start;
        double proportion, derivative, current = getDistance(), ranTimes = 0, acceptedSensorValue = 0, last, distance;
        while (distance < endDistance) {
            proportion = 1 - Math.abs((distance - start) / distanceToTravel + 0.0001);
            drive.driveSpeeds(drive.clipStrafeSpeed(fl * proportion), drive.clipStrafeSpeed(fr * proportion), drive.clipStrafeSpeed(rl * proportion), drive.clipStrafeSpeed(rr * proportion));
            current = getHeading();
            derivative = getDistance() - distance;
            if (derivative >= 0 && derivative < 6) {
                distance = getDistance();
                acceptedSensorValue++;
            }
            ranTimes++;
            telemetry.addData("Ran times", ranTimes);
            telemetry.addData("Accepted Ratio", acceptedSensorValue / ranTimes);
            telemetrizeDistance();
        }
        telemetry.update();
        drive.stopMotors();
    }

    public void driveLeftUntilDistance(double speed, double distance) {
        driveUntilDistance(-Math.abs(speed), 0, 0, distance);
    }

    public void driveRightUntilDistance(double speed, double distance) {
        driveUntilDistance(Math.abs(speed), 0, 0, distance);
    }

    public double getDistance(DistanceUnit unit) {
        return rangeSensor.getDistance(unit);
    }

    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    public void calibrateGyro() {
        imu.calibrate();
        heading = getHeading();
    }

    public double getHeading() {
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
        } catch (InterruptedException e) {
        }
    }

    private double clip(double value) {
        return Range.clip(value, -1, 1);
    }

    public void getMoreGlyphsRecovery(RelicRecoveryVuMark pictograph, ElapsedTime runTime) {
        leftGyro(drive.MAX_SPEED, 90);
        forkLift.closeAllTheWay();
        drive.backwardTime(drive.MAX_SPEED, 75);
        switch (pictograph) {
            case LEFT:
                drive.strafeLeft(drive.MAX_SPEED, (2 * drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY) - 3);
                break;
            case CENTER:
                drive.strafeLeft(drive.MAX_SPEED, drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY - 3);
                break;
            case RIGHT:
                drive.strafeRight(drive.MAX_SPEED, drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY - 3);
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
        strafeForMultiGlyph(distanceToStrafe - drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY + 6);
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
        phone.faceFront();
        leftGyro(drive.MAX_SPEED, 90);
        yPos = 0;
        size = 0;
        bestGlyphSize = 0;
        distanceToStrafe = 0;
        if (runTime.seconds() > 23) {
            phone.faceSideways();
            sleep(1000);
            return;
        } else {
            glyphDetector.enable();
            while (glyphDetector.getXPos() == xPos) {
            }
            xPos = 0;
        }
        telemetry.addData("Run Time", runTime.seconds());
        telemetry.update();
        while (runTime.seconds() < 19) {
            xPos = glyphDetector.getXPos();
            yPos = glyphDetector.getYPos();
            size = glyphDetector.getSize();
            if ((xPos != AutoGlyphs.DEFAULT_X_POS_VALUE) && (size < 125) && (size > 60) && (yPos < 40) && (yPos > -170) && (Math.abs(xPos) < 70)) {
                bestGlyphPos.x = xPos;
                bestGlyphPos.y = yPos;
                bestGlyphSize = size;
                distanceToStrafe = bestGlyphPos.x * STRAFING_DAMPEN_FACTOR_FOR_MULTI_GLYPH + phone.PHONE_DISTANCE_OFFSET;
                break;
            }
        }
        if (distanceToStrafe != 0) {
            telemetry.addData("Glyph Position", bestGlyphPos.toString());
            telemetry.addData("Size", bestGlyphSize);
        } else {
            telemetry.addData("Would be glyph position", xPos + ", " + yPos);
            telemetry.addData("Would be glyph size", size);
        }
        telemetry.update();
        glyphDetector.disable();
        forkLift.openClaw();
        strafeForMultiGlyph(distanceToStrafe);
        phone.faceSideways();
        drive.forward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPH_PIT_DISTANCE + drive.DRIVE_INTO_GLYPHS_DISTANCE - 11);
        drive.forward(drive.DRIVE_INTO_GLYPHS_SPEED, drive.DRIVE_INTO_GLYPHS_DISTANCE);
        forkLift.closeClaw();
        sleep(300);
        forkLift.moveMotor(1, 550);
        gyroGoTo(0.5, 90);
        drive.backward(drive.MAX_SPEED, drive.DRIVE_INTO_GLYPH_PIT_DISTANCE - 12);
        leftGyro(drive.MAX_SPEED, -90);
        strafeForMultiGlyph(distanceToStrafe);
        drive.strafeRight(drive.MAX_SPEED, drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY + 6);
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
