package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue Recovery", group = "Autonomous")
public class BlueRecovery extends LinearOpMode {
    private RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;
    private Robot robot;
    private static final double MOVE_TOWARDS_CRYPTOBOX_DISTANCE_BLUE_RECOVERY = 36.5;

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
        robot.jewelArm.knockJewel(JewelColor.BLUE);
        pictograph = robot.phone.getMark();
        if (pictograph == RelicRecoveryVuMark.UNKNOWN) {
            pictograph = RelicRecoveryVuMark.RIGHT;
        }
        switch (pictograph) {
            case LEFT:
                robot.drive.backward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_BLUE_RECOVERY - robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
            case CENTER:
                robot.drive.backward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_BLUE_RECOVERY);
            case RIGHT:
                robot.drive.backward(robot.drive.DRIVE_OFF_BALANCE_BOARD_SPEED, MOVE_TOWARDS_CRYPTOBOX_DISTANCE_BLUE_RECOVERY + robot.drive.CRYPTOBOX_COLUMNS_OFFSET_RECOVERY);
        }
        robot.forkLift.moveMotor(-1, 250);
        robot.rightGyro(robot.drive.MAX_SPEED, -90);
        robot.drive.forward(robot.drive.DRIVE_INTO_CRYPTOBOX_SPEED, 5);
        robot.pushInBlock();
        robot.drive.backward(robot.drive.MAX_SPEED, 4);
        robot.leftGyro(robot.drive.MAX_SPEED, 90);
        robot.forkLift.openClaw();
        robot.forkLift.moveUntilDown();
        sleep(1000);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}