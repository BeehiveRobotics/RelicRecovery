package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Kaden on 10/19/2017.
 */
@TeleOp(name = "DriveEverythingTranslateRotate", group = "linear OpMode")
public class DriveEverythingTranslateRotate extends OpMode {
    private ForkLift ForkLift;
    private RelicClaw RelicClaw;
    private DriveMecanum drive;

    @Override
    public void init() {
        ForkLift = new ForkLift(hardwareMap.servo.get("s5"), hardwareMap.servo.get("s6"), hardwareMap.dcMotor.get("m6"), hardwareMap.digitalChannel.get("b0"), hardwareMap.digitalChannel.get("b1"), telemetry);
        ForkLift.init();
        RelicClaw = new RelicClaw(hardwareMap.servo.get("s1"), hardwareMap.servo.get("s2"), hardwareMap.dcMotor.get("m5"), telemetry);
        RelicClaw.init();
        drive = new DriveMecanum(hardwareMap.dcMotor.get("m1"), hardwareMap.dcMotor.get("m2"), hardwareMap.dcMotor.get("m3"), hardwareMap.dcMotor.get("m4"), 1.0, telemetry);
    }

    @Override
    public void loop() {
        //Drive
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            drive.driveTranslateRotate(gamepad1.left_stick_y/4, gamepad1.left_stick_x/4, gamepad1.right_stick_x/4);

        } else {
            drive.driveTranslateRotate(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }
        //ForkLift
        if (gamepad1.a) {
            ForkLift.closeClaw();
        }
        if (gamepad1.b) {
            ForkLift.openClaw();
        }
        ForkLift.moveMotor(gamepad1.right_trigger - gamepad1.left_trigger);
        //Relic arm
        if (gamepad2.a) {
            RelicClaw.closeClaw();
        }
        if (gamepad2.b) {
            RelicClaw.openClaw();
        }
        if (gamepad2.x) {
            RelicClaw.down();
        }
        if (gamepad2.y) {
            RelicClaw.up();
        }
        RelicClaw.moveMotor(-gamepad2.left_stick_y);
    }
}