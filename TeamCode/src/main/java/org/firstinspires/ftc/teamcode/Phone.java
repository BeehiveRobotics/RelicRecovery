package org.firstinspires.ftc.teamcode;

/**
 * Created by Kaden on 12/26/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class Phone {
    private static final double SIDE_POSITION = 0.5;
    private ClosableVuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    private ElapsedTime time;
    private Servo servo;
    private final double PICTOGRAPH_POSITION = 0.5;
    private final double FRONT_POSITION = 0;
    private boolean isCameraOpened = false;
    public Phone(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.servo = hardwareMap.servo.get("s7");

    }
    public void readyVuforia () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQfaFkD/////AAAAGXGcl3Y64kcghjX73kddwxOG8QFmwZwdDenQL/6cYT4JrZ70fydV0F5+iIWald5VzqX9BOtH9HwJ93W9oSnZmSwZSEQbnV3ELVR08qyIoujP5Z7O5p9yyepVydgdsjNw2shES0SmGoqhJF25ZIBN2YRVAYM++FTu4nuEEpLxN9LzbnrYLEfZB6mcuV9jea6D+CLXoQW7VpRpey73HjKCxPw1Hs3CjRx9/80Z6AR8YNjr3Yqx5MSZWNIn48rSR+nC0urM6YLs8xBwNA662icRKwkgAoCUXehfvxjK6LcSCnuQKG76IlOmSp3SZB9MFJ1HasxaFLxfS1xEa+6fdA7jE/WhukyuNvzmOVrWatS2WWDm";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        try {
            vuforia = new ClosableVuforiaLocalizer(parameters);
            isCameraOpened = true;
        } catch(VuforiaLocalizerImpl.FailureException e) {isCameraOpened = false;}
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        time = new ElapsedTime();
        vuMark = RelicRecoveryVuMark.UNKNOWN;
    }
    public RelicRecoveryVuMark getMark() {
        CameraDevice.getInstance().setFlashTorchMode(true);
        if(!isCameraOpened) {return vuMark;}
        relicTrackables.activate();
        setServoPosition(PICTOGRAPH_POSITION);
        time.reset();
        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
          vuMark = RelicRecoveryVuMark.from(relicTemplate);
          if(time.seconds()>2) {
              break;
          }
        }
        telemetry.addData("Pictograph", "%s visible", vuMark);
        telemetry.update();
        closeVuforia();
        return vuMark;
    }
    private void setServoPosition(double position) {
      servo.setPosition(position);
    }
    public void closeVuforia() {
        vuforia.close();
    }
    public void faceFront() {setServoPosition(FRONT_POSITION);}

    public void faceSideways() {
        setServoPosition(SIDE_POSITION);
    }
}
