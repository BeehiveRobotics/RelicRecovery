package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor{
    private DcMotor motor;
    private OpMode opMode;

    private boolean isAuto;
    public Motor(OpMode opMode, String name) {
        this.motor = opMode.hardwareMap.dcMotor.get(name);
        this.opMode = opMode;
        isAuto = false;
    }
    public Motor(LinearOpMode opMode, String name) {
        this.motor = opMode.hardwareMap.dcMotor.get(name);
        this.opMode = opMode;
        isAuto = true;
    }
    public void setPower(double power) {
        if(isAuto) {
            //if(!Robot.opModeIsActive()) return;
        }
        motor.setPower(power);

    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavoir) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavoir);
    }
    /*public boolean opModeIsActive() {
        return Robot.getInstance(null).opModeIsActive();
    }*/
}
