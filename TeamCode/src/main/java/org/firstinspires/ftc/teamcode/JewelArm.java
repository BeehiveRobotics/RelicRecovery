package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JewelArm {
    public Servo upDownServo;
    public Servo endServo;
    ColorSensor cs;
    private final double DOWN_POSITION = 0;
    private final double UP_POSITION = 1;
    private final double RIGHT_POSITION = 0.8;
    private final double LEFT_POSITION = 0.2;
    private final double MIDDLE_POSITION = 0.5;
    private final double MIN_COLOR_DETECTION_THRESHOLD = 25;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public JewelArm (OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.endServo = hardwareMap.servo.get("s3");
        this.upDownServo = hardwareMap.servo.get("s4");
        this.cs = hardwareMap.colorSensor.get("cs1");
        this.telemetry = opMode.telemetry;
    }

    public void down() {
        setUpDownPosition(DOWN_POSITION);
    }

    public void up() {
        setUpDownPosition(UP_POSITION);
    }

    public void right() {setEndPosition(RIGHT_POSITION);}

    public void left() {setEndPosition(LEFT_POSITION);}

    public void middle() {setEndPosition(MIDDLE_POSITION);}

    public void init() {
        up();
        cs.enableLed(true);
    }

    public void knockJewel(AllianceColor allianceColor) {
        AllianceColor jewelColor = AllianceColor.UNKNOWN;
        if (cs.red() > cs.blue()) {
            jewelColor = AllianceColor.RED;
        }
        else if (cs.blue()>cs.red()) {
            jewelColor = AllianceColor.BLUE;
        }
        if(allianceColor == AllianceColor.RED) {
          if (jewelColor == AllianceColor.RED) {
            right();
          }
          else if (jewelColor == AllianceColor.BLUE) {
            left();
          }
        }
        if(allianceColor == AllianceColor.BLUE) {
          if (jewelColor == AllianceColor.BLUE) {
            right();
          }
          else if (jewelColor == AllianceColor.RED) {
            left();
          }
        }
    }

    public void setUpDownPosition(double postion) {
        upDownServo.setPosition(postion);
        upDownServo.setPosition(postion);
    }
    public void setEndPosition(double position) {
        endServo.setPosition(position);
        endServo.setPosition(position);
    }
    private void addLine(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }
}
