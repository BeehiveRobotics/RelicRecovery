package org.firstinspires.ftc.teamcode;

/**
 * Created by Kaden on 1/2/2018.
 */

public enum JewelColor {
    RED, BLUE, UNKNOWN;
    JewelColor not(JewelColor target) {
        switch (target) {
            case RED: return BLUE;
            case BLUE: return RED;
            case UNKNOWN: return UNKNOWN;
        }
        return UNKNOWN;
    }
}