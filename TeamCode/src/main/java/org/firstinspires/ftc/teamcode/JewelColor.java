package org.firstinspires.ftc.teamcode;

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