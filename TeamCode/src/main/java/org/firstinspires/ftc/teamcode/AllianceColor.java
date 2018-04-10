package org.firstinspires.ftc.teamcode;

public enum AllianceColor {
    RED, BLUE, UNKNOWN;
    AllianceColor not(AllianceColor target) {
        switch (target) {
            case RED: return BLUE;
            case BLUE: return RED;
            case UNKNOWN: return UNKNOWN;
        }
        return UNKNOWN;
    }
}