package org.firstinspires.ftc.teamcode.Subsystems.Arm.Shoulder;

public enum ShoulderPosition {

    // Constants that store the ticks for the shoulder position
    // 288 ticks/revolution
    GROUND(10), // Roughly half of max
    HIGH(-10), // Just under zero
    FLEX(35); // Max before breakage occurs

    // Stores ticks in var value
    private final int value;

    // Assigns the ticks to each constant
    ShoulderPosition(int value) {
        this.value = value;
    }

    // Gets the ticks from the enum
    public int getValue() {
        return value;
    }

}
