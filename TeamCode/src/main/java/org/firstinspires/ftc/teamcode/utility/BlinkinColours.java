package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.RobotContainer;

public class BlinkinColours {
    public enum BlinkinColour {
        NOINFORMATION,
        BLUEALLIANCE,
        REDALLIANCE,
        ALLIANCESAMPLE,
        NETURALSAMPLE,
        PIECEGRABED,
        APRILTAGDETECTED,
    }


    public static void SetBlinkinColour(BlinkinColour colour) {
        if (colour == BlinkinColour.NOINFORMATION) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        } else if (colour == BlinkinColour.BLUEALLIANCE) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        } else if (colour == BlinkinColour.REDALLIANCE) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        } else if (colour == BlinkinColour.ALLIANCESAMPLE) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
        } else if (colour == BlinkinColour.NETURALSAMPLE) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        } else if (colour == BlinkinColour.PIECEGRABED) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
        } else if (colour == BlinkinColour.APRILTAGDETECTED) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        }

    }
}