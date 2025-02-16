package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.BlinkinColour;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;

/**
 * The Blinkin class is a subsystem that controls the Rev Blinkin LED driver.
 * It extends the SubsystemBase class from FTCLib.
 */
public class Blinkin extends SubsystemBase {

    // Instance of the RevBlinkinLedDriver
    private RevBlinkinLedDriver blinkinLedDriver;

    // blink counter
    private int counter;

    /**
     * Constructor for the Blinkin class.
     * Initializes the Blinkin LED driver and sets the initial pattern.
     */
    public Blinkin() {
        // Creates a Blinkin LED driver using the hardware map
        blinkinLedDriver = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        counter=0;
    }

    /**
     * The periodic method is called periodically by the scheduler.
     * This method is empty as the Blinkin subsystem does not require periodic actions.
     */
    @Override
    public void periodic() {
        // blink counter
        counter +=1;
        if (counter>8)
            counter=0;
        boolean blink = (counter>4);

        // set blinkin colour
        if (RobotContainer.clawCamera.getVisionProcessingMode() == VisionProcessorMode.YELLOW_BLOB_ONLY
            && !RobotContainer.clawCamera.GetBlobDetections().isEmpty())
            setPattern(BlinkinColour.NEUTRAL_SAMPLE.getPattern());
        else if ((RobotContainer.clawCamera.getVisionProcessingMode() == VisionProcessorMode.RED_BLOB_ONLY ||
                RobotContainer.clawCamera.getVisionProcessingMode() == VisionProcessorMode.BLUE_BLOB_ONLY ) &&
                !RobotContainer.clawCamera.GetBlobDetections().isEmpty())
            setPattern(BlinkinColour.ALLIANCE_SAMPLE.getPattern());
        else if (RobotContainer.operatingMode.getSelectedMode() && blink)
            setPattern(BlinkinColour.NO_INFORMATION.getPattern());
        else if (RobotContainer.isRedAlliance)
            setPattern(BlinkinColour.RED_ALLIANCE.getPattern());
        else
            setPattern(BlinkinColour.BLUE_ALLIANCE.getPattern());
    }

    /**
     * Sets the pattern of the Blinkin LED driver.
     *
     * @param blinkinPattern The pattern to set on the Blinkin LED driver.
     */
    public void setPattern(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
        blinkinLedDriver.setPattern(blinkinPattern);
    }
}
