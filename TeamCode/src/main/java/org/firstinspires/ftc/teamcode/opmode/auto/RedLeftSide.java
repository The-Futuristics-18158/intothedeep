package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.Auto.LeftSideAuto87Pts;
import org.firstinspires.ftc.teamcode.RobotContainer;

/*
 * This file contains an example of an "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 */
@Autonomous(name="RedLeftSide", group="OpMode", preselectTeleOp="Red TeleOp")
//@Disabled
public class RedLeftSide extends CommandOpMode {

    // Initialize all objects, set up subsystems, etc...
    @Override
    public void initialize() {

        // initialize for Auto in robot container
        // set team alliance color to red (isRedAlliance=true)
        RobotContainer.Init_Auto(this, true);

        // do not proceed until start button is pressed
        waitForStart();

        // ---------- autonomous command ----------

        // add autonomous command to scheduler and run it
        new LeftSideAuto87Pts().schedule();

    }

    // Run Op Mode. Is called after user presses play button
    // called continuously
    @Override
    public void run() {
        // execute robot periodic function
        RobotContainer.Periodic();
    }
}