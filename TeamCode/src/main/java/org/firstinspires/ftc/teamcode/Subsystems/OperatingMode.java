package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class OperatingMode extends SubsystemBase {

    // Local objects and variables here
    boolean TriggerLock;
    boolean prevTriggerState;

    /** Place code here to initialize subsystem */
    public OperatingMode() {
        prevTriggerState = false;
        TriggerLock = false;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // get trigger value from joystick - uses guide button
        double trigger = 0.0;
        if (RobotContainer.driverOp.gamepad.guide) trigger=1.0;

        // alternative if using l-trigger button
        //double trigger = RobotContainer.driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        if (!prevTriggerState && trigger > 0.75) {
            prevTriggerState=true;
            TriggerLock =!TriggerLock;
        }

        if (prevTriggerState && trigger < 0.25) {
            prevTriggerState=false; }

    }

    // returns false if mode is off, true if mode is enabled
    public boolean getSelectedMode()
    { return TriggerLock; }

}