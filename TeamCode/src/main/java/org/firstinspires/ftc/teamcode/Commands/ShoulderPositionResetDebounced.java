package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class ShoulderPositionResetDebounced extends CommandBase {

    ElapsedTime debouceTime;

    // constructor
    public ShoulderPositionResetDebounced() {

        debouceTime = new ElapsedTime();
        debouceTime.reset();

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.shoulderJoint);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // if 5 seconds has expired then allow execution. Otherwise, ignore
        if (debouceTime.seconds() > 10.0)
        {
            RobotContainer.shoulderJoint.ResetMotorPositionOnButton();
            debouceTime.reset();
        }

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return true;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}