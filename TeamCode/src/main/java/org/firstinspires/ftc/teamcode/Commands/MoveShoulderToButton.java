package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class MoveShoulderToButton extends CommandBase {
    ElapsedTime initialDelay;
    double Angle;
    ElapsedTime Delay;

    // constructor
    public MoveShoulderToButton() {

        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.shoulderJoint);
        initialDelay = new ElapsedTime();
        Delay = new ElapsedTime();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        Angle = 55;
        RobotContainer.shoulderJoint.RotateTo(Angle);
        initialDelay.reset();
        Delay.reset();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        if (initialDelay.seconds()>= 0.2) {


        }
        else
            Delay.reset();

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return RobotContainer.shoulderJoint.getShoulderButton();

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            RobotContainer.shoulderJoint.ResetMotorPositionOnButton();

        }
    }

}