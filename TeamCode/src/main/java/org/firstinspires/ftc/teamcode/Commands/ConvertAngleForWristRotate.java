package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.vision.ColorAndOrientationDetect;
import org.firstinspires.ftc.teamcode.vision.DetectedAngle;


// command template
public class ConvertAngleForWristRotate extends CommandBase {

    // constructor
    public ConvertAngleForWristRotate() {

        addRequirements(RobotContainer.wristRotateServo);
        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        RobotContainer.wristRotateServo.RotateTo(RobotContainer.piece_angle, 90);
        //RobotContainer.wristRotateServo.RotateTo((int) Math.round(new ColorAndOrientationDetect().calAngle("Blue")));
        //new Pause(3);
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