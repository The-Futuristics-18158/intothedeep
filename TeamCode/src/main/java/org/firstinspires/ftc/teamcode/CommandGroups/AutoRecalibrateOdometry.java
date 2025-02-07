package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class AutoRecalibrateOdometry extends SequentialCommandGroup {

    // constructor
    public AutoRecalibrateOdometry() {

        addCommands (
                new MoveToPose(0.5,
                        0.25,
                        AutoFunctions.redVsBlue(new Pose2d(0.5, 0.25, new Rotation2d(Math.toRadians(-180))))));

    }

    @Override
    public boolean isFinished() {

        boolean hasTouched = RobotContainer.frontTouch.hasTouched();

        return hasTouched;

    }
}

