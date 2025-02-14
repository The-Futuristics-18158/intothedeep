package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class Sweep1 extends SequentialCommandGroup {

    // constructor
    public Sweep1() {

        addCommands (

                new MoveToPose(
                        2.0,
                        1.5,
                        AutoFunctions.redVsBlue(new Pose2d(-0.9, 1.2, new Rotation2d(Math.toRadians(-90.0))))
                ),

                new MoveToPose(
                        2.0,
                        1.5,
                        AutoFunctions.redVsBlue(new Pose2d(-1.05, 0.25, new Rotation2d(Math.toRadians(-90.0))))
                ),

                new MoveToPose(
                        2.0,
                        1.5,
                        AutoFunctions.redVsBlue(new Pose2d(-1.05, 1.35, new Rotation2d(Math.toRadians(-90.0))))
                )



        );
    }

}