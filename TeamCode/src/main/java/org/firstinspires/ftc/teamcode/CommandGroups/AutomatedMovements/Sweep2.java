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

public class Sweep2 extends SequentialCommandGroup {

    // constructor
    public Sweep2() {

        addCommands (
                // Sweep 2
//                new MoveToPose(
//                        2.0,
//                        1.5,
//                        AutoFunctions.redVsBlue(new Pose2d(-1.35, 1.2, new Rotation2d(Math.toRadians(-90.0))))),

//                new MoveToPose(
//                        2.0,
//                        1.5,
//                        AutoFunctions.redVsBlue(new Pose2d(-1.35, 0.25, new Rotation2d(Math.toRadians(-90.0))))),

                new FollowPath(
                        1.0,
                        1.0,
                        1.0,
                        1.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(90.0))),
                        new ArrayList<Translation2d>() {{}},
                        AutoFunctions.redVsBlue(new Pose2d(-0.9, 1.2, new Rotation2d(Math.toRadians(90.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),

                new FollowPath(
                        1.0,
                        1.0,
                        1.0,
                        1.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
                        new ArrayList<Translation2d>() {{}},
                        AutoFunctions.redVsBlue(new Pose2d(-1.35, 0.25, new Rotation2d(Math.toRadians(90.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),

                new MoveToPose(
                        2.0,
                        1.5,
                        AutoFunctions.redVsBlue(new Pose2d(-1.35, 1.45, new Rotation2d(Math.toRadians(-90.0)))))

        );


    }

}