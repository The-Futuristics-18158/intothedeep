package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.SpecimenPlacePos;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Drive.GoToNextDropOff;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class PlaceSpecimenAddOffset extends SequentialCommandGroup {

    // constructor
    public PlaceSpecimenAddOffset() {

        addCommands (

                //sets arm to specimen place position
                new SpecimenPlacePos(),
                //sets the slides to low
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW, true)),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(0.0, 0.740, new Rotation2d(Math.toRadians(-90))))),// was 0.820

                new FollowPath(
                        1.5,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-35))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(-0.15, 0.740, new Rotation2d(Math.toRadians(-90)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90)))),

                new Pause(0.15),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.10, 0.740, new Rotation2d(Math.toRadians(-90))))),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.13, 0.740, new Rotation2d(Math.toRadians(-90))))),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMLE_SPECIMEN, true)),

                new Pause(0.5),

                new OpenClaw(),

                new Pause(0.25),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO, true)),

                new Pause(1.25),

                new ArmStowHigh()
        );


    }

}