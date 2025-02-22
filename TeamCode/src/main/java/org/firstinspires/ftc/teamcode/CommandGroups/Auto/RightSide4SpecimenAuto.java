package org.firstinspires.ftc.teamcode.CommandGroups.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.SpecimenPlacePos;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PlaceSpecimenAddOffset;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.Sweep1;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.Sweep2;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.WallPickUp;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class RightSide4SpecimenAuto extends SequentialCommandGroup {

    // constructor
    public RightSide4SpecimenAuto() {
        // start pos (0.25, 1.6, -90) on field
        addCommands (
                // sets the starting position
                new InstantCommand(() -> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(-0.22, 1.57, new Rotation2d(Math.toRadians(-90)))))),
                //makes sure the claw is closed
                new CloseClaw(),
                new Pause(0.25),

                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(35)),
                // folds the elbow in 225
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(110 + RobotContainer.elbowJoint.elbowServoOffset)),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW, true)),

                new Pause(1.0),

                new SpecimenPlacePos(),

                new MoveToPose(
                    1.5,
                    1.0,
                     AutoFunctions.redVsBlue(new Pose2d(-0.22, 1.2, new Rotation2d(Math.toRadians(-90))))
                ),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(0.0, 0.740, new Rotation2d(Math.toRadians(-90))))),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.13, 0.740, new Rotation2d(Math.toRadians(-90))))),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMLE_SPECIMEN, true)),

                new Pause(0.5),

                new OpenClaw(),

                new Pause(0.25),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO, true)),

                //placed 1
                new ArmStowHigh(),

                // gathers 1
                new Sweep1(),

                // starts placing again
                new WallPickUp(),
                // placed 2
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW, true)),

                new SpecimenPlacePos(),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.22, 1.2, new Rotation2d(Math.toRadians(-90))))
                ),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(0.0, 0.740, new Rotation2d(Math.toRadians(-90))))),

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.13, 0.740, new Rotation2d(Math.toRadians(-90))))),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMLE_SPECIMEN, true)),


                new Pause(0.5),

                new OpenClaw(),

                new Pause(0.25),

                new Sweep2()

//                new WallPickUp(),
//                // placed 3
//                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW, true)),
//
//                new SpecimenPlacePos(),
//
//                new MoveToPose(
//                        1.5,
//                        1.0,
//                        AutoFunctions.redVsBlue(new Pose2d(-0.22, 1.2, new Rotation2d(Math.toRadians(-90))))
//                ),
//
//                new MoveToPose(
//                        1.5,
//                        1.0,
//                        AutoFunctions.redVsBlue(new Pose2d(0.0, 0.740, new Rotation2d(Math.toRadians(-90))))),
//
//                new MoveToPose(
//                        1.5,
//                        1.0,
//                        AutoFunctions.redVsBlue(new Pose2d(-0.13, 0.740, new Rotation2d(Math.toRadians(-90))))),
//
//                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMLE_SPECIMEN, true)),
//
//                new Pause(0.5),
//
//                new OpenClaw(),

                //new Pause(0.25),

                // park


        );


    }

}