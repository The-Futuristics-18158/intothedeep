package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToWallPickup;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

public class WallPickUp extends SequentialCommandGroup {

    // constructor
    public WallPickUp() {

        addCommands (
        // What this position should do is give the camera a good vantage point as well as keep the arm out of the way

                new MoveToPose(
                        1.0,
                        0.5,
                        AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.2, new Rotation2d(Math.toRadians(-90.0))))
                ),

                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(45)),

                // folds the elbow in 270
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(265 + RobotContainer.elbowJoint.elbowServoOffset)),

                // folds the wrist in 0
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(85)),

                // powers the wrist and moves it to straight position
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(0)),

                new Pause(0.25),

                new OpenClaw(),

                new Pause(0.25),

                new MoveToWallPickup(),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                new Pause(0.25)
        );
    }

}