package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class PreBackDepositPose extends SequentialCommandGroup {

    // constructor
    public PreBackDepositPose() {

        addCommands (

                // put arm into stow position
                // powers shoulder
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(45)),
                // folds the elbow in 225
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(135)),
                // folds the wrist in 45
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(0)),

                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180))


        );
    }

}