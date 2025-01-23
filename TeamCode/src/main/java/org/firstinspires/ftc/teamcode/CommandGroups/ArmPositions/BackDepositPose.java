package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class BackDepositPose extends SequentialCommandGroup {

    // constructor
    public BackDepositPose() {

        addCommands (

                // lifts the shoulder up 52 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(-83)),

                // folds the elbow in 198 degrees
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(63)),

                // folds the wrist in 180 degrees
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(45)),

                // folds the wrist in 180 degrees
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(45))


        );
    }

}