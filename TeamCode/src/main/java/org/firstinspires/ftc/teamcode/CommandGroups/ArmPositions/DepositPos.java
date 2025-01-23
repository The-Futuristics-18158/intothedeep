package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class DepositPos extends SequentialCommandGroup {

    // constructor
    public DepositPos() {

        addCommands (

                // lifts the shoulder up 120 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(-15)),

                // folds the elbow in 200
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(65)),

                // folds the wrist in 70
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(-65)),

                // folds the wrist in 180
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(45))


        );
    }

}