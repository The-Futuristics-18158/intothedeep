package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class HuntingPos extends SequentialCommandGroup {

    // constructor
    public HuntingPos() {
        addCommands(

                // switch on claw camera to look for red or blue samples - depending which team we are on
                new InstantCommand(()-> {
                    if (RobotContainer.isRedAlliance)
                        RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.RED_BLOB_ONLY);
                    else
                        RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.BLUE_BLOB_ONLY);
                } ),

                // sets the elbow to a straight position at 130 degrees
                new InstantCommand(() -> RobotContainer.shoulderJoint.RotateTo(120)),

                // sets elbow to a straight position at 145 degrees,
                // 145 is becouse gravity and leverage bend the elbow down so the extera 10 degrees stops the drooping
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(145)),

                // sets wrist to a straight position  45 is parallel to floor
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(0)),

                // powers and sets wrist to a straight position
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180))

        );


    }

}