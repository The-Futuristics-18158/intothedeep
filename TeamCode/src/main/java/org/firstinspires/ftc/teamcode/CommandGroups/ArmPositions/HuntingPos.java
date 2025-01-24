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
                    if (!RobotContainer.operatingMode.getSelectedMode())
                        if (RobotContainer.isRedAlliance)
                            RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.RED_BLOB_ONLY);
                        else
                            RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.BLUE_BLOB_ONLY);
                    else RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.YELLOW_BLOB_ONLY);}),

                // sets the elbow to a straight position at 102 degrees
                new InstantCommand(() -> RobotContainer.shoulderJoint.RotateTo(-33)),

                // sets elbow to a straight position at 150 degrees,
                // 150 is because of gravity and leverage bend the elbow down so the extra 10 degrees stops the drooping
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(15)),

                // sets wrist to a straight position  0 is parallel to floor
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(-135)),

                // powers and sets wrist to 180 degrees
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(45))

        );


    }

}