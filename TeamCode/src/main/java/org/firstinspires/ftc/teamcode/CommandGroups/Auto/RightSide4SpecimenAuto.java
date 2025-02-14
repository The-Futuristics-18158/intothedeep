package org.firstinspires.ftc.teamcode.CommandGroups.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PlaceSpecimenAddOffset;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.Sweep1;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.Sweep2;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.WallPickUp;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
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
                new ArmStowHigh(),
                new CloseClaw(),
                new Pause(0.25),

                // placed 1
                new PlaceSpecimenAddOffset(),

                // needed to keep out of way of sweep path
                new ArmStowHigh(),
                // gathers 2
                new Sweep1(),
                new Sweep2(),

                // starts placing again
                //new WallPickUp(),
                // placed 2
                new PlaceSpecimenAddOffset(),

                //new WallPickUp(),
                // placed 3
                new PlaceSpecimenAddOffset(),

               // new WallPickUp(),
                // placed 4
                new PlaceSpecimenAddOffset(),

                // park
                new MoveToPose(
                    2.0,
                    1.5,
                    AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.2, new Rotation2d(Math.toRadians(-90.0))))
                )

        );


    }

}