package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.opencv.core.Mat;

/**
 * Command to recalibrate the odometry of the robot.
 */
public class RecalibrateOdometry extends CommandBase {

    // known Y coordinate for the Red Alliance Chamber
    private static final double redAllianceChamber = -0.77;
    // known X coordinate for the Red Alliance Ascent
    private static final double redAllianceAscent = -0.5;
    // known Y coordinate for the Blue Alliance Chamber
    private static final double blueAllianceChamber = 0.77;
    // known X coordinate for the Blue Alliance Ascent
    private static final double blueAllianceAscent = 0.5;

    // how close the robot needs to be to the known position to be considered in the subzone
    private static final double positionTolerance = 0.25;
    // how close the robot needs to be to the known angle to be considered facing the submersible
    private static final double angleTolerance = 10.0;

    // The expected angle of the robot when facing the  red alliance chamber
    private static final double redAllianceChamberAngle = RobotContainer.RedStartAngle;
    // The expected angle of the robot when facing the red alliance ascent
    private static final double redAllianceAscentAngle = 0.0;
    // The expected angle of the robot when facing the blue alliance chamber
    private static final double blueAllianceChamberAngle =  RobotContainer.BlueStartAngle;
    // The expected angle of the robot when facing the blue alliance ascent
    private static final double blueAllianceAscentAngle = 180.0;

    // Determine if the robot is on the Red Alliance
    private static final boolean isRedAlliance = RobotContainer.isRedAlliance();

    // Current position of the robot
    Pose2d currentPos;

    //finished updating the robot's position based on its current position and alliance color.
    boolean finishedUpdating = false;

    /**
     * Constructor for the recalibrateOdometry command.
     * Adds the odometry subsystem as a requirement.
     */
    public RecalibrateOdometry() {
        addRequirements(RobotContainer.getOdometry());
    }

    /**
     * Called once when the command is started.
     * Initializes the current position and recalibrates the robot's position based on the alliance color.
     */
    @Override
    public void initialize() {
        currentPos = RobotContainer.getOdometry().getCurrentPos();
        updatePosition();
        RobotContainer.getDBTelemetry().addData("Pressed","Depressed");
        RobotContainer.getDBTelemetry().update();
    }

    /**
     * Called periodically while the command is active.
     */
    @Override
    public void execute() {
        // No periodic actions required for this command
    }

    /**
     * Determines if the command is finished.
     *
     * @return false as this command does not have a termination condition.
     */
    @Override
    public boolean isFinished() {
        RobotContainer.getDBTelemetry().addData("Pressed","Happy");
        RobotContainer.getDBTelemetry().update();
        return finishedUpdating;
    }

    /**
     * Called once when the command is finished.
     *
     * @param interrupted whether the command was interrupted/canceled.
     */
    @Override
    public void end(boolean interrupted) {
        // No cleanup actions required for this command
    }

    /**
     * Updates the robot's position based on its current position and alliance color.
     */
    private void updatePosition() {
        if (inSubZone()) {
            // Facing Chamber Wall
            if (facingChamber()) {
                RobotContainer.getDBTelemetry().addData("Facing X Wall","Yes");
                RobotContainer.getDBTelemetry().update();
                if (isRedAlliance) {
                    RobotContainer.getDBTelemetry().addData("Y position set to ", redAllianceChamber);
                    RobotContainer.getDBTelemetry().update();
                    RobotContainer.getOdometry().setCurrentPos(new Pose2d(redAllianceChamber, currentPos.getY(), new Rotation2d(Math.toRadians(redAllianceChamberAngle))));
                } else {
                    RobotContainer.getDBTelemetry().addData("Y position set to ", blueAllianceChamber);
                    RobotContainer.getDBTelemetry().update();
                    RobotContainer.getOdometry().setCurrentPos(new Pose2d(currentPos.getX(),blueAllianceChamber, new Rotation2d(Math.toRadians(blueAllianceChamberAngle))));
                }
            }

            // Facing Ascent Wall
            if (facingAscent()) {
                RobotContainer.getDBTelemetry().addData("Facing Y Wall","Yes");
                RobotContainer.getDBTelemetry().update();
                if (isRedAlliance) {
                    RobotContainer.getDBTelemetry().addData("X position set to ", redAllianceAscent);
                    RobotContainer.getDBTelemetry().update();
                    RobotContainer.getOdometry().setCurrentPos(new Pose2d(currentPos.getX(), redAllianceAscent, new Rotation2d(Math.toRadians(redAllianceAscentAngle))));
                } else {
                    RobotContainer.getDBTelemetry().addData("X position set to ", blueAllianceAscent);
                    RobotContainer.getDBTelemetry().update();
                    RobotContainer.getOdometry().setCurrentPos(new Pose2d( blueAllianceAscent,currentPos.getY(), new Rotation2d(Math.toRadians(blueAllianceAscentAngle))));
                }
            }
        }
        finishedUpdating = true;
        RobotContainer.getDBTelemetry().addData("finished updating","true");
        RobotContainer.getDBTelemetry().update();
    }

    /**
     * Checks if the robot is facing the X wall.
     *
     * @return true if the robot is facing the X wall, false otherwise.
     */
    private boolean facingChamber() {
        double angle = Math.toDegrees(currentPos.getHeading());
        RobotContainer.getDBTelemetry().addData("Print Value",angle);
        RobotContainer.getDBTelemetry().update();
        if (isRedAlliance) {
            return Math.abs(angle - redAllianceChamberAngle) <= angleTolerance;
        } else {
            return Math.abs(angle - blueAllianceChamberAngle) <= angleTolerance;
        }
    }

    /**
     * Checks if the robot is facing the Y wall.
     *
     * @return true if the robot is facing the Y wall, false otherwise.
     */
    private boolean facingAscent() {
        double angle = Math.toDegrees(currentPos.getHeading());
        if (isRedAlliance) {
            return Math.abs(angle - redAllianceAscentAngle) <= angleTolerance;
        } else {
            return Math.abs(angle - blueAllianceAscentAngle) <= angleTolerance;
        }
    }

    /**
     * Checks if the robot is in the SubZone.
     *
     * @return true if the robot is in the SubZone, false otherwise.
     */
    private boolean inSubZone() {
        boolean inTheZone = false;
        // Check if we are in the SubZone
        if (isRedAlliance) {
            // Check if we are in the Red Alliance SubZone
            inTheZone = Math.abs(currentPos.getY() - redAllianceChamber) <= positionTolerance || Math.abs(currentPos.getX() - redAllianceAscent) <= positionTolerance;
        } else {
            // Check if we are in the Blue Alliance SubZone
            inTheZone = Math.abs(currentPos.getY() - blueAllianceChamber) <= positionTolerance || Math.abs(currentPos.getX() - blueAllianceAscent) <= positionTolerance;
        }
        return inTheZone;
        //return true;
    }
}