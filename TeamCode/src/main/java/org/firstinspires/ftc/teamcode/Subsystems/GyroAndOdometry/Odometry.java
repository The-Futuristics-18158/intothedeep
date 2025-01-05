package org.firstinspires.ftc.teamcode.Subsystems.GyroAndOdometry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.AprilTagUtils;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Point;

import java.util.List;


/** Subsystem */
//@Config // EXAMPLE - use @Config to add public variables to dashboard for realtime updating
public class Odometry extends SubsystemBase {

    // stored robot position (static) used to keep robot pose between opmodes
    // value persists even if new odometry system is created.
    private static Pose2d StoredRobotPose = new Pose2d(0,0, new Rotation2d(0));

    // Local objects and variables here
    private double previousLeftPos;
    private double previousRightPos;
    private double previousFrontPos;
    private double fieldX = 0.0;
    private double fieldY = 0.0;
    private double fieldAngle = 0.0;

    // historical x and y position samples
    final int history_array_size = 10;
    private double[] fieldHistoryX;
    private double[] fieldHistoryY;
    private double[] fieldHistoryTime;
    private ElapsedTime timer;

    // variables used for displaying paths on dashboard field widget
    // arrays hold x and y points of currently shown path(s)
    // arrays are set to null if no path to be shown
    private double[] currentTrajectoryXpoints;
    private double[] currentTrajectoryYpoints;

    // flag is true if apriltags are currently detected
    private boolean isTagDetected;

    /** Place code here to initialize subsystem */
    public Odometry() {

        // initialize field position from stored value (i.e. previous op-mode)
        setCurrentPos(StoredRobotPose);

        // create historical field position arrays
        fieldHistoryX = new double[history_array_size];
        fieldHistoryY = new double[history_array_size];;
        fieldHistoryTime = new double[history_array_size];;

        timer = new ElapsedTime();
        timer.reset();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // only update odometry if op mode is not about to shut down
        // this prevents possible error in calculation cycle
        if (!RobotContainer.ActiveOpMode.isStopRequested()) {

            double leftPos;
            double rightPos;
            double frontPos;

            leftPos = RobotContainer.odometryPod.getLeftEncoderDistance();
            rightPos = RobotContainer.odometryPod.getRightEncoderDistance();
            frontPos = RobotContainer.odometryPod.getFrontEncoderDistance();

            double leftChangePos;
            double rightChangePos;
            double frontChangePos;

            leftChangePos = leftPos - previousLeftPos;
            rightChangePos = rightPos - previousRightPos;
            frontChangePos = frontPos - previousFrontPos;

            previousLeftPos = leftPos;
            previousRightPos = rightPos;
            previousFrontPos = frontPos;

            // creating the value of sin theta (aka the angle of the hipotinuse)
            double theta = Math.asin((rightChangePos - leftChangePos) / RobotContainer.odometryPod.LATERAL_DISTANCE);

            // equation that tells us how much the robot has moved forward
            double ForwardChange = (leftChangePos + rightChangePos) / 2.0;

            // equation that tells us how much the robot has moved laterally
            double LateralChange = (frontChangePos - RobotContainer.odometryPod.FORWARD_OFFSET * Math.sin(theta));// Lateral means left to right

            double IMUHeading = Math.toRadians(RobotContainer.gyro.getYawAngle());

            double fieldForwardChange = ForwardChange * Math.cos(IMUHeading) - LateralChange * Math.sin(IMUHeading);

            double fieldLateralChange = ForwardChange * Math.sin(IMUHeading) + LateralChange * Math.cos(IMUHeading);

            fieldX += fieldForwardChange;// += means is equal to and add fieldForwardChange to itself
            fieldY += fieldLateralChange;// += means is equal to and add fieldLateralChange to itself
            fieldAngle = IMUHeading;

            // update history of positions
            // save most recent position and timestamp
            for (int i=history_array_size-1;i>0;--i)
            {
                fieldHistoryX[i] = fieldHistoryX[i-1];
                fieldHistoryY[i] = fieldHistoryY[i-1];
                fieldHistoryTime[i] = fieldHistoryTime[i-1];
            }
            fieldHistoryX[0] = fieldX;
            fieldHistoryY[0] = fieldY;
            fieldHistoryTime[0] = timer.seconds();

            // process apriltag detections (if any)
            ProcessAprilTags();

            // only update dashboard and controller telemetry if opmode not about to be shut down
            if (!RobotContainer.ActiveOpMode.isStopRequested()) {

                RobotContainer.ActiveOpMode.telemetry.addData("fieldX", fieldX);
                RobotContainer.ActiveOpMode.telemetry.addData("fieldY", fieldY);
                RobotContainer.ActiveOpMode.telemetry.addData("Yaw", Math.toDegrees(fieldAngle));

                // update FTC dashboard with latest odometry info - in separate function below for clarity
                UpdateDashBoard();
            }



            // save position to data store, in case op mode ends
            // check again if op mode not about to shut down - otherwise don't save it
            // Note: this code used to store position and recover it between auto and teleop
            if (!RobotContainer.ActiveOpMode.isStopRequested())
                StoredRobotPose = new Pose2d(fieldX, fieldY, new Rotation2d(fieldAngle));
        }
    }

    // place special subsystem methods here
    public Pose2d getCurrentPos() {
       return new Pose2d(fieldX,fieldY,new Rotation2d(fieldAngle));
    }

    public void setCurrentPos(Pose2d pos){
        fieldX = pos.getX();
        fieldY = pos.getY();
        fieldAngle = pos.getHeading();
        RobotContainer.gyro.setYawAngle(Math.toDegrees(fieldAngle));
    }

    public void resetCurrentPos(){
        setCurrentPos(new Pose2d(0,0,new Rotation2d(0)));
    }


    // ********** AprilTags **********

    // internal function to process apriltags and make any corrections to positions
    private void ProcessAprilTags()
    {
        // assume no tags detected unless determined otherwise
        boolean tagdetected = false;

        // get list of apriltag detections from camera
        List<AprilTagDetection> tags = RobotContainer.tagCamera.GetFreshAprilTagDetections();

        // go through each detection in list
        if (tags!=null)
            for (int i=0; i<tags.size(); ++i)
            {
                // if tag is valid and within range expected for 'into the deep' game.
                if (tags.get(i)!=null && tags.get(i).id >=11 && tags.get(i).id <=16)
                {
                    // for maximum accuracy, check that apriltag completely fits within frame of camera
                    Point[] corners = tags.get(i).corners;
                    if (corners.length == 4 &&
                            corners[0].x >= 10 && corners[0].x <= 630 &&
                            corners[1].x >= 10 && corners[1].x <= 630 &&
                            corners[2].x >= 10 && corners[2].x <= 630 &&
                            corners[3].x >= 10 && corners[3].x <= 630 &&
                            corners[0].y >= 10 && corners[0].y <= 470 &&
                            corners[1].y >= 10 && corners[1].y <= 470 &&
                            corners[2].y >= 10 && corners[2].y <= 470 &&
                            corners[3].y >= 10 && corners[3].y <= 470)
                    {
                        // we detected a valid tag
                        tagdetected = true;

                        // calculate robot position from apriltags (note returns position in inches!)
                        Pose2d newpos = AprilTagUtils.CalculateRobotFieldPose(tags.get(i), 0);

                        // get field history
                        // estimate of frame acquisition time
                        // 1/2*avg typ iteration time (guess 50ms) + (1/frames per second) + shutter time
                        double fps = RobotContainer.tagCamera.GetCameraFPS();
                        double frame_acquisition_time = (0.5*0.05) + 0.002;
                        if (fps!=0.0)
                            frame_acquisition_time += (1.0/fps);

                        // best estimate of when picture was first taken is current - acquisitiontime
                        double frame_time = timer.seconds() - frame_acquisition_time;



                        int j;
                        for (j=0;j<=history_array_size-1;++j) {
                            if (fieldHistoryTime[j]<frame_time)
                                break;
                        }

                        // determine x and y at time picture was taken.
                        double historical_x=fieldX, historical_y=fieldY;
                        // every thing in list is old - use newest in list we have
                        if (j==0)
                            { historical_x = fieldHistoryX[0]; historical_y = fieldHistoryY[0]; }

                        // everything in list is newer- we don't have enough history - use oldest we have
                        else if (j==history_array_size)
                           { historical_x = fieldHistoryX[history_array_size-1]; historical_y = fieldHistoryY[history_array_size-1]; }

                        // otherwise, linearly interpolate
                        else
                        {
                           if ((fieldHistoryTime[j-1]-fieldHistoryTime[j])!=0.0)
                           {
                               historical_x = fieldHistoryX[j]+(fieldHistoryX[j-1]-fieldHistoryX[j])*(frame_time - fieldHistoryTime[j])/(fieldHistoryTime[j-1]-fieldHistoryTime[j]);
                               historical_y = fieldHistoryY[j]+(fieldHistoryY[j-1]-fieldHistoryY[j])*(frame_time - fieldHistoryTime[j])/(fieldHistoryTime[j-1]-fieldHistoryTime[j]);
                           }
                           else
                           { historical_x = fieldHistoryX[j-1]; historical_y = fieldHistoryY[j-1]; }
                        }


                        // differences with recorded position - convert new position from in to m
                        double deltaX = newpos.getX()*0.0254 - historical_x;
                        double deltaY = newpos.getY()*0.0254 - historical_y;

                        // apply fraction of error to our current robot position
                        // applying only small amount filters apriltag results in case of large error
                        // larger value faster convergence but less filtering
                        fieldX += 0.05*deltaX;
                        fieldY += 0.05*deltaY;

                        // need to update all of our historical positions accordingly
                        for (int k=0;k<history_array_size; ++k){
                                fieldHistoryX[k] += 0.05*deltaX;
                                fieldHistoryY[k] += 0.05*deltaY;
                        }

                    }
                }
            }

        // record if a tag is detected
        isTagDetected = tagdetected;
    }

    // returns true if an apriltag is currently detected.
    public boolean isTagDetected() {
        return isTagDetected;
    }

    // ********** Dashboard **********



    // Updates dashboard field widget with robot odometry info
    private void UpdateDashBoard()
    {
        // robot outline (note: values intentionally left in inches)
        // 0,0 is center of robot
        Vector2d p1 = new Vector2d(9, 0);
        Vector2d p2 = new Vector2d(-9, 9);
        Vector2d p3 = new Vector2d(-9, -9);
        Vector2d p4 = new Vector2d(0,0);

        // define robot field position offset vector
        Vector2d pos = new Vector2d(fieldX*39.3701, fieldY*39.3701);

        // rotate outline by angle of odometry and then add x,y position offset
        Vector2d p1rotated=(p1.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);
        Vector2d p2rotated=(p2.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);
        Vector2d p3rotated=(p3.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);
        Vector2d p4rotated=(p4.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);

        // create field telemetry packet
        TelemetryPacket field = new TelemetryPacket();
        field.fieldOverlay()
                .drawGrid(0, 0, 144, 144, 7, 7);

        // set red or blue based on alliance colour
        if (RobotContainer.isRedAlliance)
            field.fieldOverlay().setStroke("red");
        else
            field.fieldOverlay().setStroke("blue");

        // draw robot on field
        field.fieldOverlay()
                .strokeLine(p1rotated.getX(), p1rotated.getY(), p2rotated.getX(), p2rotated.getY())
                .strokeLine(p2rotated.getX(), p2rotated.getY(), p3rotated.getX(), p3rotated.getY())
                .strokeLine(p3rotated.getX(), p3rotated.getY(), p1rotated.getX(), p1rotated.getY())
                .fillCircle(p1rotated.getX(),p1rotated.getY(), 2)
                .fillCircle(p4rotated.getX(),p4rotated.getY(), 1);
        //.fillText("Origin", 0, 0, "4px Arial", Math.toRadians(90), false)
        //.setRotation(Math.toRadians(90))
        //.strokeRect(x,y,width,height)
        //.drawImage("/dash/ftc.jpg", 24, 24, 18, 18, Math.toRadians(90), 24, 24, false);

        // do we have a trajectory to plot?
        if (currentTrajectoryXpoints!=null && currentTrajectoryYpoints!=null)
            field.fieldOverlay().strokePolyline(currentTrajectoryXpoints, currentTrajectoryYpoints);

        // get list of apriltag detections from camera
        List<AprilTagDetection> tags = RobotContainer.tagCamera.GetCurrentAprilTagDetections();
        if (tags!=null)
            for (int i=0; i<tags.size(); ++i)
                if (tags.get(i)!=null) {
                    // calculate robot position from apriltags
                    Pose2d ATPos = AprilTagUtils.CalculateRobotFieldPose(tags.get(i),0);
                    field.fieldOverlay().strokeCircle(ATPos.getX(), ATPos.getY(), 1.0);
                    //RobotContainer.DBTelemetry.addData("Distance", tags.get(i).ftcPose.range);
                    //RobotContainer.DBTelemetry.addData("TagX", tags.get(i).ftcPose.y);
                    //RobotContainer.DBTelemetry.addData("TagY", tags.get(i).ftcPose.x);
                    //RobotContainer.DBTelemetry.addData("Aquisition Time", tags.get(i).frameAcquisitionNanoTime);
                }

        // update field
        RobotContainer.DashBoard.sendTelemetryPacket(field);

        // show robot position on dashboard
        //RobotContainer.DBTelemetry.addData("Robot x pos: ", fieldX);
        //RobotContainer.DBTelemetry.addData("Robot y pos: ", fieldY);
        //RobotContainer.DBTelemetry.addData("Robot angle: ", Math.toDegrees(fieldAngle));
        //RobotContainer.DBTelemetry.addData("Frame Rate", RobotContainer.tagCamera.GetCameraFPS());
        //RobotContainer.DBTelemetry.update();


        // Show data on dashboard
        // double value1 = 1.0;
        // double value2 = 5.0;

        // Method #1
        // RobotContainer.DBTelemetry.addData("Value 1a", value1);
        // RobotContainer.DBTelemetry.addData("Value 2a", value2);
        // RobotContainer.DBTelemetry.update();

        // Method #2
        // TelemetryPacket data = new TelemetryPacket();
        // data.put("Value 1b", value1);
        // data.put("Value 2b", value2);
        // RobotContainer.DashBoard.sendTelemetryPacket(data);
    }

    // display provided trajectory on the dashboard field widget
    // set trajectory to null turn off trajectory
    public void DisplayTrajectory (Trajectory trajectory) {

        if (trajectory!=null)
        {
            // translate provided trajectory into separate x,y points array for use by dashboard field widget
            int length = trajectory.getStates().size();
            currentTrajectoryXpoints = new double[length];
            currentTrajectoryYpoints = new double[length];
            for (int index = 0; index < length; ++index) {
                currentTrajectoryXpoints[index] = 39.3701*trajectory.getStates().get(index).poseMeters.getX();
                currentTrajectoryYpoints[index] = 39.3701*trajectory.getStates().get(index).poseMeters.getY();
            }
        }
        else
        {
            // no trajectory to display - set arrays to null
            currentTrajectoryXpoints = null;
            currentTrajectoryYpoints = null;
        }
    }

}