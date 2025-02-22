package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.AutoPickUpOffGround;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;


// command template
public class MoveToPickup extends CommandBase {
    PIDController xControl;
    PIDController yControl;
    double xError;
    double yError;
    ElapsedTime timer;
    // constructor
    public MoveToPickup() {
        xControl = new PIDController(0.0024, 0.0008, 0.0);  // was d=0.000245
        yControl = new PIDController(0.0024, 0.0008, 0.0);

        timer = new ElapsedTime();
        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        xControl.reset();
        yControl.reset();

        timer.reset();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        double Target_X = 280.0; // was (double) (315 + 330) / 2 before flip
        double Target_Y = 220.0; // was (double) (325 + 315) / 2 before flip // 355.0  / was 320

        // get list of current blobs from camera
        List<ColorBlobLocatorProcessor.Blob> blobList = RobotContainer.clawCamera.GetBlobDetections();

        // get closest blob to camera target
        ColorBlobLocatorProcessor.Blob blob = getClosestBlob(blobList, Target_X, Target_Y);

        // if piece is detected the move robot towards piece
        // otherwise, keep moving as per before
        if (blob!=null) {
            xError = Target_X - blob.getBoxFit().center.x;
            yError = Target_Y - blob.getBoxFit().center.y;

            double xSpeed = xControl.calculate(xError);
            double ySpeed = yControl.calculate(yError);

            RobotContainer.drivesystem.RobotDrive(ySpeed, xSpeed, 0);
//            // determin angle of nearest blob
//            int piece_angle = (int) Math.round(blob.getBoxFit().angle);
//            if (blob.getBoxFit().size.width<blob.getBoxFit().size.height)
//                piece_angle += 90;
//
//            // move wrist for angle
//            RobotContainer.wristRotateServo.RotateTo(piece_angle, 90);
        }
        else {
            xControl.reset();
            yControl.reset();
            RobotContainer.drivesystem.RobotDrive(0, 0, 0);
        }

        if (xError>20 || yError>20) {
            timer.reset();
        }

//        RobotContainer.DBTelemetry.addData("Target X", Target_X);
//        RobotContainer.DBTelemetry.addData("Target Y", Target_Y);
//        RobotContainer.DBTelemetry.addData("xSpeed", xSpeed);
//        RobotContainer.DBTelemetry.addData("ySpeed", ySpeed);
//        RobotContainer.DBTelemetry.update();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        if (timer.seconds()>0.5) {
            RobotContainer.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            return true;
        }else {
            return false;
        }

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0,0,0);
    }


    // helper to return closest blob out of a list
    ColorBlobLocatorProcessor.Blob getClosestBlob(List<ColorBlobLocatorProcessor.Blob> bloblist,
                                                  double Target_X,
                                                  double Target_Y) {

        // do we have a list and list is not empty
        if (bloblist!=null && !bloblist.isEmpty())
        {
            ColorBlobLocatorProcessor.Blob blob = bloblist.get(0);
            double closest = (bloblist.get(0).getBoxFit().center.x- Target_X) * (bloblist.get(0).getBoxFit().center.x- Target_X) +
                    (bloblist.get(0).getBoxFit().center.y- Target_Y) * (bloblist.get(0).getBoxFit().center.y- Target_Y);

            // cycle through list and get blob nearest the target
            for (int i=1; i< bloblist.size(); ++i)
            {
                double dist = (bloblist.get(i).getBoxFit().center.x- Target_X) * (bloblist.get(i).getBoxFit().center.x- Target_X) +
                        (bloblist.get(i).getBoxFit().center.y- Target_Y) * (bloblist.get(i).getBoxFit().center.y- Target_Y);

                // if found a closer blob, then select this one
                if (dist < closest)
                {
                  blob = bloblist.get(i);
                    closest = dist;
                }
            }

            return blob;

        }
        else
            return null;


    }





}