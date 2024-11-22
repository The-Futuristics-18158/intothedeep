package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.vision.ColorAndOrientationDetect;
import org.firstinspires.ftc.teamcode.vision.DetectedAngle;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

/** Subsystem */
public class ClawCamera extends SubsystemBase {

    // Used for managing the color detection process.
    private ColorAndOrientationDetect myColorAndOrienDetProcessor;
    // Local objects and variables here
    private final VisionPortal CameraPortal;

    private boolean dashboardInitialized = false;

    /** Place code here to initialize subsystem */
    public ClawCamera(String cameraName) {
        myColorAndOrienDetProcessor = new ColorAndOrientationDetect();
        myColorAndOrienDetProcessor.setMinBoundingBoxArea(0.05);

        CameraPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .addProcessors(myColorAndOrienDetProcessor) // add all the processors here
                .setCameraResolution(new Size(640,480))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        RobotContainer.DashBoard.startCameraStream(CameraPortal, 0);
    }

    @Override
    public void periodic() {
//        double ang = myColorAndOrienDetProcessor.calAngle("Blue");
//        if(Math.abs(ang + 999.99)<1e-3 ){
//            System.out.println(" ang1 " + ang  );
//        }
//        else
//        {
//            System.out.println(" ang2 " + ang  );
//        }
//        if (!dashboardInitialized) {
//            initializeDashboard();
//            dashboardInitialized = true;
//        }
//        updateDashboard();
    }

    // Method to initialize dashboard with default (null) values
    private void initializeDashboard() {
    }

    // Method to update the dashboard with actual detected values
    public void updateDashboard() {
    }


    // use to turn on/off AprilTag processing
    public void EnableColorDetectProcessing (boolean enable) {
        CameraPortal.setProcessorEnabled(myColorAndOrienDetProcessor, enable);
    }
}