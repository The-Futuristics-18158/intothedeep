package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.util.Size;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


/** Subsystem */
public class Camera extends SubsystemBase {

    // Camera vision portal
    private volatile VisionPortal visionPortal;

    // Available vision processors to be used with camera
    // create a processor for each colour of field object
    private volatile AprilTagProcessor aprilTagProcessor;
    private volatile ColorBlobLocatorProcessor redBlobProcessor;
    private volatile ColorBlobLocatorProcessor blueBlobProcessor;
    private volatile ColorBlobLocatorProcessor yellowBlobProcessor;

    // current selected vision mode
    private VisionProcessorMode currentMode = VisionProcessorMode.NONE;

    // camera stream enable/disabled status
    private boolean isCameraStreamEnabled;

    /** Place code here to initialize subsystem */
    public Camera(String cameraName) {

        // Build the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                //.setNumThreads(tbd)
                .build();

        // set default apriltag resolution decimation factor
        SetDecimation(2);

        // Build the RED blob vision processor
        redBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                //.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                //                    new Scalar(19.8, 164.0, 68.0),
                //                    new Scalar(171.4, 200.0, 120.0)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
                .setBlurSize(4)
                .setErodeSize(4)
                .setDilateSize(4)
                .setDrawContours(true)
                .build();
        redBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 300, 300000));
        redBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.25, 2.75));
        redBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.7, 1.0));
        redBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));

        // Build the BLUE blob vision processor
        blueBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                //.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                //                    new Scalar(55.3, 90.7, 141.7),
                //                    new Scalar(181.3, 182.8, 255.0)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
                .setBlurSize(4)
                .setErodeSize(4)
                .setDilateSize(4)
                .setDrawContours(true)
                .build();
        blueBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 300, 300000));
        blueBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.25, 2.75));
        blueBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.7, 1.0));
        blueBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));

        // Build the yellow blob vision processor
        yellowBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                //.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                //                    new Scalar(79.3, 140.0, 20.0),
                //                    new Scalar(204.0, 225.0, 75.0)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
                .setBlurSize(4)
                .setErodeSize(4)
                .setDilateSize(4)
                .setDrawContours(true)
                .build();
        yellowBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 300, 300000));
        yellowBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.25, 2.75));
        yellowBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.7, 1.0));
        yellowBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));

        // Build the camera vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720)) if have an HD camera
                .addProcessors(aprilTagProcessor, redBlobProcessor, blueBlobProcessor, yellowBlobProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // by default disable all processors
        setVisionProcessingMode(VisionProcessorMode.NONE);

        enableDashBoardView(false);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // use this function to set vision processing mode of the camera
    public void setVisionProcessingMode(@NonNull VisionProcessorMode newMode) {

        // record if DB stream is currently enabled
        boolean isViewEnabled = isCameraStreamEnabled;

        enableDashBoardView(false);
        RobotContainer.ActiveOpMode.sleep(100);

        // enable/disable modes depending on vision processing mode selected
        //visionPortal.stopStreaming();
        switch (newMode) {
            case APRIL_TAG_ONLY:
                currentMode = VisionProcessorMode.APRIL_TAG_ONLY;
                //setCameraExposure(10, 200);
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                visionPortal.setProcessorEnabled(redBlobProcessor, false);
                visionPortal.setProcessorEnabled(blueBlobProcessor, false);
                visionPortal.setProcessorEnabled(yellowBlobProcessor, false);

                break;
            case RED_BLOB_ONLY:
                currentMode = VisionProcessorMode.RED_BLOB_ONLY;
                //setCameraExposure(60, 350);
                //SetAutoCameraExposure();
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                visionPortal.setProcessorEnabled(redBlobProcessor, true);
                visionPortal.setProcessorEnabled(blueBlobProcessor, false);
                visionPortal.setProcessorEnabled(yellowBlobProcessor, false);
                break;
            case BLUE_BLOB_ONLY:
                currentMode = VisionProcessorMode.BLUE_BLOB_ONLY;
                //setCameraExposure(60, 350);
                //SetAutoCameraExposure();
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                visionPortal.setProcessorEnabled(redBlobProcessor, false);
                visionPortal.setProcessorEnabled(blueBlobProcessor, true);
                visionPortal.setProcessorEnabled(yellowBlobProcessor, false);
                break;
            case YELLOW_BLOB_ONLY:
                currentMode = VisionProcessorMode.YELLOW_BLOB_ONLY;
                //setCameraExposure(60, 350);
                // SetAutoCameraExposure();
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                visionPortal.setProcessorEnabled(redBlobProcessor, false);
                visionPortal.setProcessorEnabled(blueBlobProcessor, false);
                visionPortal.setProcessorEnabled(yellowBlobProcessor, true);
                break;
            case NONE:
                currentMode = VisionProcessorMode.NONE;
                //SetAutoCameraExposure();
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                visionPortal.setProcessorEnabled(redBlobProcessor, false);
                visionPortal.setProcessorEnabled(blueBlobProcessor, false);
                visionPortal.setProcessorEnabled(yellowBlobProcessor, false);
                break;
            default:
                // don't change anything
        }
        RobotContainer.ActiveOpMode.sleep(100);

        // if DB view was previously enabled then enable it again.
        if (isViewEnabled)
            enableDashBoardView(true);
    }



    // ---------- Apriltag Access functions ----------

    // get fresh AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<AprilTagDetection> GetFreshAprilTagDetections() {
        if (currentMode==VisionProcessorMode.APRIL_TAG_ONLY)
            return aprilTagProcessor.getFreshDetections();
        else
            return new ArrayList<>();
    }

    // get current AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<AprilTagDetection> GetCurrentAprilTagDetections() {
        if (currentMode==VisionProcessorMode.APRIL_TAG_ONLY)
            return aprilTagProcessor.getDetections();
        else
            return new ArrayList<>();
    }

    // sets decimation of AprilTag processing
    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    public void SetDecimation (int num) {
        aprilTagProcessor.setDecimation(num);
    }


    // ---------- Colour blob access functions ----------

    // get color blob detections (if any) from camera
    // returns list containing info on each tag detected
    public List<ColorBlobLocatorProcessor.Blob> GetBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case RED_BLOB_ONLY:
               blobs = redBlobProcessor.getBlobs();
               break;
            case BLUE_BLOB_ONLY:
                blobs = blueBlobProcessor.getBlobs();
                break;
            case YELLOW_BLOB_ONLY:
                blobs = yellowBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // available filtering functions
        //ColorBlobLocatorProcessor.Util.filterByArea(25, 300000, blobs);
        //ColorBlobLocatorProcessor.Util.filterByDensity(0.10, 1.0, blobs);
        //ColorBlobLocatorProcessor.Util.filterByAspectRatio(0.1, 10.0, blobs);

        // available sorting functions
        //ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
        //ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
        //ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);

        return blobs;
    }


    // ---------- General Camera access functions

    // enables camera view in dashboard
    public void enableDashBoardView(boolean enable) {

        // record status
        isCameraStreamEnabled = enable;

        if (enable)
            RobotContainer.DashBoard.startCameraStream(visionPortal, 0);
        else
            RobotContainer.DashBoard.stopCameraStream();
    }

    public void enableLiveView(boolean enable) {
        if (enable)
            visionPortal.resumeLiveView();
        else
            visionPortal.stopLiveView();
    }

    // get camera frames per second
    public double GetCameraFPS () {
        return visionPortal.getFps();
    }

    public boolean cameraReady(){
        return( visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    /** Set the camera gain and exposure. */
    public void setCameraExposure(int exposureMS, int gain) {

        // wait until camera in streaming mode
        while (visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
        {}

        // set exposure control to manual
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

            RobotContainer.ActiveOpMode.sleep(50);
        }

        // set exposure and gain
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        RobotContainer.ActiveOpMode.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RobotContainer.ActiveOpMode.sleep(20);
    }

    /** Sets the camera exposure to automatic */
    public void SetAutoCameraExposure() {

        // wait until camera in streaming mode
        while (visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
        {}

        // set camera to auto
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }


}