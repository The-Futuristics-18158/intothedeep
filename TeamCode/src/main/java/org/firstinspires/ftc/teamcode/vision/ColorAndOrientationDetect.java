package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import java.util.ArrayList;
import java.util.List;

public class ColorAndOrientationDetect implements VisionProcessor {

    // Set the YCrCb color ranges for blue based on the slider data
    private final Scalar lowerBlue = new Scalar(55.3, 90.7, 141.7);  // Adjusted blue lower bound
    private final Scalar upperBlue = new Scalar(181.3, 182.8, 255.0); // Adjusted blue upper bound

    // Set the YCrCb color ranges for red based on the slider data
    private final Scalar lowerRed = new Scalar(19.8, 164.0, 68.0);   // Adjusted red lower bound
    private final Scalar upperRed = new Scalar(171.4, 200.0, 120.0); // Adjusted red upper bound

    // Set the YCrCb color ranges for yellow based on the slider data
    private final Scalar lowerYellow = new Scalar(79.3, 140.0, 20.0);  // Adjusted yellow lower bound
    private final Scalar upperYellow = new Scalar(204.0, 225.0, 75.0); // Adjusted yellow upper bound

    // Define a minimum bounding box area (1% of total frame area for 640x480)
    private double MIN_BOUNDING_BOX_AREA = 0.05 * 640 * 480;  // 3072 pixels

    // List to store detected colors with angles
    private final List<DetectedColorWithAngle> detectedColors = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize any necessary variables or settings here
    }

    //set the min detect box , percentage from 0.01-1
    public void setMinBoundingBoxArea(double percentSet){
        MIN_BOUNDING_BOX_AREA = percentSet *640 * 480;
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        detectedColors.clear();  // Clear previous detections

        // Convert the input image from RGB to YCrCb color space for color detection
        Mat ycrcb = new Mat();
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        // Convert the input image to HSV color space for saturation detection
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Step 1: Detect Blue and mask it out
        Mat maskBlue = new Mat();
        Core.inRange(ycrcb, lowerBlue, upperBlue, maskBlue);
        processColorWithOrientation(maskBlue, hsvMat, input, new Scalar(255, 0, 0), "Blue", 500);
        Core.bitwise_not(maskBlue, maskBlue);
        Core.bitwise_and(ycrcb, ycrcb, ycrcb, maskBlue);  // Mask out blue region

        // Step 2: Detect Red on the remaining image
        Mat maskRed = new Mat();
        Core.inRange(ycrcb, lowerRed, upperRed, maskRed);
        processColorWithOrientation(maskRed, hsvMat, input, new Scalar(0, 255, 0), "Red", 500);
        Core.bitwise_not(maskRed, maskRed);
        Core.bitwise_and(ycrcb, ycrcb, ycrcb, maskRed);  // Mask out red region

        // Step 3: Detect Yellow on the remaining image
        Mat maskYellow = new Mat();
        Core.inRange(ycrcb, lowerYellow, upperYellow, maskYellow);
        processColorWithOrientation(maskYellow, hsvMat, input, new Scalar(0, 255, 255), "Yellow", 500);
        Core.bitwise_not(maskYellow, maskYellow);
        Core.bitwise_and(ycrcb, ycrcb, ycrcb, maskYellow);  // Mask out yellow region

        // Release Mat objects to prevent memory leaks
        ycrcb.release();
        hsvMat.release();
        maskBlue.release();
        maskRed.release();
        maskYellow.release();

        return null;
    }

    private void processColorWithOrientation(Mat mask, Mat hsvMat, Mat frame, Scalar boxColor, String colorName, int minArea) {
        // Find contours on the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Get bounding box for the contour
            Rect rect = Imgproc.boundingRect(contour);

            // Calculate bounding box area
            double boundingBoxArea = rect.width * rect.height;

            // Filter out small contours by bounding box area
            if (boundingBoxArea > MIN_BOUNDING_BOX_AREA) {
                // Get the average saturation of this region
                double avgSaturation = getAvgSaturation(hsvMat, rect);

                // Get the orientation of this object
                double angle = getAngle(contour);

                // Calculate the center of the bounding box
                Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

                // Create a new DetectedColorWithAngle and add it to the list
                DetectedColorWithAngle detectedColor = new DetectedColorWithAngle(
                        colorName, rect, boundingBoxArea, angle, new double[]{avgSaturation, 0, 0}); // Sample color values
                detectedColors.add(detectedColor);

                // Draw the bounding box around the region
                Imgproc.rectangle(frame, rect, boxColor, 2);

                // Put text label near the bounding box with the color name, saturation value, and orientation
                String label = colorName + ": " + String.format("%.2f deg", angle);
                Imgproc.putText(frame, label, new Point(rect.x, rect.y - 5), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, boxColor, 2);
            }
        }

        // Release resources
        hierarchy.release();
        for (MatOfPoint contour : contours) {
            contour.release();
        }
    }

    // Function to get the average saturation in a given region
    protected double getAvgSaturation(Mat hsvInput, Rect rect) {
        Mat submat = hsvInput.submat(rect);
        Scalar meanVal = Core.mean(submat);
        submat.release();
        return meanVal.val[1]; // Return the saturation channel value
    }

    // Function to detect the orientation of the object and return angle in degrees
    private double getAngle(MatOfPoint contour) {
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
        contour2f.release();

        double angle = rotatedRect.angle;
        if (rotatedRect.size.width < rotatedRect.size.height) {
            angle += 90;  // Adjust for consistent upright angle
        }
        return angle;
    }

    // Public method to retrieve the list of detected colors with angles
    public List<DetectedColorWithAngle> getDetectedColorsAndAng() {
        return detectedColors;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx , float scaleCanvasDensity, Object userContext) {
        // Optional: Implement drawing logic on canvas
    }
}
