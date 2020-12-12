package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

/**
 * CatHW_Vision.java
 *
 *
 * A "hardware" class intended to contain common code for accessing camera and other vision related
 * situations.  While previous versions were made to mostly to test various forms of machine vision,
 * this version uses the Tensor Flow system from the FTC SDK to detect the SkyStones during init in
 * our autonomous routines. We've also tested Vuforia.  TODO:  Check if this is correct...
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Vision extends CatHW_Subsystem
{
    /** Constant unit converter of millimeters to inches. */
    private static final float mmPerInch        = 25.4f;
    /** The width of the FTC field (from the center point to the outer panels) */
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;
    /** The height of the center of the target image above the floor */
    private static final float mmTargetHeight   = (6) * mmPerInch;



    private HardwareMap hwMap   = null;

    public static int regionWidth = 80;
    public static int regionHeight = 80;
    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

    static int REGION_WIDTH = regionWidth;
    static int REGION_HEIGHT = regionHeight;

    final int FOUR_RING_THRESHOLD = 80;
    final int ONE_RING_THRESHOLD = 55;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + regionWidth,
            REGION1_TOPLEFT_ANCHOR_POINT.y + regionHeight);

    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;
    Mat hsv = new Mat();

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;


    /* Enums */
    enum numRings {
        NONE,
        ONE,
        FOUR
    }

    private Deque<numRings> ringValues;

    public double lastLeft;
    public double lastRight;
    public double lastConfidence;

    // Objects and Detectors
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /* Constructor */
    public CatHW_Vision(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /**
     * Initializes the "hardware" devices for anything having to do with machine vision.
     *
     * @param ahwMap which contains the hardware to look for.
     */
    public void initVision(HardwareMap ahwMap, Mat firstFrame) {

        hwMap = ahwMap;
        ringValues = new ArrayDeque<>(30);


        /*
        Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        // Use the Optical Zoom Camera.
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine.
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        // Now init the tfod.
        /*int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE_LABEL, SKYSTONE_LABEL);
        // And now ACTIVATE!!!
        tfod.activate();*/
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }


    //----------------------------------------------------------------------------------------------
    // Open CV Metthods:
    //----------------------------------------------------------------------------------------------

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsv, Cb, 1);
        Imgproc.cvtColor(input,hsv, Imgproc.COLOR_RGB2HSV);
    }

    //@Override
    public Mat processFrame(Mat input)
    {
        region1_pointB.x = REGION1_TOPLEFT_ANCHOR_POINT.x + regionWidth;
        region1_pointB.y = REGION1_TOPLEFT_ANCHOR_POINT.y + regionHeight;

        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
        if(avg1 > FOUR_RING_THRESHOLD){
            position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE;
        }else {
            position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis()
    {
        return avg1;
    }

    /**
     * Newest way to continuously look for the SkyStone while looping INSIDE the autonomous init
     * mode but limits the amount of positional occurrences we keep using a Deque.
     */
    public void findRingNum() {
        if (ringValues.size() > 29) {
            // Make sure we keep the size at a reasonable level
            ringValues.removeFirst();
        }
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {

            /*for (Recognition recognition : updatedRecognitions) {

                if (recognition.getLabel().equals(SKYSTONE_LABEL)) {
                    int skyStoneX = (int) ((recognition.getLeft()+recognition.getRight()) / 2);
                    lastLeft = recognition.getLeft();
                    lastRight = recognition.getRight();
                    lastConfidence = recognition.getConfidence();
                    // Look for the SkyStone position and decide where it is.
                    if (skyStoneX > 300) {
                        ringValues.add(numRings.OUTSIDE);
                        return;
                    } else if (skyStoneX > 135) {
                        ringValues.add(numRings.CENTER);
                        return;
                    } else {
                        ringValues.add(numRings.INSIDE);
                        return;
                    }

                }
                ringValues.add(numRings.INSIDE);
                return;
            }*/
        }
        /*
         Since camera is only looking at the OUTSIDE and CENTER values, it will return INSIDE if is
         doesn't see the SkyStone (basic logic).
         */
    }

    /**
     * A new way to take the all the values during the init and choosing the value in the deque that
     * has the most occurrences.
     *
     * @return which SkyStone position is the most likely.
     */
    public numRings giveSkyStonePos() {
        Log.d("catbot", String.format("giveSamplePos:  INSIDE: %d, CENTER: %d, OUTSIDE: &d",
                Collections.frequency(ringValues, numRings.ONE),
                Collections.frequency(ringValues, numRings.FOUR),
                Collections.frequency(ringValues, numRings.NONE)));

        // Check to see which value has the most occurrences in the deque
        if (Collections.frequency(ringValues, numRings.ONE) > Collections.frequency(ringValues, numRings.FOUR) &&
                Collections.frequency(ringValues, numRings.ONE) > Collections.frequency(ringValues, numRings.NONE)) {
            // If the amount of INSIDE readings is the most in the past 30 readings, return INSIDE.
            return numRings.ONE;
        } else if (Collections.frequency(ringValues, numRings.FOUR) > Collections.frequency(ringValues, numRings.ONE) &&
                Collections.frequency(ringValues, numRings.FOUR) > Collections.frequency(ringValues, numRings.NONE)) {
            // If the amount of CENTER readings is the most in the past 30 readings, return CENTER.
            return numRings.FOUR;
        } else {
            // Just return back OUTSIDE since it is the last possible value.
            return numRings.NONE;
        }
    }
}