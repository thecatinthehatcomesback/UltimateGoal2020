package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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
    public static class UltimateGoalPipeline extends OpenCvPipeline
    {
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
        private volatile numRings position = numRings.FOUR;


        /* Enums */
        enum numRings {
            NONE,
            ONE,
            FOUR
        }
        private Deque<numRings> ringValues;
        public numRings avgValue;

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
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

            position = numRings.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = numRings.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = numRings.ONE;
            }else {
                position = numRings.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            if (ringValues.size() > 29) {
                // Make sure we keep the size at a reasonable level
                ringValues.removeFirst();
            }
            ringValues.add(position);
            if (Collections.frequency(ringValues, UltimateGoalPipeline.numRings.ONE) > Collections.frequency(ringValues, UltimateGoalPipeline.numRings.FOUR) &&
                    Collections.frequency(ringValues, UltimateGoalPipeline.numRings.ONE) > Collections.frequency(ringValues, UltimateGoalPipeline.numRings.NONE)) {
                // If the amount of INSIDE readings is the most in the past 30 readings, return INSIDE.
                avgValue=numRings.ONE;
            } else if (Collections.frequency(ringValues, UltimateGoalPipeline.numRings.FOUR) > Collections.frequency(ringValues, UltimateGoalPipeline.numRings.ONE) &&
                    Collections.frequency(ringValues, UltimateGoalPipeline.numRings.FOUR) > Collections.frequency(ringValues, UltimateGoalPipeline.numRings.NONE)) {
                // If the amount of CENTER readings is the most in the past 30 readings, return CENTER.
                avgValue=numRings.FOUR;
            } else {
                // Just return back OUTSIDE since it is the last possible value.
               avgValue=numRings.NONE;
            }

            return input;
        }
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, Cb, 1);
            Imgproc.cvtColor(input,hsv, Imgproc.COLOR_RGB2HSV);
        }



        public int getAnalysis()
        {
            return avg1;
        }

    }


    UltimateGoalPipeline pipeline;
    OpenCvCamera webcam;



    private HardwareMap hwMap   = null;






    /* Constructor */
    public CatHW_Vision(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /**
     * Initializes the "hardware" devices for anything having to do with machine vision.
     *
     * @param ahwMap which contains the hardware to look for.
     */
    public void initVision(HardwareMap ahwMap) {

        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new UltimateGoalPipeline();
        pipeline.ringValues = new ArrayDeque<>(30);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }
        });

    }


    //----------------------------------------------------------------------------------------------
    // Open CV Metthods:
    //----------------------------------------------------------------------------------------------


    /**
     * A new way to take the all the values during the init and choosing the value in the deque that
     * has the most occurrences.
     *
     * @return which SkyStone position is the most likely.
     */
    public UltimateGoalPipeline.numRings getNumRings() {
        return pipeline.avgValue;
    }
}