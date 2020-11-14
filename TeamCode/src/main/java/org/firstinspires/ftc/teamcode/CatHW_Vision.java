package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String STONE_LABEL = "Stone";
    private static final String SKYSTONE_LABEL = "Skystone";

    private HardwareMap hwMap   = null;

    /* Enums */
    enum skyStonePos {
        INSIDE,
        CENTER,
        OUTSIDE
    }

    private Deque<skyStonePos> skyStoneValues;

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
    public void initVision(HardwareMap ahwMap) {
        hwMap = ahwMap;
        skyStoneValues = new ArrayDeque<>(30);

        /*
        Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        // Use the Optical Zoom Camera.
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 2");

        //  Instantiate the Vuforia engine.
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        // Now init the tfod.
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE_LABEL, SKYSTONE_LABEL);
        // And now ACTIVATE!!!
        tfod.activate();
    }

    /**
     * Newest way to continuously look for the SkyStone while looping INSIDE the autonomous init
     * mode but limits the amount of positional occurrences we keep using a Deque.
     */
    public void findSkyStone() {
        if (skyStoneValues.size() > 29) {
            // Make sure we keep the size at a reasonable level
            skyStoneValues.removeFirst();
        }
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {

                for (Recognition recognition : updatedRecognitions) {

                    if (recognition.getLabel().equals(SKYSTONE_LABEL)) {
                        int skyStoneX = (int) ((recognition.getLeft()+recognition.getRight()) / 2);
                        lastLeft = recognition.getLeft();
                        lastRight = recognition.getRight();
                        lastConfidence = recognition.getConfidence();
                        // Look for the SkyStone position and decide where it is.
                        if (skyStoneX > 300) {
                            skyStoneValues.add(skyStonePos.OUTSIDE);
                            return;
                        } else if (skyStoneX > 135) {
                            skyStoneValues.add(skyStonePos.CENTER);
                            return;
                        } else {
                            skyStoneValues.add(skyStonePos.INSIDE);
                            return;
                        }

                    }
                    skyStoneValues.add(skyStonePos.INSIDE);
                    return;
                }
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
    public skyStonePos giveSkyStonePos() {
        Log.d("catbot", String.format("giveSamplePos:  INSIDE: %d, CENTER: %d, OUTSIDE: &d",
                Collections.frequency(skyStoneValues, skyStonePos.INSIDE),
                Collections.frequency(skyStoneValues, skyStonePos.CENTER),
                Collections.frequency(skyStoneValues, skyStonePos.OUTSIDE)));

        // Check to see which value has the most occurrences in the deque
        if (Collections.frequency(skyStoneValues, skyStonePos.INSIDE) > Collections.frequency(skyStoneValues, skyStonePos.CENTER) &&
                Collections.frequency(skyStoneValues, skyStonePos.INSIDE) > Collections.frequency(skyStoneValues, skyStonePos.OUTSIDE)) {
            // If the amount of INSIDE readings is the most in the past 30 readings, return INSIDE.
            return skyStonePos.INSIDE;
        } else if (Collections.frequency(skyStoneValues, skyStonePos.CENTER) > Collections.frequency(skyStoneValues, skyStonePos.INSIDE) &&
                Collections.frequency(skyStoneValues, skyStonePos.CENTER) > Collections.frequency(skyStoneValues, skyStonePos.OUTSIDE)) {
            // If the amount of CENTER readings is the most in the past 30 readings, return CENTER.
            return skyStonePos.CENTER;
        } else {
            // Just return back OUTSIDE since it is the last possible value.
            return skyStonePos.OUTSIDE;
        }
    }
}