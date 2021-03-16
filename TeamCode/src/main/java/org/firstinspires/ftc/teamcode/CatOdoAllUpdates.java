package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.CatHW_Vision.mmPerInch;

/**
 * CatOdoAllUpdates.java
 *
 *
 * This class spawns a separate thread to keep track the robot encoders using USB bulk reads to get
 * consistent odometry readings which helps for position and power calculations.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatOdoAllUpdates implements Runnable
{
    /** static variable singleInstance of type Singleton. */
    private static CatOdoAllUpdates singleInstance = null;

    /** Thread run condition. */
    private boolean isRunning = true;
    /** The amount of time the thread will rest before running its tasks again. */
    private int sleepTime = 25;

    private OpenGLMatrix lastLocation = null;

    /* Classes that will run off this thread. */
    CatOdoPositionUpdate positionUpdate;
    CatOdoPowerUpdate powerUpdate;

    /**
     * Stops the position update thread.
     */
    public void stop(){ isRunning = false; }

    /* Constructor */
    public CatOdoAllUpdates(HardwareMap hwMap, DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH) {
        positionUpdate = new CatOdoPositionUpdate(hwMap, verticalEncoderLeft, verticalEncoderRight, horizontalEncoder, COUNTS_PER_INCH);
        powerUpdate = new CatOdoPowerUpdate(positionUpdate);
        //positionUpdate.reverseLeftEncoder();
    }

    /**
     * TODO:  Add Javadoc.
     *
     * //@param inExpansionHubIn
     * @param verticalEncoderLeftIn
     * @param verticalEncoderRightIn
     * @param horizontalEncoderIn
     * @param COUNTS_PER_INCHIn
     * @return
     */
    public static CatOdoAllUpdates getInstanceAndInit(HardwareMap hwMap,
                                                      DcMotor verticalEncoderLeftIn,
                                                      DcMotor verticalEncoderRightIn,
                                                      DcMotor horizontalEncoderIn,
                                                      double COUNTS_PER_INCHIn) {
        if (singleInstance == null) {
            singleInstance = new CatOdoAllUpdates(hwMap, verticalEncoderLeftIn,
                    verticalEncoderRightIn, horizontalEncoderIn, COUNTS_PER_INCHIn);
        }

        return singleInstance;
    }

    /**
     * Resets the minimum power and position for the robot.
     */
    public void resetThreads() {
        powerUpdate.resetPowerToNormal();
        positionUpdate.resetPos();
        isRunning = true;
    }

    /**
     * Runs the thread.
     */
    @Override
    public void run() {
        while(isRunning) {
            positionUpdate.globalCoordinatePositionUpdate();
            powerUpdate.updatePower();
            //logVuforia();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    public void logVuforia(){
        // check all the trackable targets to see which one (if any) is visible.
        boolean targetVisible = false;
        CatHW_Vision eyes = CatHW_Async.getInstance().eyes;
        if(eyes == null){
            return;
        }
        if(eyes.allTrackables == null){
            return;
        }
        Log.d("catbot", String.format("Vuforia Logging"));

        for (VuforiaTrackable trackable : eyes.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            Log.d("catbot", String.format("pos x/y/theta %.1f / %.1f / %.1f", translation.get(0)/ mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle));

        }
    }
}
