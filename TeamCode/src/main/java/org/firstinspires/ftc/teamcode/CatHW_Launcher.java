package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CatHW_Launcher.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the Launcher.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Launcher extends CatHW_Subsystem
{

    // Motors:
    public DcMotor launcher     = null;
    public Servo stopper   = null;
    public Servo aimer   = null;
    private double launchPower = 0.52;
    private boolean isOn = false;

    /* local OpMode members. */
    // Timers:
    private ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Launcher(CatHW_Async mainHardware) {
        super(mainHardware);
    }


    /**
     * Initialize standard Hardware interfaces.
     */
    public void init() {

        // Define and Initialize Motors and Servos: //
        launcher        = hwMap.get(DcMotorEx.class, "launcher");
        stopper    = hwMap.servo.get("ring_stopper");
        aimer    = hwMap.servo.get("aimer");

        // Set Motor and Servo Directions: //
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Motor and Servo Modes: //
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    //----------------------------------------------------------------------------------------------
    // launcher Methods:
    //----------------------------------------------------------------------------------------------

    /**
     *  Turning On and Off the launched wheel
     */

    public void togglePower () {
        if (isOn) {
            powerOff();
        } else {
            powerOn();
        }
    }

    public void powerOn() {
        isOn = true;
        launcher.setPower(launchPower);
    }

    public void updatePower () {
        if (isOn) {
            launcher.setPower(launchPower);
        }
    }

    public void presetPowerShot () {
        launchPower = 0.47;
    }

    public void presetGoal () {
        launchPower = 0.52;
    }

    public void powerOff() {
        isOn = false;
        launcher.setPower(0.0);
    }

    public double getPower () {
        return launchPower;
    }

    public void increasePower () {
        launchPower = launchPower + 0.02;
    }

    public void decreasePower () {
        launchPower = launchPower - 0.02;
    }

    /**
     * Opens and Closes stopper.
     */
    public void openLauncher () {
        stopper.setPosition(0.0);
    }

    public void closeLauncher () {
        stopper.setPosition(1.0);
    }
    /**
     * right and left for the aimer.
     */
    public void aimL () {
        aimer.setPosition(1.0);
    }

    public void aimR () {
        aimer.setPosition(0.0);
    }

    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("launcher power %.2f,", launcher.getPower()));
        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = 3.0;
        return !(launcher.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}// End of class bracket