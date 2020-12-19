package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CatHW_Tail.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the tail/stacker.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Tail extends CatHW_Subsystem
{
    /* OpMode members. */
    private static final double GRABBER_OPEN = 0.52;
    private static final double GRABBER_CLOSE = 0.2;
    private static final int ARM_DOWN = 650;
    private static final int ARM_MIDDLE = 200;
    private static final int ARM_UP = 0;

    // Motors:
    public DcMotor tailLift     = null;
    public Servo grabberServo   = null;

    /* local OpMode members. */
    private boolean isGrab = false;
    // Timers:
    private ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Tail(CatHW_Async mainHardware) {
        super(mainHardware);
    }


    /**
     * Initialize standard Hardware interfaces.
     */
    public void init() {

        // Define and Initialize Motors and Servos: //
        tailLift        = hwMap.dcMotor.get("tail_lift");
        grabberServo    = hwMap.servo.get("grabber_servo");

        // Set Motor and Servo Directions: //
        tailLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Motor and Servo Modes: //
        tailLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tailLift.setTargetPosition(0);
        tailLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    //----------------------------------------------------------------------------------------------
    // Stacker Methods:
    //----------------------------------------------------------------------------------------------


    //public void setTailPower (double tailPower) {
       // tailLift.setPower(tailPower);
    //}

    public void setArmDown (){

        tailLift.setTargetPosition(ARM_DOWN);
        tailLift.setPower(0.3);
    }
    public void setArmMiddle(){

        tailLift.setTargetPosition(ARM_MIDDLE);
        tailLift.setPower(0.3);

    }
    public  void setArmUp(){

        tailLift.setTargetPosition(ARM_UP);
        tailLift.setPower(0.3);
    }

    //Toggle Power
    public void toggleGrab () {
        if (isGrab) {
            closeGrabber();
        } else {
            openGrabber();
        }
    }

    /**
     * Closes the wobble grabber.
     */
    public void closeGrabber() {
        grabberServo.setPosition(GRABBER_CLOSE);
        isGrab = false;
    }

    /**
     * Opens the wobble grabber.
     */
    public void openGrabber() {
        grabberServo.setPosition(GRABBER_OPEN);
        isGrab = true;
    }

    public void checkMotor(){
        if(!tailLift.isBusy()){
            if(tailLift.getTargetPosition() == ARM_UP){
                tailLift.setPower(0);

            }
        }
    }



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("tail lift power %.2f,", tailLift.getPower()));
        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = 3.0;
        checkMotor();
        return !(tailLift.isBusy() && (runtime.seconds() < TIMEOUT));

    }
}// End of class bracket