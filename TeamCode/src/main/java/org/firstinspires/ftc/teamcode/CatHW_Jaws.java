package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CatHW_Jaws.java
 *
 *
 * This class containing common code accessing hardware specific to the movement of the jaws/intake.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem
{
    /* Public OpMode members. */
    public static final double JAW_POWER = 0.9;

    // Motors: //
    public DcMotor rightJawMotor = null;

    // Sensors: //
    public DigitalChannel intakeSensor = null;

    // Timers: //
    private ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: //
        rightJawMotor = hwMap.dcMotor.get("right_jaw_motor");
        rightJawMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes: //
        rightJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize sensors: //
        intakeSensor = hwMap.digitalChannel.get("intakeSensor");
    }


    //----------------------------------------------------------------------------------------------
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the power of both jaw motors.
     *
     * @param power at which the motors will spin.
     */
    public void setJawPower(double power) {
        rightJawMotor.setPower(power);
    }

    /**
     * Turn on both jaws motors to suck in at JAW_POWER.
     */
    public void intakeJaws() {
        rightJawMotor.setPower(JAW_POWER);
    }

    /**
     * Turn on both jaws motors to suck in, but has the left motor slightly faster to improve pick
     * up chance on blue side.
     */
    public void intakeJawsBlue() {
        rightJawMotor.setPower(-JAW_POWER - .15);
    }

    /**
     * Turn on both jaws motors to suck in, but has the right motor slightly faster to improve pick
     * up chance on red side.
     */
    public void intakeJawsRed() {
        rightJawMotor.setPower(JAW_POWER);
    }

    /**
     * Turn on both jaws motors to spit out.  Also reduce the power so that we aren't penalized for
     * launching stones.
     */
    public void outputJaws() {
        rightJawMotor.setPower(-JAW_POWER * 0.4);
    }

    /**
     * Turn off both jaws motors.
     */
    public void turnOffJaws() {
        rightJawMotor.setPower(0.0);
    }



    //----------------------------------------------------------------------------------------------
    // Intake Sensor Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * @return whether robot has a stone inside the intake.
     */
    public boolean hasStone() {
        return !intakeSensor.getState();
    }


    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        //Log.d("catbot", String.format("left jaw power %.2f,", leftJawMotor.getPower()));
        Log.d("catbot", String.format("right jaw power %.2f,", rightJawMotor.getPower()));
        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = 3.0;
        return !(rightJawMotor.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}