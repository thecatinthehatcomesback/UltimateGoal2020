package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

    // Motors: //
    public DcMotor intakeMotor = null;
    public Servo transferServo = null;
    static final double transferUp = 0.6;
    static final double transferDown = 0.2;


    // Timers: //
    private ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: //
        intakeMotor = hwMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes: //
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferServo = hwMap.servo.get("transfer");
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
        if(transferServo.getPosition() == transferDown){
            intakeMotor.setPower(power);
        }else{
            intakeMotor.setPower(0);
        }
    }
    public double getJawPower() {
        return intakeMotor.getPower();
    }

    //----------------------------------------------------------------------------------------------
    // transfer Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the transfer to up and down.
     */
    public void transferUp() { transferServo.setPosition(transferUp); }

    public void transferDown() { transferServo.setPosition(transferDown); }

    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = 3.0;
        return !((runtime.seconds() < TIMEOUT));
    }
}