package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
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
    public DcMotorEx launcher     = null;
    public Servo fireServo   = null;
    private CRServo turretAim = null;
    private double launchRPM = 2350;
    private boolean isOn = false;
    private double targetAngle = 0;
    private double currentAngle = 0;
    private DcMotor turretEncoder = null;
    private launchStateMachine launchSM = null;

    //8192 ticks of the encoder and 64 tooth to 18 tooth gear
    static final double ticksPerDegree = 8192*(64/18)/360*1.2;
    static final double firePosistion = 0.77;
    static final double backPosistion = 0.38;

    public static CatHW_Launcher instance = null;



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
        fireServo       = hwMap.get(Servo.class,"fire_servo");
        turretAim       = hwMap.get(CRServo.class,"aimer");
        turretEncoder   = hwMap.get(DcMotor.class,"right_front_motor");


        // Set Motor Directions: //
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretAim.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set Motor and Servo Modes: //
        //launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //coef = launchWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotConstants.LAUNCH_PID.p = 30;
        RobotConstants.LAUNCH_PID.i = 0.01;
        RobotConstants.LAUNCH_PID.d = 0;
        RobotConstants.LAUNCH_PID.f = 14;
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,RobotConstants.LAUNCH_PID);

        //sets up turret stuff
        targetAngle = 0;
        currentAngle = 0;
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchSM = new launchStateMachine();

        instance = this;

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
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,RobotConstants.LAUNCH_PID);
        launcher.setVelocity(launchRPM / 60 * 28);
    }

    public void updatePower () {
        if (isOn) {
            launcher.setVelocity(launchRPM / 60 * 28);
        }
    }

    public void presetPowerShot () {
        launchRPM = 2150;
        updatePower();
    }

    public double getLaunchRPM() {
        return launchRPM;
    }

    public double getCurrentRPM(){
        return (launcher.getVelocity()* 60 / 28);
    }

    public void presetGoal () {
        launchRPM = 2350;
        updatePower();
    }

    public void powerOff() {
        isOn = false;
        launcher.setPower(0.0);
    }

    public void increasePower () {
        launchRPM = launchRPM + 30;
        updatePower();
    }

    public void decreasePower () {
        launchRPM = launchRPM - 30;
        updatePower();
    }



    public void shootPowerShots(){
        mainHW.robotWait(.7);

        launch();
        mainHW.robotWait(.5);
        launch();
        mainHW.robotWait(.5);
        launch();
        mainHW.robotWait(.5);

        powerOff();
    }

    public void shootHighGoal(){
        mainHW.robotWait(.7);

        launch();
        mainHW.robotWait(.5);
        launch();
        mainHW.robotWait(.5);
        launch();
        mainHW.robotWait(.5);

        powerOff();
    }

    public void waitForShot (){
        ElapsedTime timeOut = new ElapsedTime();
        double prevRPM = getCurrentRPM();
        double currRPM = getCurrentRPM();
        timeOut.reset();
        while((prevRPM - currRPM) < 200){
            prevRPM = currRPM;
            mainHW.robotWait(.05);
            currRPM = getCurrentRPM();
            if((timeOut.seconds() > 1) || !mainHW.opMode.opModeIsActive()){
                break;
            }
        }
    }
    public void launch(){
        launchSM.launch();

    }
    public void setTargetAngle(double newAngle){
        targetAngle = newAngle;

    }
    public double getTargetAngle(){
        return targetAngle;
    }
    //this is called in a seperate thread multiple times per second
    public void doUpdates(){
        //calculate turret angle
        currentAngle = turretEncoder.getCurrentPosition()/ticksPerDegree;
        //update the turret posistion and motor
        if(Math.abs(currentAngle-targetAngle)<1.0){
            turretAim.setPower(0);
        }else if(currentAngle>targetAngle){
            turretAim.setPower(-0.5);
        }else{
            turretAim.setPower(0.5);
        }
        //update launcher servo
        launchSM.update();


    }
    private enum LaunchStates{
        IDLE,
        MOVINGFORWARD,
        MOVINGBACK

    }
    class launchStateMachine{
         private LaunchStates state = LaunchStates.IDLE;
         private ElapsedTime timer = new ElapsedTime();

         public void update(){
             switch(state){
                 case IDLE:
                     break;
                 case MOVINGFORWARD:
                     if(timer.seconds()>0.3){
                         state = LaunchStates.MOVINGBACK;
                         fireServo.setPosition(backPosistion);
                         timer.reset();
                     }
                     break;
                 case MOVINGBACK:
                     if(timer.seconds()>0.35){
                         state = LaunchStates.IDLE;
                         timer.reset();
                     }
                     break;
             }
         }
         public void launch(){
             if(state == LaunchStates.IDLE) {
                 fireServo.setPosition(firePosistion);
                 timer.reset();
                 state = LaunchStates.MOVINGFORWARD;
             }
         }
    }

    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("launcher power %.2f,", launcher.getPower()));
        /* isDone stuff for CatHW_Jaws */
        return true;
    }
}// End of class bracket