package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * CatHW_DriveOdo.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the drive train using odometry modules as position givers.  This file is used by the
 * new autonomous OpModes to run multiple operations at once with odometry.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * NOTE: All names are lower case and have underscores between words.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_DriveOdo extends CatHW_DriveBase
{
    //----------------------------------------------------------------------------------------------
    // Odometry Module Constants:                               TODO: Are these constants updated???
    //----------------------------------------------------------------------------------------------

    /**
     * The number of encoder ticks per one revolution of the odometry wheel.
     * 8192 ticks for a REV encoder from REV Robotics.
     */
    private static final double ODO_COUNTS_PER_REVOLUTION = 8192;
    /** The measurement of the odometry wheel diameter for use in calculating circumference. */
    private static final double ODO_WHEEL_DIAMETER_INCHES = 2.0;
    /**
     * The amount of odometry encoder ticks equal to movement of 1 inch.  Used for conversion in the
     * robot's positioning algorithms so that when a user inputs (X,Y) coordinates in inches, those
     * can be converted into encoder ticks.
     */
    static final double ODO_COUNTS_PER_INCH     = ODO_COUNTS_PER_REVOLUTION /
            (ODO_WHEEL_DIAMETER_INCHES * Math.PI);


    //TODO: Other attributes/field should get some Javadoc sometime...
    private double targetX;
    private double targetY;
    private double targetTheta;
    private double xMin;
    private double xMax;
    private double yMin;
    private double yMax;
    private double thetaMin;
    private double thetaMax;
    private double lastPower = 0;
    private double maxPower;
    private double strafePower;

    private boolean isNonStop;

    /** Enumerated type for the style of drive the robot will make. */
    private enum DRIVE_METHOD {
        PURE_PURSUIT,
        TURN
    }
    /** Variable to keep track of the DRIVE_METHOD that's the current style of robot's driving. */
    private DRIVE_METHOD currentMethod;

    /** ArrayList of Points that the robot will drive towards. */
    private ArrayList<CatType_CurvePoint> targetPoints;

    /** A default follow radius for our Pure Pursuit Algorithms. */
    private final double DEFAULT_FOLLOW_RADIUS = 20.0;
    /** The following distance between the robot and any point on the line paths. */
    private double followRadius = DEFAULT_FOLLOW_RADIUS;



    //----------------------------------------------------------------------------------------------
    // Public OpMode Members
    //----------------------------------------------------------------------------------------------

    // Motors
    public DcMotor leftOdometry    = null;
    public DcMotor rightOdometry   = null;
    public DcMotor backOdometry    = null;
    //public ExpansionHubEx expansionHub = null;

    // Access to Update Thread
    public CatOdoAllUpdates updatesThread;

    /* Constructor */
    public CatHW_DriveOdo(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /**
     * Initialize standard Hardware interfaces in the CatHW_DriveOdo hardware.
     *
     * @throws InterruptedException in case of error.
     */
    public void init()  throws InterruptedException  {

        // Calls DriveBase's init: //
        super.init();

        // Define and Initialize Motors and Expansion Hub: //
        leftOdometry     = hwMap.dcMotor.get("right_rear_motor");
        rightOdometry    = hwMap.dcMotor.get("right_front_motor");
        backOdometry     = hwMap.dcMotor.get("intake");

        // Set odometry modes: //
        resetOdometryEncoders();

        // Odometry Setup: //
        updatesThread = updatesThread.getInstanceAndInit(hwMap, leftOdometry, rightOdometry,
                backOdometry, ODO_COUNTS_PER_INCH);
        Thread allUpdatesThread = new Thread(updatesThread);
        updatesThread.resetThreads();
        allUpdatesThread.start();

        // Sets enums to a default value: //
        currentMethod = DRIVE_METHOD.PURE_PURSUIT;
    }



    //----------------------------------------------------------------------------------------------
    // Driving Chassis Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the odometry wheels to STOP_AND_RESET_ENCODER.
     */
    public void resetOdometryEncoders() {
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Overloaded method (fancy way of saying same method header with different parameter list) that calls the other
     * pursuitDrive() method, but automatically sets the followRadius to the DEFAULT_FOLLOW_RADIUS constant.
     *
     * @param points is an ArrayList of Points that make up the user-defined path the robot will follow.
     * @param power at which the robot's max speed will be set to using motion profiling.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step. This is
     *         used/useful for stall outs.
     */
    public void pursuitDrive(ArrayList<CatType_CurvePoint> points, double power, double timeoutS) {

        pursuitDrive(points, power, DEFAULT_FOLLOW_RADIUS, timeoutS);
    }

    /**
     * Used to move the robot across the field.  The robot can also TURN while moving along the path in order for it to
     * face a certain by the end of its path.  This method assumes the robot has odometry modules which give an absolute
     * position of the robot on the field.
     *
     * @param points is an ArrayList of Points that make up the user-defined path the robot will follow.
     * @param power at which the robot's max speed will be set to using motion profiling.
     * @param followRadius is the distance between the robot and the point ahead of it on the path that it will
     *         choose to follow.
     * @param timeout is how much time needs to pass before the robot moves onto the next step. This is
     *         used/useful for stall outs.
     */
    public void pursuitDrive(ArrayList<CatType_CurvePoint> points, double power, double followRadius, double timeout) {

        currentMethod = DRIVE_METHOD.PURE_PURSUIT;
        this.timeout = timeout;
        isDone = false;
        targetPoints = points;
        targetPoints.add(0, updatesThread.positionUpdate.returnRobotPointInches().toCurvePoint());
        this.followRadius = followRadius;

        //CatType_CurvePoint targetPoint = updatesThread.powerUpdate.getFollowPoint(targetPoints,
        //        updatesThread.positionUpdate.returnRobotPointInches(), followRadius);


        // Power update Thread:
        if (isNonStop) {
            // If the last drive method call was nonstop:
            updatesThread.powerUpdate.setNonStopTarget(points, power, followRadius);
        } else {
            // If the last drive method call was normal:
            updatesThread.powerUpdate.setTarget(points, power, followRadius);
        }

        // Set it so the next one will be nonstop.
        isNonStop = false;

        // Reset timer once called.
        runTime.reset();
    }

    /**
     * Nonstop TRANSLATE.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param finishedXMin the smallest X value that the drive will consider done at.
     * @param finishedXMax the largest X value that the drive will consider done at.
     * @param finishedYMin the smallest Y value that the drive will consider done at.
     * @param finishedYMax the largest Y value that the drive will consider done at.
     * @param finishedThetaMin the smallest Theta value that the drive will consider done at.
     * @param finishedThetaMax the largest Theta value that the drive will consider done at.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step. This is
     *         used/useful for stall outs.
     */
    public void pursuitDrive(double x, double y, double power, double theta,
                             double finishedXMin, double finishedXMax, double finishedYMin, double finishedYMax,
                             double finishedThetaMin, double finishedThetaMax, double timeoutS) {

        currentMethod = DRIVE_METHOD.PURE_PURSUIT;
        timeout = timeoutS;
        isDone = false;
        //targetX = x;
        //targetY = y;
        //strafePower = power;
        //targetTheta = theta;
        xMin = finishedXMin;
        xMax = finishedXMax;
        yMin = finishedYMin;
        yMax = finishedYMax;
        thetaMin = finishedThetaMin;
        thetaMax = finishedThetaMax;
        maxPower = power;

        isNonStop = false;
        //if the last drive was nonstop
        //updatesThread.powerUpdate.setNonStopTarget(x, y, power);

        // Reset timer once called
        runTime.reset();
    }

    /*public void followCurve(ArrayList<CurvePoint> allPoints, double maxPower, double followAngle,
                            double turnSpeed) {
        //TODO:  Add some debug logs here...
        CurvePoint followThisPoint = updatesThread.powerUpdate.getFollowPoint(allPoints,
                updatesThread.positionUpdate.returnRobotPointInches(), allPoints.get(0).followDistance);
        //TODO:  Going to want to want to think about how we use the followAngle here...
        //goToPosition(followThisPoint.x, followThisPoint.y, followAngle);
        //translateDrive(followThisPoint.x, followThisPoint.y, maxPower, followAngle, turnSpeed, 5.0);
        //TODO:  Add logic to stop at the final point in the array list.
    }*/



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {

        //wait for the update
        while (!updatesThread.positionUpdate.isUpdated){
            mainHW.opMode.sleep(1);
        }
        //set the updated status back to false so it knows that we are now using current powers
        updatesThread.positionUpdate.isUpdated = false;

        boolean keepDriving = true;


        // Exit if timeout is hit.  Helpful for when the robot stalls/stalls out.
        if ((runTime.seconds() > timeout)) {
            Log.d("catbot", "Timed OUT.");
            keepDriving = false;
        }

        switch (currentMethod) {
            case TURN:
                // Current orientation from odometry modules:
                int zVal = getCurrentAngle();

                Log.d("catbot", String.format("target %d, current %d  %s",
                        -targetAngleZ, -zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                break;

            case PURE_PURSUIT:
                // Current robot position and orientation from odometry modules:
                CatType_Point getRobotPos = updatesThread.positionUpdate.returnRobotPointInches();
                double getTheta = updatesThread.positionUpdate.returnOrientation();
                double getPower = updatesThread.powerUpdate.updatePower();

                // Assign the point to follow
                CatType_CurvePoint targetPointOnLine = updatesThread.powerUpdate.getPointOnLine();
                double totalDistRemaining = updatesThread.powerUpdate.getDistanceToFinalTargetPoint();
                int targetPointIndex = updatesThread.powerUpdate.getTargetPoint();

                // Check if ready to end without the isNonStop.
                if (!isNonStop) {
                    if ((Math.abs(targetPoints.get(targetPoints.size()-1).y - getRobotPos.y) < 2 &&
                            Math.abs(targetPoints.get(targetPoints.size()-1).x - getRobotPos.x) < 2) &&
                            (Math.abs(targetPoints.get(targetPoints.size()-1).getTheta() - getTheta) < 5)) {

                        keepDriving = false;
                    }
                } else {

                    // if isNonStop
                    if (xMin < getRobotPos.x && getRobotPos.x < xMax &&
                            yMin < getRobotPos.y && getRobotPos.y < yMax &&
                            thetaMin < getTheta && getTheta < thetaMax){

                        keepDriving = false;
                    }

                    if (lastPower > getPower){
                        getPower = maxPower;
                    }
                }

                lastPower = getPower;


                /*
                Calculate robot angles:
                 */
                double absAngleToTarget = (Math.atan2(targetPointOnLine.x - getRobotPos.x,
                        targetPointOnLine.y - getRobotPos.y));
                double relativeAngleToTarget    = absAngleToTarget - Math.toRadians(getTheta);
                /*
                Calculate robot mecanum wheel powers:
                 */
                double lFrontPower = (Math.cos(relativeAngleToTarget) +
                        Math.sin(relativeAngleToTarget));
                double rFrontPower = (Math.cos(relativeAngleToTarget) -
                        Math.sin(relativeAngleToTarget));
                double lBackPower;
                double rBackPower;

                // Set powers for mecanum wheels:
                lBackPower = rFrontPower;
                rBackPower = lFrontPower;

                double minTP = (updatesThread.powerUpdate.getDistanceToFinalTargetPoint() - 6.0) / -20.0;

                if (minTP > .2){
                    minTP = .2;
                } else if (minTP < 0){
                    minTP = 0;
                }

                if ((getTheta - targetPoints.get(targetPointIndex).theta) < 2) {
                    minTP = 0;
                }

                double turnPower = Math.abs((getTheta - targetPoints.get(targetPointIndex).theta) / 120.0);

                if (turnPower < minTP){
                    turnPower = minTP;
                }
                if (turnPower > .5){
                    turnPower = .5;
                }
                Log.d("catbot",  String.format("minTP: %.2f , TP: %.2f",minTP, turnPower));

                /*
                Calculate scale factor and motor powers:
                 */
                double SF = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                lFrontPower = lFrontPower  * getPower * SF;
                rFrontPower = rFrontPower  * getPower * SF;
                lBackPower  = lBackPower   * getPower * SF;
                rBackPower  = rBackPower   * getPower * SF;

                // Adds TURN powers to each mecanum wheel.
                if ((getTheta - targetPoints.get(targetPointIndex).theta) < 0) {
                    // Turn right
                    rFrontPower = rFrontPower - (turnPower);
                    rBackPower  = rBackPower  - (turnPower);
                    lFrontPower = lFrontPower + (turnPower);
                    lBackPower  = lBackPower  + (turnPower);
                } else {
                    // Turn left
                    rFrontPower = rFrontPower + (turnPower);
                    rBackPower  = rBackPower  + (turnPower);
                    lFrontPower = lFrontPower - (turnPower);
                    lBackPower  = lBackPower  - (turnPower);
                }

                // Calculate scale factor and motor powers:
                double SF2 = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                leftFrontMotor.setPower(lFrontPower  * SF2);
                rightFrontMotor.setPower(rFrontPower * SF2);
                leftRearMotor.setPower(lBackPower    * SF2);
                rightRearMotor.setPower(rBackPower   * SF2);

                Log.d("catbot", String.format("Translate LF:%.2f; RF:%.2f; LR:%.2f; RR:%.2f;" +
                                "   TargetX/Y/Θ: %.2f %.2f %.1f;" +
                                "   AimX/Y/Θ: %.2f %.2f %.1f;" +
                                "   CurrentX/Y/Θ: %.2f %.2f %.1f;  Power: %.2f",
                        leftFrontMotor.getPower(), rightFrontMotor.getPower(),
                        leftRearMotor.getPower(), rightRearMotor.getPower(),
                        targetPoints.get(targetPointIndex).x, targetPoints.get(targetPointIndex).y,
                        targetPoints.get(targetPointIndex).theta,
                        targetPointOnLine.x, targetPointOnLine.y, targetPointOnLine.theta,
                        getRobotPos.x, getRobotPos.y, getTheta, getPower));
                break;
        }

        if (!keepDriving) {
            if (isNonStop){
                isDone = true;
                return true;
            } else {
                // Stop all motion;
                setDrivePowers(0, 0, 0, 0);
                isDone = true;
                return true;
            }
        }
        return isDone;
    }
}