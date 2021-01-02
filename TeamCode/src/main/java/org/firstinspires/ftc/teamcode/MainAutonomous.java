package org.firstinspires.ftc.teamcode;/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

*/

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MainAutonomous.java
 *
 *
 * A Linear OpMode class to be an autonomous method for both Blue & Red alliance sides where we pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 * Also will detect the position and deliver the skystone using machine vision and move the
 * foundation.
 *
 * Mec_Odo_AutonomousLevel6_Statey is written to use machine vision and SkyStone delivery to our
 * autonomous route with the help intake jaws that suck in a stone at any orientation using a
 * "touch it-own it" approach.  A servo and two motors make up TC-73/Bucky's arm and stack stones as
 * well as our team marker.

 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="MainAutonomous", group="CatAuto")

public class MainAutonomous extends LinearOpMode
{

/* Declare OpMode members. */

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;



    @Override
    public void runOpMode() throws InterruptedException {


/*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this, true);



/*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isRedAlliance) {
                    isRedAlliance = false;
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }



            /**
             * LED code:
             */

            if (robot.eyes.getNumRings() == CatHW_Vision.UltimateGoalPipeline.numRings.NONE) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            }
            if (robot.eyes.getNumRings() == CatHW_Vision.UltimateGoalPipeline.numRings.ONE) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            }
            if (robot.eyes.getNumRings() == CatHW_Vision.UltimateGoalPipeline.numRings.FOUR) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
            }


            /**
             * Telemetry while waiting for PLAY:
             */

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
            telemetry.addData("Num of Rings", "%s",robot.eyes.getNumRings().toString());
            dashboardTelemetry.addData("Num of Rings", "%s",robot.eyes.getNumRings().toString());
            dashboardTelemetry.addData("Analysis", "%d", robot.eyes.pipeline.getAnalysis());

            dashboardTelemetry.update();

            telemetry.update();


            /**
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */

        }
        CatHW_Vision.UltimateGoalPipeline.numRings numRings = robot.eyes.getNumRings();

            /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */



            /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */

        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);

        robot.launcher.powerOn();
        robot.driveOdo.quickDrive(4,48,0.5,5,5.0);
        robot.launcher.openLauncher();
        robot.launcher.openLauncher();
        robot.launcher.openLauncher();
        robot.launcher.openLauncher();

        delayTimer.reset();
        while(delayTimer.seconds()<0.25){
            robot.launcher.openLauncher();
        }
       //robot.robotWait(0.25);
        robot.jaws.setTransferPower(.6);
        while(delayTimer.seconds()<6.0){
            robot.launcher.openLauncher();
        }
        //robot.robotWait(6.0);
        robot.jaws.setTransferPower(0);
        robot.launcher.closeLauncher();
        delayTimer.reset();
        robot.launcher.powerOff();

        numRings = CatHW_Vision.UltimateGoalPipeline.numRings.ONE;
        switch (numRings){
            case NONE:
                driveNone();
                break;
            case ONE:
                driveOne();
                break;
            case FOUR:
                driveFour();
                break;
        }


        robot.driveOdo.updatesThread.stop();
    }
    public void driveNone(){
        robot.driveOdo.quickDrive(-4,86,0.5,-90,3.0);
        robot.tail.setArmDown();
        //robot.tail.waitUntilDone();
        robot.robotWait(1);
        robot.tail.openGrabber();
        robot.robotWait(.5);
        robot.driveOdo.quickDrive(-28,37,0.5,0,5.0);
        robot.driveOdo.quickDrive(-27,27,0.5,0,5.0);
        robot.tail.closeGrabber();
        robot.robotWait(.5);
        robot.tail.setArmUp();
        robot.robotWait(1);
        robot.driveOdo.quickDrive(-4,76,0.5,-90,4.0);
        robot.tail.setArmDown();
        robot.robotWait(1);
        robot.tail.openGrabber();
        robot.driveOdo.quickDrive(-12,76,0.5,-90,3.0);
    }
    public void driveOne(){
        robot.driveOdo.quickDrive(-13,101,0.5,90,5.0);
        robot.tail.setArmDown();
        robot.robotWait(1);
        robot.tail.openGrabber();
        robot.robotWait(.5);
        robot.driveOdo.quickDrive(-9,101,0.5,90,3.0);
        robot.driveOdo.quickDrive(-38,60,0.5,0,5.0);
        robot.driveOdo.quickDrive(-30,27,0.5,-15,5.0);
        robot.tail.closeGrabber();
        robot.robotWait(.5);
        robot.tail.setArmUp();
        robot.robotWait(1);
        robot.driveOdo.quickDrive(-30,81,0.5,180,5.0);
        robot.tail.setArmDown();
        robot.robotWait(1);
        robot.tail.openGrabber();
        robot.driveOdo.quickDrive(-30,75,0.5,180,3.0);
    }
    public void driveFour(){


    }

}
