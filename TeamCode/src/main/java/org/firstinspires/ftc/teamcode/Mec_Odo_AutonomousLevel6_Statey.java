/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

*/
/**
 * Mec_Odo_AutonomousLevel6_Statey.java
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
 *//*

@Autonomous(name="State Odo Autonomous", group="CatAuto")
public class Mec_Odo_AutonomousLevel6_Statey extends LinearOpMode
{
    */
/* Declare OpMode members. *//*

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isBuildZone = false;
    private boolean isParkAtWall = false;
    private boolean isFoundation = true;

    private CatHW_Vision.skyStonePos skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;

    @Override
    public void runOpMode() throws InterruptedException {

        */
/*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         *//*

        robot.init(hardwareMap, this, true);


        */
/*
        Init Delay Option Select:
         *//*

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }

            robot.eyes.findSkyStone();

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
            if (((gamepad1.y) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isBuildZone) {
                    isBuildZone = false;
                } else {
                    isBuildZone = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isParkAtWall) {
                    isParkAtWall = false;
                } else {
                    isParkAtWall = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.dpad_right) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isFoundation) {
                    isFoundation = false;
                } else {
                    isFoundation = true;
                }
                delayTimer.reset();
            }

            */
/**
             * LED code:
             *//*

            if (robot.isRedAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            } else {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            }
            */
/**
             * Telemetry while waiting for PLAY:
             *//*

            telemetry.addData("Delay Timer: ", timeDelay);

            skyStonePos = robot.eyes.giveSkyStonePos();
            if (skyStonePos == CatHW_Vision.skyStonePos.OUTSIDE && !isRedAlliance){
                skyStonePos = CatHW_Vision.skyStonePos.CENTER;
            }else if (skyStonePos == CatHW_Vision.skyStonePos.CENTER && !isRedAlliance){
                skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;
            }


            telemetry.addData("Label", skyStonePos);

            telemetry.addData("left position", robot.eyes.lastLeft);
            telemetry.addData("right position", robot.eyes.lastRight);
            telemetry.addData("center position", (robot.eyes.lastRight+robot.eyes.lastLeft)/2);
            telemetry.addData("confidence", robot.eyes.lastConfidence);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            telemetry.addData("isBuildZone Side? ", isBuildZone);
            telemetry.addData("isParkAtWall ", isParkAtWall);
            telemetry.addData("isFoundation ", isFoundation);
            telemetry.update();

            */
/**
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             *//*

        }
        */
/**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         *//*


        */
/**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         *//*

        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);

        */
/* Go! *//*

        if (isBuildZone) {
            driveBuildZone();
        } else {
            driveLoadingZone();
        }
        robot.driveOdo.updatesThread.stop();
    }
    public void driveLoadingZone() throws InterruptedException {

        if(isRedAlliance) {
            //move away from wall
            robot.tail.openGrabber();
            robot.driveOdo.quickDrive(0, 6, .9, 0,  1);
            //robot.jaws.intakeJawsRed();
            if (isFoundation) {
                //2 skystones, move foundation and park
                switch (skyStonePos) {
                    case INSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(3, 32, .9, -30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-1, 38, .9, -50, 2);
                        //go under skybridge backwards
                        robot.driveOdo.quickDrive(-1, 22, .9, -90, 1);
                        robot.driveOdo.quickDrive(70, 24, .9, -90, 3);
                        //TURN toward foundation and latch on
                        robot.driveOdo.quickDrive(84, 38, .9, -170, 2);
                        robot.claw.extendClaws();
                        robot.robotWait(1);
                        //move foundation and release stone
                        robot.jaws.outputJaws();
                        robot.robotWait(.2);
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.55);
                        //theta = -100
                        robot.driveOdo.quickDrive(72, 8, .9, -50, 3);
                        robot.driveOdo.quickDrive(78, 6, .9, -90, 1);
                        //release foundation
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.claw.retractClaws();
                        robot.robotWait(.75);
                        robot.driveOdo.quickDrive(84, 26, .9, -90, 2);
                        //back up
                        robot.driveOdo.quickDrive(-12, 26, .9, -90, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(-22, 50, .9, -40, 1.5);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(-16, 30, .9, 30, 2);
                        robot.driveOdo.quickDrive(42, 30, .9, 90, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(30, 40, .9, 90, 2);

                    break;
                    case CENTER:
                        //drive to stone
                        robot.driveOdo.quickDrive(11, 32, .9, -30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(6, 38, .9, -55, 2);
                            //go under skybridge backwards
                            robot.driveOdo.quickDrive(-1, 24, .9, -90, 1);
                            robot.driveOdo.quickDrive(66, 24, .9, -90, 3);
                            //TURN toward foundation and latch on
                            robot.driveOdo.quickDrive(84, 42, .9, -170, 2);
                            robot.claw.extendClaws();
                            robot.robotWait(1);
                            //move foundation and release stone
                            robot.jaws.outputJaws();
                            robot.robotWait(.75);
                            robot.driveOdo.updatesThread.powerUpdate.powerBoost(.55);
                        robot.driveOdo.quickDrive(72, 8, .9, -100, 3);
                        robot.driveOdo.quickDrive(78, 6, .9, -90, 1);
                            //release foundation
                            robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                            robot.claw.retractClaws();
                            robot.robotWait(.75);
                            robot.driveOdo.quickDrive(84, 29, .9, -90, 2);
                            //back up
                            robot.driveOdo.quickDrive(0, 28, .9, -90, 4);
                            //pick up second skystone
                            robot.jaws.intakeJaws();
                            robot.driveOdo.quickDrive(-15, 58, .9, -40, 1.5);
                            //back up and drive under bridge
                            robot.driveOdo.quickDrive(-8, 36, .9, 30, 2);
                            robot.driveOdo.quickDrive(42, 36, .9, 90, 4);
                            robot.jaws.outputJaws();
                            robot.driveOdo.quickDrive(30, 40, .9, 90, 2);

                        break;
                    case OUTSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(19, 32, .9, -30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(15, 42, .9, -50, 2);
                            //go under skybridge backwards
                            robot.driveOdo.quickDrive(-1, 22, .9, -90, 1);
                            robot.driveOdo.quickDrive(60, 24, .9, -90, 3);
                            //TURN toward foundation and latch on
                            robot.driveOdo.quickDrive(84, 40, .9, -180, 2);
                            robot.claw.extendClaws();
                            robot.robotWait(1);
                            //move foundation and release stone
                            robot.jaws.outputJaws();
                            robot.robotWait(.75);
                            robot.driveOdo.updatesThread.powerUpdate.powerBoost(.55);
                            robot.driveOdo.quickDrive(72, 8, .9, -100, 3);
                            robot.driveOdo.quickDrive(78, 6, .9, -90, 1);
                            //release foundation
                            robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                            robot.claw.retractClaws();
                            robot.robotWait(.75);
                            robot.driveOdo.quickDrive(84, 30, .9, -90, 2);
                            //back up
                            robot.driveOdo.quickDrive(1, 30, .9, -90, 4);
                            //pick up second skystone
                            robot.jaws.intakeJaws();
                            robot.driveOdo.quickDrive(-8, 55, .9, -40, 1.5);
                            //back up and drive under bridge
                            robot.driveOdo.quickDrive(0, 32, .9, 30, 2);
                            robot.driveOdo.quickDrive(42, 32, .9, 90, 4);
                            robot.jaws.outputJaws();
                            robot.driveOdo.quickDrive(30, 30, .9, 90, 2);

                        break;
                }
            }
            else {
                //2 skystones and park (is redAlliance)
                switch (skyStonePos) {
                    case INSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(3, 32, .9, -30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-1, 38, .9, -50, 2);
                        //back up and TURN part way
                        robot.driveOdo.quickDrive(0, 30, .9, 30, 2);
                        //if the block is fully in the intake TURN off the jaws to save power
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        //drive under bridge
                        robot.driveOdo.quickDrive(38, 30, .9, 90, 3.5);
                        //output stone
                        robot.jaws.outputJaws();
                        //back up
                        robot.driveOdo.quickDrive(-10, 38, .9, -90, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        //robot.driveOdo.quickDrive(-11, 58, .9, -90, 1.5);
                        robot.driveOdo.quickDrive(-18, 58, .9, -55, 1);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(-16, 38, .9, 30, 2);
                        robot.driveOdo.quickDrive(38, 38, .9, 90, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(30, 38, .9, 90, 2);
                        robot.robotWait(2);

                        break;
                    case CENTER:
                        //drive to stone
                        robot.driveOdo.quickDrive(11, 32, .9, -30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(7, 40, .9, -55, 2);
                        //back up and TURN part way
                        robot.driveOdo.quickDrive(8, 30, .9, 30, 2);
                        //if the block is fully in the intake TURN off the jaws to save power
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        //drive under bridge
                        robot.driveOdo.quickDrive(38, 30, .9, 90, 3.5);
                        //output stone
                        robot.jaws.outputJaws();
                        //back up
                        robot.driveOdo.quickDrive(-2, 38, .9, -90, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(-11, 58, .9, -55, 1);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(-16, 34, .9, 30, 2);
                        robot.driveOdo.quickDrive(38, 34, .9, 90, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(30, 36, .9, 90, 2);
                        robot.robotWait(2);

                        break;
                    case OUTSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(19, 32, .9, -30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(15, 42, .9, -50, 2);
                        //back up and TURN part way
                        robot.driveOdo.quickDrive(0, 28, .9, 30, 2);
                        //if the block is fully in the intake TURN off the jaws to save power
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        //drive under bridge
                        robot.driveOdo.quickDrive(38, 28, .9, 90, 3.5);
                        //output stone
                        robot.jaws.outputJaws();
                        //back up
                        robot.driveOdo.quickDrive(5, 34, .9, -90, 4);
                        //pick up second skystone
                        robot.driveOdo.quickDrive(1, 40, .9, -90, 1);
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(-8, 60, .9, -60, 1.5);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(0, 34, .9, 30, 2);
                        robot.driveOdo.quickDrive(38, 38, .9, 90, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(30, 40, .9, 90, 2);
                        robot.robotWait(2);

                }
            }

            }
        else {
            //if not red alliance
            //move away from wall
            robot.tail.openGrabber();
            robot.driveOdo.quickDrive(0, 6, .9, 0,  1);
            robot.jaws.intakeJawsRed();
            if (isFoundation) {
                //2 skystones, move foundation and park
                switch (skyStonePos) {
                    case INSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(-2, 34, .9, 40, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(6, 36, .9, 57, 2);
                        //go under skybridge backwards
                        robot.driveOdo.quickDrive(1, 22, .9, 83, 1);
                        //TURN off intake if it has a stone
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        robot.driveOdo.quickDrive(-70, 24, .9, 90, 2);
                        //TURN toward foundation and latch on
                        robot.driveOdo.quickDrive(-80, 49, .9, 175, 2);
                        robot.claw.extendClaws();
                        robot.robotWait(.5);
                        //move foundation and release stone
                        robot.jaws.outputJaws();
                        //robot.robotWait(.15);
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.55);
                        robot.driveOdo.quickDrive(-68, 8, .9, 100, 3);
                        robot.driveOdo.quickDrive(-78, 8, .9, 90, 1);
                        //release foundation
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.claw.retractClaws();
                        robot.robotWait(.2);
                        robot.driveOdo.quickDrive(-84, 34, .9, 90, 2);
                        //back up
                        robot.driveOdo.quickDrive(17, 35, .9, 77, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(28, 46, .9, 55, 1.5);
                        //back up and drive under bridge
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.4);
                        robot.driveOdo.quickDrive(16, 25, .9, -30, 2);
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.driveOdo.quickDrive(-42, 42, .9, -83, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(-28, 42, .9, -83, 2);

                        break;
                    case CENTER:
                        //drive to stone
                        robot.driveOdo.quickDrive(-11, 32, .9, 30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-6, 38, .9, 55, 2);
                        //go under skybridge backwards
                        robot.driveOdo.quickDrive(1, 22, .9, 83, 1);
                        //TURN off intake if it has a stone
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        robot.driveOdo.quickDrive(-70, 24, .9, 90, 2);
                        //TURN toward foundation and latch on
                        robot.driveOdo.quickDrive(-80, 49, .9, 175, 2);
                        robot.claw.extendClaws();
                        robot.robotWait(.5);
                        //move foundation and release stone
                        robot.jaws.outputJaws();
                        //robot.robotWait(.15);
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.55);
                        robot.driveOdo.quickDrive(-68, 8, .9, 100, 3);
                        robot.driveOdo.quickDrive(-78, 8, .9, 90, 1);
                        //release foundation
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.claw.retractClaws();
                        robot.robotWait(.2);
                        robot.driveOdo.quickDrive(-84, 34, .9, 90, 2);
                        //back up
                        robot.driveOdo.quickDrive(7, 33, .9, 80, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(20, 46, .9, 55, 1.5);
                        //back up and drive under bridge
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.4);
                        robot.driveOdo.quickDrive(16, 22, .9, -30, 2);
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.driveOdo.quickDrive(-42, 37, .9, -83, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(-28, 40, .9, -83, 2);

                        break;
                    case OUTSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(-20, 36, .9, 48, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-14, 38, .9, 60, 2);
                        //go under skybridge backwards
                        robot.driveOdo.quickDrive(-7, 22, .9, 83, 1);
                        //TURN off intake if it has a stone
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        robot.driveOdo.quickDrive(-70, 24, .9, 90, 2);
                        //TURN toward foundation and latch on
                        robot.driveOdo.quickDrive(-78, 53, .9, 180, 2);
                        robot.claw.extendClaws();
                        robot.robotWait(.5);
                        //move foundation and release stone
                        robot.jaws.outputJaws();
                        //robot.robotWait(.15);
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.55);
                        robot.driveOdo.quickDrive(-68, 8, .9, 100, 3);
                        robot.driveOdo.quickDrive(-78, 8, .9, 90, 1);
                        //release foundation
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.claw.retractClaws();
                        robot.robotWait(.2);
                        robot.driveOdo.quickDrive(-84, 34, .9, 90, 2);
                        //back up
                        robot.driveOdo.quickDrive(-1, 33, .9, 80, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(12, 46, .9, 55, 1.5);
                        //back up and drive under bridge
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.4);
                        robot.driveOdo.quickDrive(8, 22, .9, -30, 2);
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.driveOdo.quickDrive(-42, 37, .9, -83, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(-28, 40, .9, -83, 2);

                        */
/*
                        //drive to stone
                        robot.driveOdo.quickDrive(-19, 32, .9, 30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-15, 42, .9, 50, 2);
                        //go under skybridge backwards
                        robot.driveOdo.quickDrive(1, 22, .9, 83, 1);
                        //TURN off intake if it has a stone
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        robot.driveOdo.quickDrive(-70, 24, .9, 90, 2);
                        //TURN toward foundation and latch on
                        robot.driveOdo.quickDrive(-80, 49, .9, 175, 2);
                        robot.claw.extendClaws();
                        robot.robotWait(.5);
                        //move foundation and release stone
                        robot.jaws.outputJaws();
                        //robot.robotWait(.15);
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.4);
                        robot.driveOdo.quickDrive(-68, 8, .9, 100, 3);
                        robot.driveOdo.quickDrive(-78, 8, .9, 90, 1);
                        //release foundation
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.claw.retractClaws();
                        robot.robotWait(.2);
                        robot.driveOdo.quickDrive(-84, 34, .9, 90, 2);
                        //back up
                        robot.driveOdo.quickDrive(4, 35, .9, 90, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(12, 46, .9, 55, 1.5);
                        //back up and drive under bridge
                        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.4);
                        robot.driveOdo.quickDrive(16, 25, .9, -30, 2);
                        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
                        robot.driveOdo.quickDrive(-42, 42, .9, -83, 4);
                        robot.jaws.outputJaws();
                        robot.driveOdo.quickDrive(-28, 42, .9, -83, 2);
*//*

                        break;
                }
            }
            else {

                //is blue and no foundation
                //2 skystones and park
                switch (skyStonePos) {
                    case INSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(-2, 34, .9, 40, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(6, 36, .9, 57, 2);
                        //back up and TURN part way
                        robot.driveOdo.quickDrive(3, 27, .9, -35, 2);
                        //if the block is fully in the intake TURN off the jaws to save power
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        //drive under bridge
                        robot.driveOdo.quickDrive(-42, 32, .9, -75, 2.5);
                        //output stone
                        robot.jaws.outputJaws();
                        robot.robotWait(.5);
                        //back up
                        robot.driveOdo.quickDrive(14, 28, .9, 67, 4);
                        //pick up second skystone
                        //robot.driveOdo.quickDrive(-11, 58, .9, -90, 1.5);
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(29, 38, .9, 50, 2);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(16, 32, .9, -37, 3);
                        robot.driveOdo.quickDrive(-40, 39, .9, -77, 4);
                        robot.jaws.outputJaws();
                        robot.robotWait(.5);
                        robot.driveOdo.quickDrive(-27, 47, .9, -77, 2);

                        break;
                    case CENTER:
                        //drive to stone
                        robot.driveOdo.quickDrive(-11, 30, .9, 30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-7, 40, .9, 55, 2);
                        //back up and TURN part way
                        robot.driveOdo.quickDrive(-8, 27, .9, -30, 2);
                        //if the block is fully in the intake TURN off the jaws to save power
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        //drive under bridge
                        robot.driveOdo.quickDrive(-40, 34, .9, -80, 3.5);
                        //output stone
                        robot.jaws.outputJaws();
                        robot.robotWait(.5);
                        //back up
                        robot.driveOdo.quickDrive(6, 34, .9, 90, 4);
                        //pick up second skystone
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(17, 48, .9, 55, 2);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(16, 29, .9, -30, 2);
                        robot.driveOdo.quickDrive(-40, 41, .9, -80, 4);
                        robot.jaws.outputJaws();
                        robot.robotWait(.5);
                        robot.driveOdo.quickDrive(-30, 42, .9, -80, 2);

                        break;
                    case OUTSIDE:
                        //drive to stone
                        robot.driveOdo.quickDrive(-17, 32, .9, 30, 2);
                        //pick up stone
                        robot.driveOdo.quickDrive(-12, 42, .9, 50, 2);
                        //back up and TURN part way
                        robot.driveOdo.quickDrive(0, 27, .9, -30, 2);
                        //if the block is fully in the intake TURN off the jaws to save power
                        if (robot.jaws.hasStone()) {
                            robot.jaws.turnOffJaws();
                        }
                        //drive under bridge
                        robot.driveOdo.quickDrive(-38, 33, .9, -80, 3.5);
                        //output stone
                        robot.jaws.outputJaws();
                        robot.robotWait(.5);
                        //back up
                        robot.driveOdo.quickDrive(-5, 31, .9, 70, 4);
                        //pick up second skystone
                        robot.driveOdo.quickDrive(6, 32, .9, 70, 2);
                        robot.jaws.intakeJaws();
                        robot.driveOdo.quickDrive(14, 43, .9, 60, 2);
                        //back up and drive under bridge
                        robot.driveOdo.quickDrive(0, 28, .9, -30, 2);
                        robot.driveOdo.quickDrive(-38, 42, .9, -77, 4);
                        robot.jaws.outputJaws();
                        robot.robotWait(.5);
                        robot.driveOdo.quickDrive(-30, 42, .9, -77, 2);

                }
            }

        }
        }



    public void driveBuildZone() throws InterruptedException {
        //drive to the foundation and latch on
        robot.driveOdo.quickDrive(isRedAlliance? -24 : 20, -34, .6, 0, 2);
        robot.claw.extendClaws();
        robot.robotWait(.7);
        //pull foundation back and TURN
        robot.driveOdo.quickDrive(isRedAlliance? -2 : 2, isRedAlliance? 0 : -4, .9, isRedAlliance? 80 : -75, 2);
        //release foundation
        robot.claw.retractClaws();
        //push foundation into building site
        robot.driveOdo.quickDrive(isRedAlliance? -20 : 17, isRedAlliance? 0 : -4, .9, isRedAlliance? 100 : -95, 2);
        //drive towards wall to avoid hitting alliance partners
        //robot.driveOdo.quickDrive(isRedAlliance? -14 : 14, isRedAlliance? -6 : 6, .9, isRedAlliance? 90 : -90, 2);
        //park next to wall
        robot.driveOdo.quickDrive(isRedAlliance? 30 : -26, isRedAlliance? -1 : 5, .9, isRedAlliance? 95 : -90, 3);



        */
/*
        // drive to the isFoundation slowly
        robot.driveOdo.quickDrive( isRedAlliance ? -20 : 17,-35,.45,isRedAlliance ? 8 : -8,.2,4);
        //lower the isFoundation claws
        robot.claw.extendClaws();
        robot.robotWait(.25);
        //override the min power so we have enough power to move the isFoundation while driving
        robot.driveOdo.updatesThread.powerUpdate.powerBoost(.7);
        //drive straight forward a little to simplify the TURN
        robot.driveOdo.quickDrive(isRedAlliance ? -13 : 13,-19,.9,0,.2,3);
        //rotate the isFoundation while moving forward
        robot.driveOdo.quickDrive(isRedAlliance ? -5 : 5,-6,.9,isRedAlliance ? 90 : -90,.7,4);
        //push the isFoundation against the wall
        robot.driveOdo.quickDrive(isRedAlliance ? -13 : 13,-6,.9,isRedAlliance ? 90 : -90,.67,3);
        //reset min power to normal
        robot.driveOdo.updatesThread.powerUpdate.resetPowerToNormal();
        //lift up the claw
        robot.claw.retractClaws();
        robot.robotWait(.25);
        if(isParkAtWall){
            robot.driveOdo.quickDrive(isRedAlliance ? -5 : 5, -3, .8, isRedAlliance ? 90 : -90, .2, 2);
            robot.driveOdo.quickDrive(isRedAlliance ? 25 : -26, isRedAlliance ? -3 : 0, .8, isRedAlliance ? 90 : -90, .2, 2);
        }
        else {
            robot.driveOdo.quickDrive(isRedAlliance ? -5 : 5, -30, .8, isRedAlliance ? 90 : -90, .2, 2);
            robot.driveOdo.quickDrive(isRedAlliance ? 22 : -26, isRedAlliance ? -33 : -28, .8, isRedAlliance ? 90 : -90, .2, 2);
        }
*//*

    }
}*/
