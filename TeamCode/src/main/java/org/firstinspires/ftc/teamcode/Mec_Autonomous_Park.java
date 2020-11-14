package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Mec_Autonomous_Park.java
 *
 *
 * A Linear OpMode class to be an extremely simple autonomous method for both Blue & Red alliance
 * sides to allow our alliance partner to perform all their autonomous objectives.  We will pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 *
 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */
@Autonomous(name="Park Autonomous", group="CatAuto")
public class Mec_Autonomous_Park extends LinearOpMode
{
    /* Declare OpMode members. */
    CatHW_Async robot = new CatHW_Async();    // All the hardware classes init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isParkAtWall = true;

    private CatHW_Vision.skyStonePos skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;

    @Override
    public void runOpMode()  throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        robot.init(hardwareMap, this, true);


        /*
        Send telemetry message to signify robot getting ready:
         */
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset: //
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :%7d  :%7d",
                robot.driveClassic.leftFrontMotor.getCurrentPosition(),
                robot.driveClassic.rightFrontMotor.getCurrentPosition(),
                robot.driveClassic.leftRearMotor.getCurrentPosition(),
                robot.driveClassic.rightRearMotor.getCurrentPosition());
        telemetry.update();


        /*
        Init Delay Option Select:
         */
        // After init is pushed but before Start we can change the delay using dpad up/down:
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start:
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed.
                return;
            }

            // Look for SkyStone positions:
            robot.eyes.findSkyStone();

            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait.
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait.
                if (timeDelay > 0) {
                    // No such thing as negative time.
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides.
                if (isRedAlliance) {
                    isRedAlliance = false;
                    CatHW_Async.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    CatHW_Async.isRedAlliance = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides.
                isParkAtWall = !isParkAtWall;
                delayTimer.reset();
            }

            /*
            LED code:
             */
            if (CatHW_Async.isRedAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            } else {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            }

            /*
            Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);


            skyStonePos = robot.eyes.giveSkyStonePos();
            if (skyStonePos == CatHW_Vision.skyStonePos.OUTSIDE && !isRedAlliance){
                skyStonePos = CatHW_Vision.skyStonePos.CENTER;
            } else if (skyStonePos == CatHW_Vision.skyStonePos.CENTER && !isRedAlliance){
                skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;
            }


            telemetry.addData("Label", skyStonePos);

            telemetry.addData("\n\tLeft position", robot.eyes.lastLeft);
            telemetry.addData("\tRight position", robot.eyes.lastRight);
            telemetry.addData("\tCenter position",
                    (robot.eyes.lastRight + robot.eyes.lastLeft) / 2);
            telemetry.addData("Confidence", robot.eyes.lastConfidence);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            telemetry.addData("isParkAtWall ", isParkAtWall);
            telemetry.update();

            /*
            We don't need a "waitForStart()" since we've been running our own loop all this time so
            that we can make some changes.
             */
        }



        /*
         * Runs after hit Start:
         */

        // Init the IMU after play so that it is not offset after remaining idle for a few minutes.
        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);

        /* Go! */
        if (isParkAtWall) {
            robot.driveOdo.leftFrontMotor.setPower(.4);
            robot.driveOdo.leftRearMotor.setPower(.4);
            robot.driveOdo.rightFrontMotor.setPower(.4);
            robot.driveOdo.rightRearMotor.setPower(.4);
            robot.robotWait(.75);
            robot.driveOdo.leftFrontMotor.setPower(0);
            robot.driveOdo.leftRearMotor.setPower(0);
            robot.driveOdo.rightFrontMotor.setPower(0);
            robot.driveOdo.rightRearMotor.setPower(0);
        } else {
            if (isRedAlliance) {
                robot.driveOdo.quickDrive(0, 25, .6, 0, 3);
                robot.driveOdo.quickDrive(-22, 25, .6, 0, 2);
            }else {
                robot.driveOdo.quickDrive(0, 25, .6, 0, 3);
                robot.driveOdo.quickDrive(22, 25, .6, 0, 2);
            }
        }
        robot.driveOdo.updatesThread.stop();

    }
}