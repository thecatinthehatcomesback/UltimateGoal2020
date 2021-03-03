package org.firstinspires.ftc.teamcode;/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

*/

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TestVision.java
 *
 *
 *This is a test to see if we can use the field targets to orient ourselves on the field
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@TeleOp(name="TestVision", group="CatAuto")

public class TestVision extends LinearOpMode
{

/* Declare OpMode members. */

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isPowerShot = false;



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
                robot.driveOdo.updatesThread.stop();
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
            if (gamepad1.b && (delayTimer.seconds() > 0.8)) {
                isPowerShot = !isPowerShot;
                delayTimer.reset();
            }

            //Allow the intake to run in autonomous
            robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.3));
            if (robot.jaws.getJawPower() > 0.05) {
                robot.jaws.setTransferPower(0.6);
            } else {
                robot.jaws.setTransferPower(0);
            }


            /*
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


            /*
             * Telemetry while waiting for PLAY:
             */

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
            if (isPowerShot) {
                telemetry.addData("Goal: ", "Power Shot");
            } else {
                telemetry.addData("Goal: ", "High Goal");
            }
            telemetry.addData("Num of Rings", "%s", robot.eyes.getNumRings().toString());
            dashboardTelemetry.addData("Num of Rings", "%s", robot.eyes.getNumRings().toString());
            dashboardTelemetry.addData("Analysis", "%d", robot.eyes.pipeline.getAnalysis());

            dashboardTelemetry.update();

            telemetry.update();


            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */

        }
        CatHW_Vision.UltimateGoalPipeline.numRings numRings = robot.eyes.getNumRings();

        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);
        //powers on launcher
        robot.launcher.powerOn();
        if (isPowerShot) {
            robot.launcher.presetPowerShot();
            robot.launcher.aimR();

        } else {
            robot.launcher.aimHigh();
        }
        robot.robotWait(1);
        //drives to position to shoot rings
        robot.driveOdo.quickDrive(5, 60, 0.75, 5, 5.0);

        //shoots rings
        if (isPowerShot) {
            robot.launcher.shootPowerShots();
        } else {
            robot.launcher.shootHighGoal();
        }
        robot.launcher.aimHigh();



        robot.driveOdo.updatesThread.stop();
        robot.eyes.stop();
    }
}
