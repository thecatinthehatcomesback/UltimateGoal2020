package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Test_TeleOp.java
 *
 *
 * A Linear opMode class to be our TeleOp testing method to try and solve our problems throughout
 * the year without having to modify the main TeleOp.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@TeleOp(name="Test Servo", group="CatTest TeleOp")
public class TestServoPos extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime upButtonTimer = new ElapsedTime();
    static final double ticksPerDegree = 8192*(64/18)/360*1.2;


    /* Constructor */
    public TestServoPos() {

    }


    @Override
    public void runOpMode() throws InterruptedException {
        Servo fireServo       = hardwareMap.get(Servo.class,"fire_servo");
        CRServo turretAim       = hardwareMap.get(CRServo.class,"aimer");
        Servo transferServo = hardwareMap.servo.get("transfer");
        DcMotor turretEncoder   = hardwareMap.get(DcMotor.class,"right_front_motor");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretAim.setDirection(DcMotorSimple.Direction.FORWARD);



        // Finished!  Now tell the driver...
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for PLAY:
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        double firePos = 0.5;
        double transferPos = 0.5;
        ElapsedTime buttonTimer = new ElapsedTime();
        while (opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            // Driver 1 Controls:
            //--------------------------------------------------------------------------------------
            if(gamepad1.dpad_up && (buttonTimer.milliseconds()>200)){
                firePos += 0.03;
                buttonTimer.reset();

            }

            if(gamepad1.dpad_down && (buttonTimer.milliseconds()>200)){
                firePos += -0.03;
                buttonTimer.reset();

            }
            if(gamepad1.dpad_left && (buttonTimer.milliseconds()>200)){
                transferPos += -0.03;
                buttonTimer.reset();

            }
            if(gamepad1.dpad_right && (buttonTimer.milliseconds()>200)){
                transferPos += 0.03;
                buttonTimer.reset();

            }
            turretAim.setPower(gamepad1.right_stick_x);
            transferServo.setPosition(transferPos);
            fireServo.setPosition(firePos);

            telemetry.addData("Transfer Pos", "%.2f",transferPos);
            telemetry.addData("Fire Pos", "%.2f", firePos);
            telemetry.addData("Turret Pow", "%.2f", turretAim.getPower());
            telemetry.addData("Turret Encoder Pos", "%d", -turretEncoder.getCurrentPosition());
            telemetry.addData("Turret Angle", "%.2f", -turretEncoder.getCurrentPosition()/ticksPerDegree);


            telemetry.update();
        }

    }
}
