package org.firstinspires.ftc.teamcode.opmodes;

/*
 * Created by MI Matrix on 11/10/2016.
 */

//------------------------------------------------------------------------------
//
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ExperimentalAuto", group="Matrix")
public class ExperimentalAuto extends LinearOpMode{

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Creates drive power variables
        double leftDrivePower = 0;
        double rightDrivePower = 0;
        //Power/speed of the gatherer motor
        double gatherPower=0;
        //Power/speed of the cannon motors
        double cannonPower=0;

        DcMotor leftMotor;
        DcMotor rightMotor;
        DcMotor gatherMotor;
        DcMotor cannonMotor;
        DcMotor cannonMotor2;

        Servo loaderServo;

        CRServo pusherServo;

        //Maximum Power/speed of the gatherer motor
        final double maxGatherPower = 1.0;

         /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        gatherMotor = hardwareMap.dcMotor.get("gather");
        cannonMotor = hardwareMap.dcMotor.get("cannon1");
        cannonMotor2 = hardwareMap.dcMotor.get("cannon2");

        loaderServo = hardwareMap.servo.get("loader");

        pusherServo = hardwareMap.crservo.get("pusher");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        gatherMotor.setDirection(DcMotor.Direction.FORWARD);
        cannonMotor.setDirection(DcMotor.Direction.FORWARD);
        cannonMotor2.setDirection(DcMotor.Direction.REVERSE);

        //Sets the power of the drive motors to the initial value
        leftMotor.setPower(leftDrivePower);
        rightMotor.setPower(rightDrivePower);

        //Init the gatherer at power 0
        gatherMotor.setPower(gatherPower);

        // Set cannon power
        cannonMotor.setPower(cannonPower);
        cannonMotor2.setPower(cannonPower);

        // cycle shoot arm down
        loaderServo.setPosition(1);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        runtime.reset();
        pusherServo.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 3.0))
        {
            telemetry.addData("Status", "push 0");    //
            telemetry.update();
        }

        runtime.reset();
        pusherServo.setPower(1);

        while (opModeIsActive() && (runtime.seconds() < 3.0))
        {
            telemetry.addData("Status", "push 1");    //
            telemetry.update();
        }

        runtime.reset();
        pusherServo.setPower(-1);

        while (opModeIsActive() && (runtime.seconds() < 3.0))
        {
            telemetry.addData("Status", "push -1");    //
            telemetry.update();
        }
        runtime.reset();
        pusherServo.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 3.0))
        {
            telemetry.addData("Status", "push 0");    //
            telemetry.update();
        }










        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 10.0))
        {
            telemetry.addData("Path", "Wait: %10.0f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds at quarter power with gatherer running
        // turn on the gather
        gatherMotor.setPower(maxGatherPower);

        //Sets the power of the drive motors to the run value
        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0))
        {
            telemetry.addData("Path", "Step 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // turn off the gather
        gatherMotor.setPower(0.0);

        //Stop the drive motors
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        // Step 2:


        // turn on the shooter (half power)
        cannonMotor.setPower(0.5);
        cannonMotor2.setPower(0.5);
        // let spin up
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0))
        {
            telemetry.addData("Path", "Reload: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // shoot twice 2 seconds between shots
        // shoot the cannon
        loaderServo.setPosition(0);
        // reset the run timer
        runtime.reset();
        // hold shoot arm up
        while (opModeIsActive() && (runtime.seconds() < 0.40))
        {
            telemetry.addData("Path", "Shoot: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // cycle shoot arm down
        loaderServo.setPosition(1);
        // reset the run timer
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0))
        {
            telemetry.addData("Path", "Reload: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // shoot the cannon
        loaderServo.setPosition(0);
        // reset the run timer
        runtime.reset();
        // hold shoot arm up
        while (opModeIsActive() && (runtime.seconds() < 0.40))
        {
            telemetry.addData("Path", "Shoot: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // cycle shoot arm down
        loaderServo.setPosition(1);

        // turn off the shooter
        cannonMotor.setPower(0.0);
        cannonMotor2.setPower(0.0);

        // Step 3:  short drive forward

        //Sets the power of the drive motors to the run value
        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5))
        {
            telemetry.addData("Path", "Step 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Stop the drive motors
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}