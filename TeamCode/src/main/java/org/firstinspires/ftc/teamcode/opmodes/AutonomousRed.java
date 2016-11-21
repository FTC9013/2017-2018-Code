package org.firstinspires.ftc.teamcode.opmodes;

/*
 * Created by MI Matrix on 11/10/2016.
 */

//------------------------------------------------------------------------------
//
// AutonomousRed
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousRed", group="Matrix")
public class AutonomousRed extends LinearOpMode{

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

//        // Step 1:  Drive forward for 5 seconds
        //Sets the power of the drive motors to the initial value
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0))
        {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        leftMotor.setPower(0);
        rightMotor.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
