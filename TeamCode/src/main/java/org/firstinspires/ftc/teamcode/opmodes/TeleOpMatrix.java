/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="Matrix Opmodes")
public class TeleOpMatrix extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode()
    {

        //Creates drive power variables
        double leftDrivePower = 0;
        double rightDrivePower = 0;
        //Power/speed of the gatherer motor
        double gatherPower=0;
        //Power/speed of the cannon motors
        double cannonPower=0;

        double leftStick1 = 0;

        double rightStick1 = 0;
        boolean shootFlag = false;

        boolean gatherOnFlag = false;
        double gatherToggleTime = 0;
        boolean gatherToggleFlag = false;

        double cannonToggleTime = 0;
        boolean cannonToggleFlag = false;
        boolean cannonOnFlag = false;

        DcMotor leftMotor;
        DcMotor rightMotor;
        DcMotor gatherMotor;
        DcMotor cannonMotor;
        DcMotor cannonMotor2;

        Servo loaderServo;

        //Maximum Power/speed of the gatherer motor
        final double maxGatherPower = 1.0;

        //Time that the loader stores to find when x amount of time has passed
        double loaderTime = 0;

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
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // gather is off, start gather in forward rotation
            if ( gamepad1.right_bumper && !gatherOnFlag && !gatherToggleFlag )
            {
                gatherPower = maxGatherPower;
                gatherOnFlag = true;
                gatherToggleTime = time;
                gatherToggleFlag = true;
            }
            // gather is on, stop the gather
            if ( gamepad1.right_bumper && gatherOnFlag && !gatherToggleFlag )
            {
                gatherPower = 0;
                gatherOnFlag = false;
                gatherToggleTime = time;
                gatherToggleFlag = true;
            }
            // gather is running, reverse gather rotation
            if ( gamepad1.left_bumper && gatherOnFlag && !gatherToggleFlag )
            {
                gatherPower *= -1;
                gatherToggleTime = time;
                gatherToggleFlag = true;
            }

            // cannon is off, start the cannon
            if ( gamepad2.left_bumper && !cannonOnFlag && !cannonToggleFlag )
            {
                cannonPower = 0.5;
                cannonOnFlag = true;
                cannonToggleTime = time;
                cannonToggleFlag = true;
            }
            // cannon is on, stop the cannon
            if ( gamepad2.left_bumper && cannonOnFlag && !cannonToggleFlag )
            {
                cannonPower = 0;
                cannonOnFlag = false;
                cannonToggleTime = time;
                cannonToggleFlag = true;
            }
            // shoot the cannon
            if ( gamepad2.right_bumper && !shootFlag && cannonOnFlag )
            {
                loaderServo.setPosition(0);
                shootFlag = true;
                loaderTime = time;
            }

            // reset the servo to the bottom of the cycle and clear the flag if button is released
            if ( time > loaderTime + 0.4 )
            {
                if ( !gamepad2.right_bumper )
                {
                    shootFlag = false;
                }
                loaderServo.setPosition(1);
            }

            // Reset the gather toggle lockout after time and both bumpers are released
            if (( time > gatherToggleTime + 1.0 ) && !gamepad1.left_bumper && !gamepad1.right_bumper )
            {
                gatherToggleFlag = false;
            }

            // Reset the cannon toggle lockout after time and bumper is released
            if (( time > cannonToggleTime + 1.0 ) && !gamepad2.left_bumper )
            {
                cannonToggleFlag = false;
            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
            //Creates and sets values left and right stick to the gamepad left stick, and right stick

            leftStick1=gamepad1.left_stick_y;

            rightStick1=gamepad1.right_stick_y;

            //Clips the left or right drive powers to 1 if it is > 1 and to -1 if it is < -1 (sets the values to between 1 and -1)
            leftDrivePower = Range.clip(rightStick1,-1,1);
            rightDrivePower = Range.clip(leftStick1,-1,1);

            //Sets the power of the drive motors to the value read from the gamepad sticks (between 0 and 1)
            leftMotor.setPower(leftDrivePower);
            rightMotor.setPower(rightDrivePower);

            //Activate the gatherer at power set
            gatherMotor.setPower(gatherPower);

            // Set cannon power
            cannonMotor.setPower(cannonPower);
            cannonMotor2.setPower(cannonPower);

            idle();
        }
    }
}
