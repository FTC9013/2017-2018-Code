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

@Autonomous(name="AutonomousRed", group="Matrix")
public class AutonomousRed extends LinearOpMode{

    //--------------------------------------------------------------------------
    //
    // AutonomousRed
    //
    public AutonomousRed ()

    {
    } // AutonomousRed

    static final double COUNTS_PER_INCH = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftMotor;
        DcMotor rightMotor;
        //DcMotor cannonMotor;

        //Initialize
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        //cannonMotor = hardwareMap.dcMotor.get("cannon");

        //Set turn direction for the motors
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        //To make sure that motors are set to the same speed regardless of direction
        //leftMotor.setMaxSpeed(rightMotor.getMaxSpeed());
        //rightMotor.setMaxSpeed(leftMotor.getMaxSpeed());

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



    }
}
