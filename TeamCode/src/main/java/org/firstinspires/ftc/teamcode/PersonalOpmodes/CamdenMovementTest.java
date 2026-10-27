package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name ="CamdenOpmode ", group = "LinearOpmode")
public class CamdenMovementTest extends LinearOpMode {

    double axial;
    double yaw;



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        //front left is a variable. The value on the rightside of the = sign is the real world component (thing) we are mapping the variable to.

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //This code says that we want an encoder to measure the velocity of our flywheel. Baisiclly, we are using an encoder.

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //Stating which direction the wheel is rotating in
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Wheel breaks when power is equal to 0
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //Stating which direction the wheel is rotating in


        waitForStart();
        while(opModeIsActive()) {
            double flywheelVel = flywheel.getVelocity();
            flywheel.setPower(0);
        }

    }
}

// Encoder returns information from flywheel (real world components)

/* 10/21/2026: Hardware mapping allows us to integrate software into real world components. Baisically, we can map our code to real world parts like motors.
Variables are always on the left side of the equal sign and tell us what we map it to. Encoders allow us to get data on real world components.
 */