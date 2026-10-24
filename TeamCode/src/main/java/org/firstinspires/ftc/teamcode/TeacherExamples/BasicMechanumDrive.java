package org.firstinspires.ftc.teamcode.TeacherExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//This code will be what we use for our robots with Mechanum Drive wheels. The robot will fail if the wheel are not correct.
@Disabled
@TeleOp(name = "BasicMechanumDrive", group = "LinearOpMode")
public class BasicMechanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor frontright = hardwareMap.get(DcMotor.class, "backright");
        DcMotor backleft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        double axial;
        double lateral;
        double yaw;

        waitForStart();
        while (opModeIsActive()) {

            axial = gamepad1.left_stick_y;
            yaw = gamepad1.right_stick_x;
            lateral = gamepad1.left_stick_x;

            double frontRightPower = axial + yaw + lateral;
            double frontLeftPower = axial - yaw - lateral;
            double backRightPower = axial + yaw - lateral;
            double backLeftPower = axial - yaw + lateral;

            double max;
            //This makes sure the motors don't exceed 1.0/-1.0
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontright.setPower(frontRightPower);
            frontleft.setPower(frontLeftPower);
            backright.setPower(backRightPower);
            backleft.setPower(backLeftPower);

            telemetry.addData("Front Wheel Power Right/Left", "%4.2f, %4.2f", frontRightPower, frontLeftPower);
            telemetry.addData("Back Wheel Power Right/Left", "%4.2f, %4.2f", backRightPower, backLeftPower);
            telemetry.update();
    }
    }
}



