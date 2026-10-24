package org.firstinspires.ftc.teamcode.TeacherExamples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//This what the code will look like when put together. If you want to create your own code, create a new java class by right-clicking
//the "TeacherExamplesLinearOpmode" or "TeacherExamplesOpmode" and pressing new
@TeleOp(name = "Basic Motor Control: with IMU", group = "LinearOpMode")
@Disabled
public class BasicMechanumDriveIMU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FRONT_L = hardwareMap.get(DcMotor.class, "FRONT_L");
        DcMotor FRONT_R = hardwareMap.get(DcMotor.class, "FRONT_R");
        DcMotor BACK_L = hardwareMap.get(DcMotor.class, "BACK_L");
        DcMotor BACK_R = hardwareMap.get(DcMotor.class, "BACK_R");

        FRONT_L.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_R.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_L.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_R.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        double heading = 0;

        double forward_p;
        double right_p;
        double spin_p;

        waitForStart();
            imu.resetYaw();
        while (opModeIsActive()) {

            forward_p = gamepad1.left_stick_y;
            spin_p = gamepad1.right_stick_x;
            right_p = gamepad1.left_stick_x;

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double frontRightPower = forward_p + spin_p + right_p;
            double frontLeftPower = forward_p - spin_p - right_p;
            double backRightPower = forward_p + spin_p - right_p;
            double backLeftPower = forward_p - spin_p + right_p;

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

            FRONT_R.setPower(frontRightPower);
            FRONT_L.setPower(frontLeftPower);
            BACK_R.setPower(backRightPower);
            BACK_L.setPower(backLeftPower);

            telemetry.addData("Front Wheel Power Right/Left", "%4.2f, %4.2f", frontRightPower, frontLeftPower);
            telemetry.addData("Back Wheel Power Right/Left", "%4.2f, %4.2f", backRightPower, backLeftPower);
            telemetry.addData("Current Rotation (Degrees)", "%.2f", heading);
            telemetry.update();
    }
    }
}



