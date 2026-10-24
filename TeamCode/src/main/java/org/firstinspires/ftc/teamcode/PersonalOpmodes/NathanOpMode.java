package org.firstinspires.ftc.teamcode.PersonalOpmodes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

//This is my personal code. If you want to use this code, please rename it.
@Disabled
@TeleOp(name = "Nathan's OpMode", group = "LinearOpMode")
public class NathanOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FRONT_L = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor FRONT_R = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor BACK_L = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor BACK_R = hardwareMap.get(DcMotor.class, "backright");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CRServo right_launch_servo = hardwareMap.get(CRServo.class, "rightServo");
        CRServo left_launch_servo = hardwareMap.get(CRServo.class, "leftServo");


        FRONT_L.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_R.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_L.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_R.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        double heading = 0;
        double x;
        double y;

        boolean cycle = false;
        boolean lastA = false;
        boolean lastB = false;
        boolean yawReset = false;
        boolean fieldToggle = false;
        double launcher = 0.0;

        double forward_p;
        double right_p;
        double spin_p;

        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {
            telemetry.addLine("dpad up toggles field centric drive");
            telemetry.addLine("dpad down resets yaw value");
            telemetry.addLine("dpad right toggles rotation lock");
            telemetry.addLine("dpad left resets strafe distance");

            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            spin_p = gamepad1.right_stick_x;

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double flywheelVelocity = flywheel.getVelocity();

            if(gamepad1.dpad_up && !lastA) {
                fieldToggle = !fieldToggle;
            }
            lastA = gamepad1.dpad_up;

            if(fieldToggle) {
                forward_p = y*cos(-heading) - x * sin(-heading);
                right_p = y *sin(-heading) + x*cos(-heading);
            }
            else {
                forward_p = y;
                right_p = x;
            }
            telemetry.addData("Field Centric Toggled:", fieldToggle);
            telemetry.update();

            if(gamepad1.dpad_down && !lastB) {
                yawReset = !yawReset;
            }
            lastB = gamepad1.dpad_down;

            if(yawReset) {
                imu.resetYaw();
            }

            if(gamepad1.a) {
                if(flywheelVelocity <= 400) {
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                    launcher = 1.0;
                    if(cycle) {
                        cycle = !cycle;
                    }
                }
                if(flywheelVelocity >= 400 && flywheelVelocity < 500) {
                    if(cycle) {
                        launcher = 0.0;
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                    }
                    else {
                        launcher = 0.3;
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                }
                if(flywheelVelocity > 500) {
                    launcher = -1.0;
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                    cycle = !cycle;
                    sleep(500);
                }
            }
            else {
                launcher = 0.0;
                right_launch_servo.setPower(0);
                left_launch_servo.setPower(0);
            }


            double frontRightPower = forward_p + spin_p + right_p;
            double frontLeftPower = forward_p - spin_p - right_p;
            double backRightPower = forward_p + spin_p - right_p;
            double backLeftPower = forward_p - spin_p + right_p;

            if (frontRightPower <= 0 && frontRightPower >= 0 && frontLeftPower <= 0 && frontLeftPower >= 0 && backLeftPower <= 0 && backLeftPower >= 0 && backRightPower <= 0 && backRightPower >= 0) {
                FRONT_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FRONT_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BACK_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BACK_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            double max;
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
            flywheel.setPower(launcher);

            telemetry.addData("Front Wheel Power Right/Left", "%4.2f, %4.2f", frontRightPower, frontLeftPower);
            telemetry.addData("Back Wheel Power Right/Left", "%4.2f, %4.2f", backRightPower, backLeftPower);
            telemetry.addData("Current Rotation (Degrees)", "%.2f", heading);
            telemetry.update();
        }
    }
}
