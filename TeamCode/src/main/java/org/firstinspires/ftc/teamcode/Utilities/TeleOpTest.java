package org.firstinspires.ftc.teamcode.Utilities;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "Test", group = "LinearOpMode")
public class TeleOpTest extends LinearOpMode {

    ElapsedTime Time = new ElapsedTime();

    double heading = 0;
    double x;
    double y;
    double moveSpeed = 1.0;

    boolean lastA = false;
    boolean fieldToggle = false;
    boolean cycle = false;
    double launcher = 0.0;

    double axial;
    double lateral;
    double yaw;

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

        imu.resetYaw();
        Time.reset();

        waitForStart();
        while(opModeIsActive()) {

            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double flywheelVelocity = flywheel.getVelocity();

            telemetry.addLine("Press Left Stick to change move speed");
            if (gamepad1.leftStickButtonWasPressed()) {
                if (moveSpeed == 1) {
                    moveSpeed = 0.5;
                } else {
                    moveSpeed = 1;
                }
            }
            telemetry.addData("Move speed = ", moveSpeed);

            telemetry.addLine("Press Dpad up to toggle field centric");
            if(gamepad1.dpad_up && !lastA) {
                fieldToggle = !fieldToggle;
            }
                lastA = gamepad1.dpad_up;

            if (fieldToggle) {
                axial = y*cos(-heading) - x * sin(-heading);
                lateral = y *sin(-heading) + x*cos(-heading);
                telemetry.addLine("Field Centric Toggled: On");
            }
            else {
                axial = y;
                lateral = x;
                telemetry.addLine("Field Centric Toggled: Off");
            }

            telemetry.addLine("Press Right Stick to reset yaw");
            if(gamepad1.rightStickButtonWasPressed()) {
                imu.resetYaw();
            }

            telemetry.addLine("Press a to fire");
            if(gamepad1.a) {
                if(flywheelVelocity <= 1500) {
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                    launcher = 1.0;
                    if(cycle) {
                        cycle = !cycle;
                    }
                }
                if(flywheelVelocity >= 1500 && flywheelVelocity < 1800) {
                    if(cycle) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                    }
                    else {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                }
                if(flywheelVelocity > 1800) {
                    launcher = 0.0;
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


            axial *= moveSpeed;
            yaw *= moveSpeed;
            lateral *= moveSpeed;

            double frontRightPower = axial + yaw + lateral;
            double frontLeftPower = axial - yaw - lateral;
            double backRightPower = axial + yaw - lateral;
            double backLeftPower = axial - yaw + lateral;

            if (frontRightPower <= 0 && frontRightPower >= 0 ) {
                if (frontLeftPower <= 0 && frontLeftPower >= 0) {
                    if (backRightPower <= 0 && backRightPower >= 0) {
                        if (backLeftPower <= 0 && backLeftPower >=0) {
                            FRONT_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            FRONT_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            BACK_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            BACK_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }
                }
                }
            else {
                FRONT_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                FRONT_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BACK_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BACK_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            telemetry.addData("Status", "Run time: " + Time.toString());
            telemetry.addData("Front Wheel Power Right/Left", "%4.2f, %4.2f", frontRightPower, frontLeftPower);
            telemetry.addData("Back Wheel Power Right/Left", "%4.2f, %4.2f", backRightPower, backLeftPower);
            telemetry.addData("Current Rotation (Degrees)", "%.2f", heading);
            telemetry.update();
        }
    }

}
