package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwareCode.PIDConfig;

@TeleOp(name = "PID Rot Test", group = "Tests")
public class PIDRotTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    //Index to select the current step size from the array
    int stepIndex = 2;
    public static double r_kD = 0.002, r_kI = 0.000001, r_kP = .008;
    PIDConfig LaunchPID = new PIDConfig(r_kP, r_kI, r_kD);
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    double targetAngle = 90;
    double oldAngle = 0;
    double timeOld = 0;
    double yaw;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        imu.resetYaw();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("right bumper & left bumper to increase or decrease target angle");
            if (gamepad1.rightBumperWasPressed()) {
                targetAngle += 5;
            }
            if (gamepad1.leftBumperWasPressed()) {
                targetAngle -= 5;
            }

            telemetry.addLine("press A to go through step sizes");
            if (gamepad2.aWasPressed()) {
                stepIndex = (stepIndex +1) % stepSizes.length; //Modulo wraps the index back to 0
            }

            if (gamepad1.dpadLeftWasPressed()) {
                LaunchPID.setkP(LaunchPID.getkP() - stepSizes[stepIndex]);
            }
            telemetry.addLine("dpad right & dpad left to increase or decrease proportion");
            if (gamepad1.dpadRightWasPressed()) {
                LaunchPID.setkP(LaunchPID.getkP() + stepSizes[stepIndex]);
            }
            telemetry.addLine("dpad up & dpad down to increase or decrease dampening");
            if (gamepad1.dpadUpWasPressed()) {
                LaunchPID.setkD(LaunchPID.getkD() + stepSizes[stepIndex]);
            }
            if (gamepad1.dpadDownWasPressed()) {
                LaunchPID.setkD(LaunchPID.getkD() - stepSizes[stepIndex]);
            }
            telemetry.addLine("press X & Y to increase or decrease integral");
            if (gamepad1.xWasPressed()) {
                LaunchPID.setkI(LaunchPID.getkI() + stepSizes[stepIndex]);
            }
            if (gamepad1.yWasPressed()) {
                LaunchPID.setkI(LaunchPID.getkI() - stepSizes[stepIndex]);
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw();

            if (gamepad1.a) {
                yaw = pid(r_kP, r_kI, r_kD, oldAngle, timeOld, targetAngle - heading, runtime.milliseconds());
            }
            else {
                yaw = 0;
            }

            frontleft.setPower(-yaw);
            frontright.setPower(yaw);
            backleft.setPower(-yaw);
            backright.setPower(yaw);

            timeOld = runtime.milliseconds();
            oldAngle = targetAngle - heading;

            //---------------------------------telemetry------------------------------------

            telemetry.addData("step size: ", stepSizes[stepIndex]);
            telemetry.addData("Proportion: ", r_kP);
            telemetry.addData("Integral: ", r_kI);
            telemetry.addData("Dampening:", r_kD);
            telemetry.addData("current angle: ", heading);
            telemetry.addData("wheel abs power: ", yaw);
            telemetry.addData("target angle: ", targetAngle);
            telemetry.update();
        }
    }
    public double pid(double proportion, double integral, double dampening, double distOld, double tOld, double distNew, double tNew){

        double p = distNew * proportion;

        double i = 0;

        double f_integralSum = (tNew - tOld) * ((distOld + distNew) / 2);
        i = f_integralSum * integral;

        double vel = (distNew - distOld)/(tNew - tOld);
        double d = -vel * dampening;

        return p + i + d;
    }
}