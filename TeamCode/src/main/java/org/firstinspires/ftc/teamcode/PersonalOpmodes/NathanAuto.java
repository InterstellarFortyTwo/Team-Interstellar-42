package org.firstinspires.ftc.teamcode.PersonalOpmodes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name ="Auto Example", group = "examples")
public class NathanAuto extends LinearOpMode {
    // I state my variables up here so they are within the scope of all my methods
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor intake;
    DcMotorEx flywheel;
    CRServo windmill;
    CRServo left_intake_servo;
    CRServo right_intake_servo;
    IMU imu;
    double heading;
    double yaw;
    boolean shooting = false;
    // Creates new time variables
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime shooterTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backright = hardwareMap.get(DcMotor.class, "backright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        windmill = hardwareMap.get(CRServo.class, "windmill");
        right_intake_servo = hardwareMap.get(CRServo.class, "rightServo");
        left_intake_servo = hardwareMap.get(CRServo.class, "leftServo");
        imu = hardwareMap.get(IMU.class, "imu");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        imu.resetYaw();
        waitForStart();


        telemetry.addData("Runtime: ", runtime);
    }
    public void move(double y, double x, double targetAngle, double rotateSpeed, long time, double power) throws InterruptedException {

        //The power changes the move speed of the robot so the time it takes for the robot to move is also increased
        time *= (1/power);

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double Degrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        /*
        I'm using field centric drive to allow more movements while making it fairly simple.
        This allows us to turn while we are moving.
         */
        double axial = y * cos(-heading) - x * sin(-heading);
        double lateral = y * sin(-heading) + x * cos(-heading);

        double error = targetAngle - Degrees;
        if (targetAngle != 0) {
            if (Degrees != targetAngle) {
                yaw = rotateSpeed * error / targetAngle;
            }
            else {
                yaw = 0;
            }
        }
        else {
            yaw = error / 90;
        }
        /*
        This is where I state all the variables for movement in an array like pattern for mechanum wheels.
        Remember the forward and sideways movement are not controlled directly from the variables that can be
        stated in the method, but in the field centric math.
         */
        double frontLeftPower = (axial - lateral + yaw) * power;
        double frontRightPower = (axial + lateral - yaw) * power;
        double backLeftPower = (axial + lateral + yaw) * power;
        double backRightPower = (axial - lateral - yaw) * power;

        //Makes sure power does not exceed 1 or -1
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

        frontleft.setPower(frontLeftPower);
        frontright.setPower(frontRightPower);
        backleft.setPower(backLeftPower);
        backright.setPower(backRightPower);

        //Makes the wheels brake only when all motors have zero power
        if (frontRightPower == 0) {
            if (frontLeftPower == 0) {
                if (backRightPower == 0) {
                    if (backLeftPower == 0) {
                        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }
            }
        }
        else {
            frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //tells the robot how long to move
        sleep((long) time);

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    public void intake(double time) {
        //sets power to motors for a given amount of time
        intake.setPower(1);
        right_intake_servo.setPower(-1);
        left_intake_servo.setPower(1);

        sleep((long) time);

        intake.setPower(0);
        right_intake_servo.setPower(0);
        left_intake_servo.setPower(0);
    }
    public void shoot(double targetVel) {

        shooting = true;
        shooterTimer.reset();

        double currentVel = flywheel.getVelocity();
        double flywheelPower = 0;

        while (shooting) {

            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            if (currentVel < targetVel) {
                if (currentVel > (targetVel / 2)) {
                    double normalVel = currentVel / targetVel;
                    flywheelPower = 1 - (2 * (normalVel - 0.5));
                    if (flywheelPower < 0) {
                        flywheelPower = 0;
                    }
                }
                else {
                    flywheelPower = 1;
                }
            }
            else {
                flywheelPower = 0;
            }
            flywheel.setPower(flywheelPower);

            if (targetVel - currentVel >= -50 && targetVel - currentVel <= 50) {
                windmill.setPower(1);
            }
            else {
                windmill.setPower(0);
            }

            sleep(5000);

            shooting = false;
        }
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}


