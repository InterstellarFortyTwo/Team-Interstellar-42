package org.firstinspires.ftc.teamcode.TeacherExamples;

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
public class autoExample extends LinearOpMode {
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
    double targetVel = 1680;
    boolean shooting = false;
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
        right_intake_servo = hardwareMap.get(CRServo.class, "rightservo");
        left_intake_servo = hardwareMap.get(CRServo.class, "leftservo");
        imu = hardwareMap.get(IMU.class, "imu");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        time *= (1/power);

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        /*
        I'm using field centric drive to allow more movements while making it fairly simple.
        This allows us to turn while we are moving.
         */
        double axial = y * cos(-heading) - x * sin(-heading);
        double lateral = y * sin(-heading) + x * cos(-heading);

        double error = targetAngle - heading;
        if (targetAngle != 0) {
            while (heading != targetAngle) {
                yaw = rotateSpeed * error / targetAngle;
            }
        }
        else {
            yaw = 0;
        }

        double frontLeftPower = axial - lateral + yaw * power;
        double frontRightPower = axial + lateral - yaw * power;
        double backLeftPower = axial + lateral + yaw * power;
        double backRightPower = axial - lateral - yaw * power;

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

        sleep((long) time);

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    public void intake(double time) {
        intake.setPower(1);
        right_intake_servo.setPower(-1);
        left_intake_servo.setPower(1);

        sleep((long) time);

        intake.setPower(0);
        right_intake_servo.setPower(0);
        left_intake_servo.setPower(0);
    }
    public void shoot() {

        shooting = true;
        shooterTimer.reset();

        double currentVel = flywheel.getVelocity();
        double error = 0;
        double shootSpeed = 10;
        double lastError;
        double ballShot = 0;

        while (shooting) {
            if (error != 0) {
                error = targetVel - currentVel;
            }
            else {
                error = 0.1;
            }
            double shootPower = (shootSpeed * error) / targetVel;
            flywheel.setPower(shootPower);

            if (-10 < error && error < 10) {
                windmill.setPower(1);
                lastError = 1700;
            }
            else {
                windmill.setPower(0);
                lastError = 0;
            }

            if((error - lastError) < 0) {
                ballShot += 1;
            }
            if (ballShot >= 4) {
                shooting = false;
                flywheel.setPower(0);
            }
        }
    }
}

