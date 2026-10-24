package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.hardwareCode.PIDConfig;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOp Rough Draft")
@Config
public class TeleOpRoughDraft extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double xPos;
    double yPos;
    double targetX;
    double targetY;
    double changeSides;
    double distToTarget;
    double angleToTarget;
    double r_error;
    double oldRot = 0;
    boolean isTargeting;

    boolean isShooting = false;
    double flywheelVel;
    double targetFlywheelVel;
    double deltaY = 1.3589;
    double time;
    double targetBallSpeed;
    double oldFlywheelDist = 0;
    double oldTime = 0;
    double flywheel_p;
    double ballsShot;
    double artifact = 0;
    boolean pluh = true;
    double oldShooterTime = 0;

    double r_integralSum;
    double f_integralSum;
    public static double r_kD = 0.006, r_kI, r_kP = .03;
    public static double f_kD = 0.002, f_kI = 0.000001, f_kP = .008;

    double frontLeftPower;
    double frontRightPower;
    double backRightPower;
    double backLeftPower;
    double yaw;
    double axial;
    double lateral;
    boolean lastA;
    boolean fieldToggle;
    double y;
    double x;
    double rot;
    double heading;
    double moveSpeed;

    DcMotor frontleft;
    DcMotor backleft;
    DcMotor frontright;
    DcMotor backright;
    DcMotorEx flywheel;
    DcMotorEx intake;
    CRServo right_intake_servo;
    CRServo left_intake_servo;
    CRServo windmill;
    IMU imu;
    Limelight3A limelight;
    OctoQuad octoQuad;

        @Override
        public void runOpMode() throws InterruptedException {

            frontleft = hardwareMap.get(DcMotor.class, "frontleft");
            frontright = hardwareMap.get(DcMotor.class, "frontright");
            backleft = hardwareMap.get(DcMotor.class, "backleft");
            backright = hardwareMap.get(DcMotor.class, "backright");
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            windmill = hardwareMap.get(CRServo.class, "windmill");
            right_intake_servo = hardwareMap.get(CRServo.class, "rightServo");
            left_intake_servo = hardwareMap.get(CRServo.class, "leftServo");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            limelight.setPollRateHz(90);
            limelight.start();
            limelight.pipelineSwitch(0);

            frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontright.setDirection(DcMotorSimple.Direction.REVERSE);
            backleft.setDirection(DcMotorSimple.Direction.FORWARD);
            backright.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            windmill.setDirection(DcMotorSimple.Direction.FORWARD);
            right_intake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            left_intake_servo.setDirection(DcMotorSimple.Direction.REVERSE);

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

            imu = hardwareMap.get(IMU.class, "imu");

            ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

            IMU.Parameters parameters = new IMU.Parameters(orientation);
            imu.initialize(parameters);

            telemetry.addData("Status", "IMU Calibrating...");
            telemetry.update();

            imu.resetYaw();
            waitForStart();
            runtime.reset();
            while (opModeIsActive()) {

                heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                flywheelVel = flywheel.getVelocity();
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                if (isShooting) {
                    yaw = rot;
                } else {
                    yaw = gamepad1.right_stick_x;
                }


                //-----------------------------------Limelight & Odometry Pods---------------------------------------


                if (gamepad1.rightBumperWasPressed()) {
                    changeSides = -1;
                    telemetry.addData("team: ", "blue");
                } else if (gamepad1.leftBumperWasPressed()) {
                    changeSides = 1;
                    telemetry.addData("team: ", "red");
                }

                limelight.updateRobotOrientation(heading + 90);
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    xPos = result.getBotpose().getPosition().x * 39.37;
                    yPos = result.getBotpose().getPosition().y * 39.37;
                    double llRot = result.getBotpose().getOrientation().getYaw();

                    if (yPos > 0) {
                        targetY = 8;
                        targetX = -12.75 * changeSides;
                    } else if (yPos < 0) {
                        targetY = -8;
                        targetX = -12.75 * changeSides;
                    }

                    double xDistToHive = Math.abs(targetX - xPos);
                    double yDistToHive = targetY - yPos;

                    distToTarget = Math.sqrt((xDistToHive * xDistToHive) + (yDistToHive * yDistToHive));
                    angleToTarget = Math.toDegrees(Math.atan2(xDistToHive, yDistToHive));
                    r_error = -(angleToTarget - (heading));

                    isTargeting = true;

                    telemetry.addData("Limelight", "Targeting");
                } else {
                    telemetry.addData("Limelight", "No Targets");
                    isTargeting = false;
                }


                //------------------------------------Movement----------------------------------------


                telemetry.addLine("Press Left Stick to change move speed");
                if (gamepad1.leftStickButtonWasPressed()) {
                    if (moveSpeed == 1) {
                        moveSpeed = 0.5;
                    } else {
                        moveSpeed = 1;
                    }
                }

                if (gamepad1.dpad_up && !lastA) {
                    fieldToggle = !fieldToggle;
                }
                lastA = gamepad1.dpad_up;

                if (fieldToggle) {
                    axial = y * cos(-heading) - x * sin(-heading);
                    lateral = y * sin(-heading) + x * cos(-heading);
                    telemetry.addLine("Field Centric Toggled: On");
                } else {
                    axial = y;
                    lateral = x;
                    telemetry.addLine("Field Centric Toggled: Off");
                }

                frontRightPower = (axial + yaw + lateral) * moveSpeed;
                frontLeftPower = (axial - yaw - lateral) * moveSpeed;
                backRightPower = (axial + yaw - lateral) * moveSpeed;
                backLeftPower = (axial - yaw + lateral) * moveSpeed;

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


                //----------------------------flywheel power calculations---------------------------


                if (isTargeting) {
                    time = distToTarget / targetBallSpeed * cos(78);
                    targetBallSpeed = (-5 * (time * time) - deltaY) / (time * sin(78));
                }
                else {
                    targetBallSpeed = 28;
                }
                targetFlywheelVel = (targetBallSpeed / (Math.PI * 0.075)) * 537.6;

                if(targetFlywheelVel != 0) {
                    flywheel_p = pid(f_kP, f_kI, f_kD, oldFlywheelDist, oldTime, targetFlywheelVel - flywheelVel, runtime.milliseconds(), "f");
                }
                else {
                    f_integralSum = 0;
                    flywheel_p = 0;
                }


                //-----------------------------------Shooting---------------------------------------


                if (gamepad1.bWasPressed()) {
                    isShooting = !isShooting;
                    shootTimer.reset();
                }

                if (isShooting) {
                    rot = pid(r_kP, r_kI, r_kD, oldRot, oldTime, r_error, runtime.milliseconds(), "r");

                    if (Math.abs(r_error) < 3.5 && targetFlywheelVel - flywheelVel > -0.5) {
                    windmill.setPower(1);
                    }
                    else {
                    windmill.setPower(0);
                    }
                }
                else {
                    ballsShot = 0;
                }

                if ((flywheelVel - targetFlywheelVel) / (shootTimer.time()) < -10) {
                    ballsShot += 1;
                    shootTimer.reset();
                }
                if (ballsShot >= 4) {
                    isShooting = false;
                }

                flywheel.setPower(flywheel_p);

                oldRot = angleToTarget;
                oldTime = runtime.milliseconds();
                oldFlywheelDist = targetFlywheelVel - flywheelVel;


                //-------------------------------intake--------------------------------


                if (gamepad1.x && pluh) {
                    left_intake_servo.setPower(1);
                    right_intake_servo.setPower(1);
                    intake.setPower(1);
                }
                else {
                    left_intake_servo.setPower(0);
                    right_intake_servo.setPower(0);
                    intake.setPower(0);
                }
                double intakeVel = intake.getVelocity();

                if (gamepad1.bWasPressed()) {
                    artifact += 1;
                }
                if (artifact >= 4) {
                    pluh = false;
                }
                else {
                    pluh = true;
                }
                if (isShooting) {
                    artifact = 0;
                }


                //---------------------telemetry--------------------

            }
        }
    public double pid(double proportion, double integral, double dampening, double distOld, double tOld, double distNew, double tNew, String letterCode){

        double p = distNew * proportion;

        double i = 0;

        if(letterCode == "r"){
            r_integralSum += (tNew-tOld)*((distOld+distNew)/2);
            i = r_integralSum * integral;
        }
        else if(letterCode == "f"){
            f_integralSum += (tNew-tOld)*((distOld+distNew)/2);
            i = f_integralSum * integral;
        }


        double vel = (distNew - distOld)/(tNew - tOld);
        double d = -vel * dampening;

        return p + i + d;
    }
}

