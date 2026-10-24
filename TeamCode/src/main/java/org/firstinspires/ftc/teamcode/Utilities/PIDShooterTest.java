package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwareCode.PIDConfig;

@TeleOp(name = "PID Shooter Test", group = "Tests")
public class PIDShooterTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    //Index to select the current step size from the array
    int stepIndex = 2;
    public static double f_kD = 0.0, f_kI = 0.0, f_kP = 0.05;
    PIDConfig LaunchPID = new PIDConfig(f_kP, f_kI, f_kD);
    DcMotorEx flywheel;
    DcMotor intake;
    double targetVel = 1280;
    double oldFlywheelVel = 0;
    double timeOld = 0;
    boolean isShooting = false;
    double intake_power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        CRServo windmill = hardwareMap.get(CRServo.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("right bumper & left bumper to increase or decrease target velocity");
            if (gamepad1.right_bumper) {
                targetVel += 10;
            }
            if (gamepad1.left_bumper) {
                targetVel -= 10;
            }

            telemetry.addLine("press B to go through step sizes");
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex +1) % stepSizes.length; //Modulo wraps the index back to 0
            }

            if (gamepad1.dpad_left) {
                LaunchPID.setkP(LaunchPID.getkP() - stepSizes[stepIndex]);
            }
            telemetry.addLine("dpad right & dpad left to increase or decrease proportion");
            if (gamepad1.dpad_right) {
                LaunchPID.setkP(LaunchPID.getkP() + stepSizes[stepIndex]);
            }
            telemetry.addLine("dpad up & dpad down to increase or decrease dampening");
            if (gamepad1.dpad_up) {
                LaunchPID.setkD(LaunchPID.getkD() + stepSizes[stepIndex]);
            }
            if (gamepad1.dpad_down) {
                LaunchPID.setkD(LaunchPID.getkD() - stepSizes[stepIndex]);
            }
            telemetry.addLine("press X & Y to increase or decrease integral");
            if (gamepad1.x) {
                LaunchPID.setkI(LaunchPID.getkI() + stepSizes[stepIndex]);
            }
            if (gamepad1.y) {
                LaunchPID.setkI(LaunchPID.getkI() - stepSizes[stepIndex]);
            }

            f_kP = LaunchPID.getkP();
            f_kI = LaunchPID.getkI();
            f_kD = LaunchPID.getkD();

            double flywheelVel = -flywheel.getVelocity();

            if (gamepad1.a) {
                isShooting = true;
            }
            else isShooting = false;

            if(isShooting) {
                flywheel.setPower(pid(f_kP, f_kI, f_kD, oldFlywheelVel, timeOld, targetVel - flywheelVel, runtime.milliseconds()));
            }
            else flywheel.setPower(0);

            if (gamepad1.b) {
                intake_power = 1;
            }
            else {
                intake_power = 0;
            }
            intake.setPower(intake_power);

            if (intake_power == 0 && targetVel - flywheelVel >= -1000 && targetVel - flywheelVel <= 1000) {
                windmill.setPower(1);
            }
            else {
                windmill.setPower(0);
            }

            timeOld = runtime.milliseconds();
            oldFlywheelVel = targetVel - flywheelVel;


            //---------------------------------telemetry------------------------------------

            telemetry.addData("step size: ", stepSizes[stepIndex]);
            telemetry.addData("Proportion: ", f_kP);
            telemetry.addData("Integral: ", f_kI);
            telemetry.addData("Dampening: ", f_kD);
            telemetry.addData("flywheel speed: ", flywheelVel);
            telemetry.addData("flywheel power: ", pid(f_kP, f_kI, f_kD, oldFlywheelVel, timeOld, targetVel - flywheelVel, runtime.milliseconds()));
            telemetry.addData("target velocity: ", targetVel);
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
