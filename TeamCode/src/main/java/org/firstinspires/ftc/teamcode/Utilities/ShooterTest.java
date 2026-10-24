package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "ShooterTest", group = "LinearOpMode")
public class ShooterTest extends LinearOpMode {

    boolean cycle = false;
    double launcher = 0.0;
    double targetflywheelRPM;
    double heightMeters = 3;
    double v = heightMeters + 5;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CRServo right_launch_servo = hardwareMap.get(CRServo.class, "rightServo");
        CRServo left_launch_servo = hardwareMap.get(CRServo.class, "leftServo");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()) {

            double ticksPerRev = 537.7;
            double flywheelVelocity = flywheel.getVelocity();
            double rpm = (flywheelVelocity / ticksPerRev) * 60;

            targetflywheelRPM = (2.3*v / 0.1);


            telemetry.addLine("Press a to fire");
            if(gamepad1.a) {
                if(rpm <= targetflywheelRPM) {
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                    launcher = 1.0;
                    if(cycle) {
                        cycle = !cycle;
                    }
                }
                if(rpm >= targetflywheelRPM && rpm <= targetflywheelRPM + 30) {
                    if(cycle) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                    }
                    else {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                }
                if(rpm > targetflywheelRPM + 30) {
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

            flywheel.setPower(launcher);
            telemetry.update();
        }
    }

}
