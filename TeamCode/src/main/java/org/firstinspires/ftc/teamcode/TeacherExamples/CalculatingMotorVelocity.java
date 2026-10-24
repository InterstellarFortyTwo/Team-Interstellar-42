package org.firstinspires.ftc.teamcode.TeacherExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Motor Velocity", group = "LinearOpMode")
@Disabled
public class CalculatingMotorVelocity extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            //We use a DcMotorEx instead of a regular DcMotor, which lets us measure the speed of the motor.
            DcMotorEx motor;
            motor = hardwareMap.get(DcMotorEx.class, "motor_name");
            //This is just test code, the variables don't work for all motors.
            double ticksPerRev = 537.7;
            double rpm = (motor.getVelocity() / ticksPerRev) * 60;
            double pulley_p;
            double launcher_p;
            double ticksPerSecond = motor.getVelocity();

            waitForStart();
            while (opModeIsActive()) {

                if (gamepad1.a) {
                    launcher_p = 1.0;
                }
                else {
                    launcher_p = 0;
                }
                //Math.abs is absolute value
                //The code makes sure the motor for the launcher is greater than 500 rpm to fire the ball.
                //For a better design however, most teams use servos to stop a ball from firing instead of stoping it with a motor.
                if (Math.abs(rpm) > 500) {
                    pulley_p = 1.0;
                }
                else {
                    pulley_p = 0.0;
                }

                motor.setPower(launcher_p);
        }
        }
}