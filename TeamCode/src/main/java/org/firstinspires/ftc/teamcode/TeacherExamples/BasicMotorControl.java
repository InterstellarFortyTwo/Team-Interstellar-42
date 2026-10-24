package org.firstinspires.ftc.teamcode.TeacherExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "BasicMotorControl", group = "LinearOpMode")
public class BasicMotorControl extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //In some cases you only need two motors for tank drive, but in this case I have four
        DcMotor frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontright = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backleft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        double axial;
        double yaw;

        waitForStart();
        while (opModeIsActive()) {

            axial = gamepad1.left_stick_y;
            yaw = gamepad1.right_stick_x;

            double left = axial - yaw;
            double right = axial + yaw;

            double max;
            //This makes sure the motors don't exceed 1.0/-1.0
            max = Math.max(Math.abs(left), Math.abs(right));

            if (max > 1.0) {
                right /= max;
                left /= max;
            }

            frontright.setPower(right);
            frontleft.setPower(left);
            backright.setPower(right);
            backleft.setPower(left);

            telemetry.addData("Wheel Power Right/Left", "%4.2f, %4.2f", right, left);
            telemetry.update();
        }
    }
}