package org.firstinspires.ftc.teamcode.TeacherExamples;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//This is a little more complicated, but just know this makes the robot easier to drive.
@TeleOp(name = "Field Centric Toggle", group = "LinearOpMode")
@Disabled
public class FieldCentricToggle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        double heading = 0;
        double x;
        double y;

        boolean lastA = false;
        boolean fieldToggle = false;

        double forward_p;
        double right_p;
        double spin_p;

        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {

            y = gamepad1.left_stick_y;
            spin_p = gamepad1.right_stick_x;
            x = gamepad1.left_stick_x;

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //Makes sure pressing the button only toggles the field centric movement once
            if(gamepad1.x && !lastA) {
                fieldToggle = !fieldToggle;
            }
            lastA = gamepad1.x;
            //calculates how much power is directed into each wheel while using field centric movement
            if(fieldToggle) {
                forward_p = y*cos(-heading) - x * sin(-heading);
                right_p = y *sin(-heading) + x*cos(-heading);
            }
            else {
                forward_p = y;
                right_p = x;

                double frontRightPower = forward_p + spin_p + right_p;
                double frontLeftPower = forward_p - spin_p - right_p;
                double backRightPower = forward_p + spin_p - right_p;
                double backLeftPower = forward_p - spin_p + right_p;
            }
    }
}
}
