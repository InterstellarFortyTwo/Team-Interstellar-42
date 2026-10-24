package org.firstinspires.ftc.teamcode.TeacherExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Example", group = "LinearOpMode")
@Disabled
public class ServoExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoName = hardwareMap.get(Servo.class, "servo_name");

        waitForStart();

        while (opModeIsActive()) {
            // 3. Set the position (0.0 to 1.0)
            if (gamepad1.x) {
                servoName.setPosition(1.0); // Move to one end
            } else if (gamepad1.b) {
                servoName.setPosition(0.0); // Move to the other end
            } else if (gamepad1.y) {
                servoName.setPosition(0.5); // Move to center
            }
        }
    }


}
