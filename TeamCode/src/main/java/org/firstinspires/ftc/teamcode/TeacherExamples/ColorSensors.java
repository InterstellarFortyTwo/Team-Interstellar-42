package org.firstinspires.ftc.teamcode.TeacherExamples;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//I'm not exactly sure what to make out of this code, but until then we will learn all the other aspects of coding a robot.
@Disabled
@TeleOp(name = "ColorSensors")
public class ColorSensors extends LinearOpMode {

    @Override
    public void runOpMode() {

        float gain = 2;
        final float[] hsvValues = new float[3];

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                gain += 0.005F;
            } else if (gamepad1.b && gain > 1) {
                gain -= 0.005F;
            }

            telemetry.addData("Gain", gain);
            colorSensor.setGain(gain);

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);

            telemetry.update();

        }
    }

}
