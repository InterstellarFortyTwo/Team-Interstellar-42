package org.firstinspires.ftc.teamcode.TeacherExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//I'm not sure if we'll need this or not.
@Disabled
@TeleOp(name = "DistanceSensors")
public class DistanceSensors extends LinearOpMode {

    @Override
    public void runOpMode() {
        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("range Millimeters", sensorDistance.getDistance(DistanceUnit.MM));
            telemetry.addData("range Centimeters", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("range Meters", sensorDistance.getDistance(DistanceUnit.METER));
            telemetry.addData("range Inches", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
