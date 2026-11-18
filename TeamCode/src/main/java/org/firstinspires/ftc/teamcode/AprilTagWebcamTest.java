package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

// test to see if webcam works basically

@TeleOp(name = "AprilTag Webcam Test", group = "4VTest")
public class AprilTagWebcamTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagWebcam webcam = new AprilTagWebcam();

        try {
            webcam.init(hardwareMap, telemetry);

            telemetry.addLine("Camera initialized. Press START.");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                webcam.update();

                telemetry.addData(
                        "Tags detected",
                        webcam.getDetectedTag().size()
                );

                for (AprilTagDetection detection : webcam.getDetectedTag()) {
                    webcam.displayDetectionTelemetry(detection);
                }

                telemetry.update();
                sleep(20);
            }
        } finally {
            webcam.stop();
        }
    }
}