package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

// TODO: switch turret_servo to continuous rotation servo with servo programmer
// TODO: add encoder code and track position with ticks

public class TurretAlignerTest extends OpMode {
    AprilTagWebcam webcam = new AprilTagWebcam();
    private TurretAligner turret = new TurretAligner();

    // tuning constants
    private static final double kP_step = 0.0001;
    private static final double kD_step = 0.00001;
    private boolean wasUp, wasDown, wasRight, wasLeft;


    public void init() {
        webcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);

        telemetry.addLine("initialized all mechanisms");
    }

    public void start() {
        turret.resetTimer(); // turret timer is on the same page
    }

    @Override
    public void loop() {

        // tuning PD controller on gamepad
        // dpad up down for kP
        // dpad left right for kD
        double scale = gamepad1.left_bumper ? 10.0 : 1.0;

        if (gamepad1.dpad_up && !wasUp) {
            turret.setkP(turret.getkP() + kP_step * scale);
        }
        if (gamepad1.dpad_down && !wasDown) {
            turret.setkP(Math.max(0.0, turret.getkP() - kP_step * scale));
        }
        if (gamepad1.dpad_right && !wasRight) {
            turret.setkD(turret.getkD() + kD_step * scale);
        }
        if (gamepad1.dpad_left && !wasLeft) {
            turret.setkD(Math.max(0.0, turret.getkD() - kD_step * scale));
        }

        wasUp = gamepad1.dpad_up;
        wasDown = gamepad1.dpad_down;
        wasRight = gamepad1.dpad_right;
        wasLeft = gamepad1.dpad_left;


        webcam.update();
        AprilTagClusterDetection target = webcam.getClusterByName("RED SCORING"); // tags 30 31 32 33 cluster in biobuzz is RED SCORING
        turret.update(target);

        // encoder telemetry
        telemetry.addData("Turret ticks from center", turret.getTurretTicks());

        // scoring cluster telemerty
        if (target == null) {
            telemetry.addLine("RED SCORING cluster not visible");
        } else {
            telemetry.addData("Cluster", target.metadata.name);
            telemetry.addData("Visible", target.percentClusterFound);

            if (target.ftcPose != null) {
                telemetry.addData("Bearing angle",target.ftcPose.bearing);
            } else {
                telemetry.addLine("Cluster detected, no pos though");
            }
        }
    }
}
