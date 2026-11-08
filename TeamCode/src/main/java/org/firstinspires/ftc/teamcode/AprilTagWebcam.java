package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private static final String webcam_name = "FGLs Webcam 2026-7";

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, webcam_name));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        if (aprilTagProcessor != null) {
            detectedTags = aprilTagProcessor.getDetections();
        }
    }

    public List<AprilTagDetection> getDetectedTag() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detection) {
        if (detection == null) {
            return;
        }

        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;

            if (single.metadata != null && single.ftcPose != null) {
                telemetry.addLine(String.format("\n (ID %d) %s", single.id, single.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", single.ftcPose.x, single.ftcPose.y, single.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", single.ftcPose.pitch, single.ftcPose.roll, single.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", single.ftcPose.range, single.ftcPose.bearing, single.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n (ID %d) Unknown", single.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", single.center.x, single.center.y));
            }
        } else if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

            telemetry.addLine(String.format("\n Cluster %s (%d%% found)", cluster.metadata.name, cluster.percentClusterFound));
            if (cluster.ftcPose != null) {
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", cluster.ftcPose.pitch, cluster.ftcPose.roll, cluster.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.elevation));
            }
        }
    }

    public AprilTagSingleDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.id == id) {
                    return single;
                }
            }
        }
        return null;
    }

    public AprilTagClusterDetection getClusterByName(String name) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (name.equals(cluster.metadata.name)) {
                    return cluster;
                }
            }
        }
        return null;
    }
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public AprilTagProcessor getProcessor() {
        return aprilTagProcessor;
    }
}