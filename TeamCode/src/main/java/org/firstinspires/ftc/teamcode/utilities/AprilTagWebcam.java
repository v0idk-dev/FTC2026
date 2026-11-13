package org.firstinspires.ftc.teamcode.utilities;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagWebcam {

    // old default name, used if you call init() without a camera name
    public static final String default_webcam_name = "FGLs Webcam 2026-7";

    private String webcamName = default_webcam_name;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    private boolean exposureApplied = false;

    // old way, still works (uses default_webcam_name)
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        init(hwMap, telemetry, default_webcam_name, -1);
    }

    // cameraName = robot config name ("viz1" / "viz2")
    // liveViewId = id from VisionPortal.makeMultiPortalView() when running 2 cameras, -1 for default
    public void init(HardwareMap hwMap, Telemetry telemetry, String cameraName, int liveViewId) {
        this.telemetry = telemetry;
        this.webcamName = cameraName;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, cameraName));
        builder.setCameraResolution(new Size(640, 480));
        if (liveViewId >= 0) {
            builder.setLiveViewContainerId(liveViewId);
        }
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public String getName() {
        return webcamName;
    }

    public void update() {
        if (aprilTagProcessor != null) {
            detectedTags = aprilTagProcessor.getDetections();
        }
    }

    public List<AprilTagDetection> getDetectedTag() {
        return detectedTags;
    }

    // ---------- exposure ----------
    // short exposure + higher gain = less motion blur while driving
    // camera has to be STREAMING first, so call this every init_loop until it returns true
    // TODO: find good values with the SDK sample ConceptAprilTagOptimizeExposure
    public boolean setManualExposure(int exposureMs, int gain) {
        if (visionPortal == null) return false;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return false;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        exposureApplied = true;
        return true;
    }

    public boolean isExposureApplied() {
        return exposureApplied;
    }

    // ---------- lookups ----------
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

    // best visible cluster whose name starts with prefix ("RED" / "BLUE"), picked by % of tags found
    // TODO: we only know "RED SCORING" (tags 30-33) for sure. check the other cluster names with
    //  getVisibleClusterNames() and make sure the prefix actually picks the cell we want to shoot at
    public AprilTagClusterDetection getBestClusterStartingWith(String prefix) {
        AprilTagClusterDetection best = null;
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata == null || cluster.ftcPose == null) continue;
                if (!cluster.metadata.name.startsWith(prefix)) continue;
                if (best == null || cluster.percentClusterFound > best.percentClusterFound) {
                    best = cluster;
                }
            }
        }
        return best;
    }

    // for telemetry, so we can read the real cluster names off the driver station
    public String getVisibleClusterNames() {
        StringBuilder sb = new StringBuilder();
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata == null) continue;
                if (sb.length() > 0) sb.append(", ");
                sb.append(cluster.metadata.name);
            }
        }
        return sb.length() == 0 ? "none" : sb.toString();
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
