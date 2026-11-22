package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.TurretAligner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@TeleOp(name = "TeleOpPrelim_vision", group = "0Main")
public class TeleOpPrelim_vision extends OpMode {

    static final String VERSION = "v1.8.2 d4.12.5 c4-12-4";

    static final String TL = "tl", TR = "tr", BL = "bl", BR = "br";
    static final String INTK = "intk";
    static final String PUSH = "push";
    static final String ACT_R = "actR";
    static final String ACT_L = "actL";
    static final String FLR = "flr";
    static final String FLL = "fll";
    static final String MV_R = "mvR";
    static final String STP = "stp";
    static final String PPC = "ppc";
    static final String VIZ1 = "FGLs Webcam 2026-7";

    static final boolean USE_VELOCITY = true;
    static final double DRIVE_RPM = 6000;
    static final double INTAKE_RPM = 312;
    static final double PUSH_RPM = 312;
    static final double POLLEN_RPM = 6000;
    static final double NECTAR_RPM = 6000;
    static final double GOBILDA_BARE_RPM = 6000;
    static final double GOBILDA_ENCODER_CPR = 28;
    static final double ENCODER_CHECK_SPEED = 0.20;
    static final double ENCODER_CHECK_TIME = 0.50;

    static final double DRIVE_SPEED = 0.70;
    static final double TURN_SPEED = 0.80;
    static final double DEADZONE = 0.05;
    static final boolean SQUARE_INPUTS = true;
    static final boolean SMOOTH_DRIVE = true;
    static final double RAMP_DOWN_PER_SEC = 2.5;
    static final double RAMP_UP_PER_SEC = 7.5;

    static final double INTAKE_POWER = 1.0;
    static final double OUTTAKE_POWER = -1.0;

    static final double FLOWER_RANGE_DEG = 270;
    static final double FLOWER_ENGAGED_DEG = 155;
    static final double FLOWER_DISENGAGED_DEG = 70;

    static final double POLLEN_REV_TIME = 1.0;
    static final double PUSH_POWER = 1.0;

    static final double NECTAR_REV_TIME = 1.0;
    static final double STOPPER_RANGE_DEG = 270;
    static final double STOPPER_CLOSED_DEG = 135;
    static final double STOPPER_OPEN_DEG = 180;
    static final double NECTAR_OPEN_TIME = 0.25;
    static final double NECTAR_RECOVER_TIME = 0.50;

    static final double TURRET_MANUAL_POWER = 0.30;
    static final boolean INVERT_TURRET = false;

    Motor tl, tr, bl, br;
    Motor intk, push, actR, actL;
    ServoImplEx flr, fll, stp;
    CRServo mvR;
    VisionPortal portal;
    AprilTagProcessor tags;
    Alliance visionColor;
    String trackedCluster = "";
    final TurretAligner turretAligner = new TurretAligner();
    static final long MAX_FRAME_AGE_NS = 300_000_000L;

    final Set<String> crashed = new HashSet<>();
    final List<String> report = new ArrayList<>();
    int missingCount = 0;

    enum Alliance { RED, BLUE }
    Alliance alliance = Alliance.RED;

    boolean intakeToggle = false, outtakeToggle = false;
    String flowerState = "untouched (limp)";
    boolean alignOnShoot = true;

    double flywheelRpm = 2250;
    boolean prevX2, prevB2, prevY2, prevA2;

    boolean pollenHeld = false;
    final ElapsedTime pollenTimer = new ElapsedTime();

    enum NectarPhase { IDLE, REVVING, OPEN, RECOVER }
    NectarPhase nectarPhase = NectarPhase.IDLE;
    final ElapsedTime nectarTimer = new ElapsedTime();
    int nectarShots = 0;

    double curStrafe = 0, curFwd = 0, curTurn = 0;
    final ElapsedTime driveTimer = new ElapsedTime();

    String turretStatus = "idle";
    String positionStatus = "-";

    boolean pA, pB, pBack, pUp, pDown;

    @Override
    public void init() {
        telemetry.log().setCapacity(6);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        tl = motor(TL, DRIVE_RPM, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        tr = motor(TR, DRIVE_RPM, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        bl = motor(BL, DRIVE_RPM, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        br = motor(BR, DRIVE_RPM, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        intk = motor(INTK, INTAKE_RPM, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        actR = motor(ACT_R, POLLEN_RPM, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        push = motor(PUSH, PUSH_RPM, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        actL = motor(ACT_L, NECTAR_RPM, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        stp = servo(STP, Servo.Direction.FORWARD, false);
        if (stp != null) setServoDeg(stp, STOPPER_CLOSED_DEG, STOPPER_RANGE_DEG);

        flr = servo(FLR, Servo.Direction.FORWARD, true);
        fll = servo(FLL, Servo.Direction.REVERSE, true);

        mvR = hardwareMap.tryGet(CRServo.class, MV_R);
        if (mvR != null) {
            turretAligner.init(hardwareMap);  // Configures the shared mvR servo as REVERSE.
            mvR.setPower(0);
            reportLine(MV_R, "OK");
        } else {
            reportMissing(MV_R);
        }

        reportStub(PPC, hardwareMap.tryGet(HardwareDevice.class, PPC) != null,
                "odometry ignored for now");
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, VIZ1);
        if (webcam == null) {
            reportMissing(VIZ1);
        } else {
            try {
                tags = new AprilTagProcessor.Builder().build();
                portal = new VisionPortal.Builder()
                        .setCamera(webcam)
                        .setCameraResolution(new Size(640, 480))
                        .addProcessor(tags)
                        .build();
                reportLine(VIZ1, "vision initialized");
            } catch (RuntimeException e) {
                reportLine(VIZ1, "vision unavailable: " + e.getMessage());
            }
        }
        report.add("encR: STUB (read through a motor port, turret encoder ignored for now)");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b || gamepad2.b) alliance = Alliance.RED;
        if (gamepad1.x || gamepad2.x) alliance = Alliance.BLUE;

        telemetry.addData("FGL26 TeleOp Prelim", VERSION);
        telemetry.addData("ALLIANCE", "%s   (B = red, X = blue)", alliance);
        telemetry.addData("Hardware", missingCount == 0
                ? "everything found" : missingCount + " missing (skipped)");
        if (portal != null) telemetry.addData("Camera", portal.getCameraState());
        selectVisionColor();
        telemetry.addData("Vision color", selectedColor());
        for (String line : report) telemetry.addLine(line);

        pA = gamepad1.a;
        pB = gamepad1.b;
        pBack = gamepad1.back;
        pUp = gamepad1.dpad_up;
        pDown = gamepad1.dpad_down;
    }

    @Override
    public void start() {
        turretAligner.resetTimer();
        // Holding an alliance-selection button across START won't adjust the RPM.
        rememberSpeedButtons();
        driveTimer.reset();
        telemetry.log().clear();
        telemetry.log().add("alliance " + alliance + ", " +
                (missingCount == 0 ? "all hardware found"
                        : missingCount + " devices missing, see init report"));
    }

    @Override
    public void loop() {
        safe("drive", this::drive);
        safe("intake", this::intake);
        safe("flower", this::flower);
        safe("secondary", this::secondaryController);
        safe("pollen", this::pollen);
        safe("nectar", this::nectar);

        safe("turret", this::turret);
        safe("positions", this::positions);

        pA = gamepad1.a;
        pB = gamepad1.b;
        pBack = gamepad1.back;
        pUp = gamepad1.dpad_up;
        pDown = gamepad1.dpad_down;

        showTelemetry();
    }

    @Override
    public void stop() {
        for (Motor m : new Motor[]{tl, tr, bl, br, intk, push, actR, actL}) {
            if (m != null) m.set(0);
        }
        if (mvR != null) mvR.setPower(0);
        if (portal != null) portal.close();
    }

    Alliance selectedColor() {
        return visionColor == null ? alliance : visionColor;
    }

    void selectVisionColor() {
        if (gamepad2.back && !gamepad2.start) visionColor = Alliance.BLUE;
        if (gamepad2.start && !gamepad2.back) visionColor = Alliance.RED;
    }

    void secondaryController() {
        selectVisionColor();
        int change = 0;
        if (gamepad2.x && !prevX2) change -= 100;
        if (gamepad2.b && !prevB2) change += 100;
        if (gamepad2.y && !prevY2) change += 50;
        if (gamepad2.a && !prevA2) change -= 50;

        flywheelRpm = Range.clip(flywheelRpm + change, 0.0,
                Math.min(POLLEN_RPM, NECTAR_RPM));
        rememberSpeedButtons();
    }

    void rememberSpeedButtons() {
        prevX2 = gamepad2.x;
        prevB2 = gamepad2.b;
        prevY2 = gamepad2.y;
        prevA2 = gamepad2.a;
    }

    void drive() {
        double wantFwd = -shape(gamepad1.left_stick_y) * DRIVE_SPEED;
        double wantStrafe = shape(gamepad1.left_stick_x) * DRIVE_SPEED;
        double wantTurn = shape(gamepad1.right_stick_x) * TURN_SPEED;

        double dt = Math.min(driveTimer.seconds(), 0.1);
        driveTimer.reset();

        curStrafe = ramp(curStrafe, wantStrafe, dt);
        curFwd = ramp(curFwd, wantFwd, dt);
        curTurn = ramp(curTurn, wantTurn, dt);

        double pTL = curFwd + curStrafe + curTurn;
        double pTR = curFwd - curStrafe - curTurn;
        double pBL = curFwd - curStrafe + curTurn;
        double pBR = curFwd + curStrafe - curTurn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(pTL), Math.abs(pTR)),
                Math.max(Math.abs(pBL), Math.abs(pBR))));

        power(tl, pTL / max);
        power(tr, pTR / max);
        power(bl, pBL / max);
        power(br, pBR / max);
    }

    double ramp(double current, double target, double dt) {
        if (!SMOOTH_DRIVE) return target;
        boolean speedingUp = current == 0
                || (Math.signum(target) == Math.signum(current)
                && Math.abs(target) > Math.abs(current));
        double maxStep = (speedingUp ? RAMP_UP_PER_SEC : RAMP_DOWN_PER_SEC) * dt;
        return current + Range.clip(target - current, -maxStep, maxStep);
    }

    void intake() {
        if (gamepad1.a && !pA) {
            intakeToggle = !intakeToggle;
            if (intakeToggle) outtakeToggle = false;
        }
        if (gamepad1.b && !pB) {
            outtakeToggle = !outtakeToggle;
            if (outtakeToggle) intakeToggle = false;
        }

        double p;
        if (gamepad1.right_bumper) p = INTAKE_POWER;
        else if (gamepad1.left_bumper) p = OUTTAKE_POWER;
        else if (intakeToggle) p = INTAKE_POWER;
        else if (outtakeToggle) p = OUTTAKE_POWER;
        else p = 0;

        power(intk, p);
    }

    void flower() {
        if (gamepad1.dpad_up && !pUp) {
            setFlower(FLOWER_ENGAGED_DEG);
            flowerState = "ENGAGED (" + (int) FLOWER_ENGAGED_DEG + ")";
        }
        if (gamepad1.dpad_down && !pDown) {
            setFlower(FLOWER_DISENGAGED_DEG);
            flowerState = "disengaged (" + (int) FLOWER_DISENGAGED_DEG + ")";
        }
    }

    void setFlower(double deg) {
        if (flr != null) setServoDeg(flr, deg, FLOWER_RANGE_DEG);
        if (fll != null) setServoDeg(fll, deg, FLOWER_RANGE_DEG);
    }

    void pollen() {
        boolean held = gamepad1.right_trigger > 0.5;
        if (held && !pollenHeld) pollenTimer.reset();
        pollenHeld = held;

        if (!held || flywheelRpm == 0) {
            pollenTimer.reset();
            power(actR, 0);
            power(push, 0);
            return;
        }

        boolean revved = pollenTimer.seconds() >= POLLEN_REV_TIME;
        power(actR, flywheelRpm / POLLEN_RPM);
        power(push, revved ? PUSH_POWER : 0);
    }

    void nectar() {
        boolean held = gamepad1.left_trigger > 0.5;

        if (!held || flywheelRpm == 0) {
            nectarPhase = NectarPhase.IDLE;
            power(actL, 0);
            stopper(false);
            return;
        }

        power(actL, flywheelRpm / NECTAR_RPM);

        switch (nectarPhase) {
            case IDLE:
                nectarPhase = NectarPhase.REVVING;
                nectarTimer.reset();
                nectarShots = 0;
                stopper(false);
                break;

            case REVVING:
                if (nectarTimer.seconds() >= NECTAR_REV_TIME) openForShot();
                break;

            case OPEN:
                if (nectarTimer.seconds() >= NECTAR_OPEN_TIME) {
                    stopper(false);
                    nectarPhase = NectarPhase.RECOVER;
                    nectarTimer.reset();
                }
                break;

            case RECOVER:
                if (nectarTimer.seconds() >= NECTAR_RECOVER_TIME) openForShot();
                break;
        }
    }

    void openForShot() {
        stopper(true);
        nectarPhase = NectarPhase.OPEN;
        nectarTimer.reset();
        nectarShots++;
    }

    void stopper(boolean open) {
        if (stp != null) {
            setServoDeg(stp, open ? STOPPER_OPEN_DEG : STOPPER_CLOSED_DEG,
                    STOPPER_RANGE_DEG);
        }
    }

    void turret() {
        if (gamepad1.back && !pBack) {
            alignOnShoot = !alignOnShoot;
            telemetry.log().add("autoalign on shoot " + (alignOnShoot ? "ON" : "OFF"));
        }

        double manual = 0;
        if (gamepad1.dpad_right && !gamepad1.dpad_left) manual = TURRET_MANUAL_POWER;
        if (gamepad1.dpad_left && !gamepad1.dpad_right) manual = -TURRET_MANUAL_POWER;

        if (manual != 0) {
            if (!trackedCluster.isEmpty()) stopAlignment("manual");
            if (mvR != null) mvR.setPower(INVERT_TURRET ? manual : -manual);
            turretStatus = mvR == null ? "manual (no mvR)"
                    : (manual > 0 ? "manual RIGHT" : "manual LEFT");
            return;
        }

        if (pollenHeld && alignOnShoot) {
            autoalign();
            return;
        }

        stopAlignment("idle");
    }

    void autoalign() {
        String color = selectedColor().name();
        String scoring = color + " SCORING";
        String audience = color + " AUDIENCE";
        if (mvR == null || portal == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            stopAlignment(mvR == null ? "turret servo missing" : "camera unavailable");
            return;
        }

        AprilTagClusterDetection best = null;
        for (AprilTagDetection detection : tags.getDetections()) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null) continue;
            String name = cluster.metadata.name;
            if (!scoring.equalsIgnoreCase(name) && !audience.equalsIgnoreCase(name)) continue;
            if (cluster.ftcPose == null || !Double.isFinite(cluster.ftcPose.bearing)) continue;

            long frame = cluster.frameAcquisitionNanoTime;
            if (System.nanoTime() - frame > MAX_FRAME_AGE_NS) continue;
            if (name.equalsIgnoreCase(trackedCluster)) {
                best = cluster;
                break;
            }
            if (best == null || Math.abs(cluster.ftcPose.bearing) < Math.abs(best.ftcPose.bearing)) {
                best = cluster;
            }
        }
        if (best == null) {
            stopAlignment("no current pose for " + color);
            return;
        }
        if (!trackedCluster.isEmpty() && !best.metadata.name.equalsIgnoreCase(trackedCluster)) {
            stopAlignment("switching to " + best.metadata.name);
            return;
        }
        trackedCluster = best.metadata.name;
        turretAligner.update(best);
        turretStatus = String.format("%s | bearing %.1f deg | power %.2f",
                trackedCluster, best.ftcPose.bearing, turretAligner.getPower());
    }

    void stopAlignment(String status) {
        if (mvR != null) turretAligner.update(null);
        trackedCluster = "";
        turretStatus = status;
    }

    void positions() {
        if (gamepad1.x) positionStatus = "position 1 (" + alliance + ") STUB";
        else if (gamepad1.y) positionStatus = "position 2 (" + alliance + ") STUB";
        else positionStatus = "-";
    }

    void showTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Vision color", selectedColor());
        telemetry.addData("Intake", gamepad1.right_bumper ? "IN (held)"
                : gamepad1.left_bumper ? "OUT (held)"
                : intakeToggle ? "IN" : outtakeToggle ? "OUT" : "off");
        telemetry.addData("Flower", flowerState);
        telemetry.addData("Pollen", (!pollenHeld || flywheelRpm == 0) ? "off"
                : (pollenTimer.seconds() < POLLEN_REV_TIME
                ? String.format("revving %.1fs", POLLEN_REV_TIME - pollenTimer.seconds())
                : "FIRING"));
        telemetry.addData("Nectar", nectarPhase == NectarPhase.IDLE ? "off"
                : nectarPhase + "  shots " + nectarShots);

        telemetry.addData("Flywheel setting", "%.0f RPM", flywheelRpm);
        telemetry.addData("Pollen actual", rpmText(actR));
        telemetry.addData("Nectar actual", rpmText(actL));

        telemetry.addData("Turret", "%s  | align on shoot: %s",
                turretStatus, alignOnShoot ? "ON" : "OFF");
        if (!positionStatus.equals("-")) telemetry.addData("Position", positionStatus);
    }

    Motor motor(String name, double rpm, DcMotorSimple.Direction dir,
                DcMotor.ZeroPowerBehavior zpb) {
        DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, name);
        if (m == null) {
            reportMissing(name);
            return null;
        }
        m.setDirection(dir);
        m.setZeroPowerBehavior(zpb);
        Motor w = new Motor(name, m, rpm);
        reportLine(name, USE_VELOCITY
                ? String.format("OK (velocity, %.0f rpm)", rpm) : "OK (power)");
        return w;
    }

    ServoImplEx servo(String name, Servo.Direction dir, boolean startLimp) {
        ServoImplEx s = hardwareMap.tryGet(ServoImplEx.class, name);
        if (s == null) {
            reportMissing(name);
            return null;
        }
        s.setDirection(dir);
        s.setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (startLimp) s.setPwmDisable();
        reportLine(name, "OK");
        return s;
    }

    void setServoDeg(ServoImplEx s, double deg, double rangeDeg) {
        s.setPosition(Range.clip(deg / rangeDeg, 0, 1));
        s.setPwmEnable();
    }

    void power(Motor m, double p) {
        if (m != null) m.set(p);
    }

    String rpmText(Motor m) {
        if (m == null) return "motor missing";
        if (!m.velocity) return "unavailable";
        return String.format("%.0f RPM", m.rpmNow());
    }

    class Motor {
        final String name;
        final DcMotorEx m;
        final double rpm;
        final double ticksPerRev, maxTicksPerSec;
        boolean velocity;
        boolean verified = false;
        double target = 0;
        final ElapsedTime checkTimer = new ElapsedTime();
        boolean checking = false;

        Motor(String name, DcMotorEx m, double rpm) {
            this.name = name;
            this.m = m;
            this.rpm = rpm;

            ticksPerRev = GOBILDA_ENCODER_CPR * (GOBILDA_BARE_RPM / rpm);
            maxTicksPerSec = rpm / 60.0 * ticksPerRev;
            velocity = USE_VELOCITY;

            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(velocity ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            set(0);
        }

        void set(double p) {
            target = Range.clip(p, -1, 1);
            if (velocity) {
                checkEncoder();
                if (velocity) {
                    m.setVelocity(target * maxTicksPerSec);
                    return;
                }
            }
            m.setPower(target);
        }

        double rpmNow() {
            return m.getVelocity() / ticksPerRev * 60.0;
        }

        void checkEncoder() {
            if (verified) return;

            if (Math.abs(target) < ENCODER_CHECK_SPEED) {
                checking = false;
                return;
            }
            if (m.getVelocity() != 0) {
                verified = true;
                return;
            }
            if (!checking) {
                checking = true;
                checkTimer.reset();
                return;
            }
            if (checkTimer.seconds() > ENCODER_CHECK_TIME) {
                velocity = false;
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.log().add(name + ": encoder reads 0, switched to plain power");
            }
        }
    }

    double shape(double v) {
        if (Math.abs(v) < DEADZONE) return 0;
        v = Math.signum(v) * (Math.abs(v) - DEADZONE) / (1.0 - DEADZONE);
        if (SQUARE_INPUTS) v = v * Math.abs(v);
        return v;
    }

    void safe(String part, Runnable r) {
        if (crashed.contains(part)) return;
        try {
            r.run();
        } catch (Exception e) {
            crashed.add(part);
            telemetry.log().add(part + " DISABLED: " + e.getClass().getSimpleName()
                    + " " + e.getMessage());
            try {
                stopPart(part);
            } catch (Exception ignored) {
                // The original failure is already in the telemetry log.
            }
        }
    }

    void stopPart(String part) {
        switch (part) {
            case "drive":
                power(tl, 0);
                power(tr, 0);
                power(bl, 0);
                power(br, 0);
                curStrafe = 0;
                curFwd = 0;
                curTurn = 0;
                break;
            case "intake":
                power(intk, 0);
                break;
            case "pollen":
                power(actR, 0);
                power(push, 0);
                break;
            case "nectar":
                power(actL, 0);
                break;
            case "turret":
                if (mvR != null) mvR.setPower(0);
                break;
        }
    }

    void reportLine(String name, String status) {
        report.add(name + ": " + status);
    }

    void reportMissing(String name) {
        missingCount++;
        report.add(name + ": MISSING (skipped)");
    }

    void reportStub(String name, boolean found, String why) {
        report.add(name + ": STUB, " + (found ? "in config" : "not in config")
                + ", " + why);
    }
}
