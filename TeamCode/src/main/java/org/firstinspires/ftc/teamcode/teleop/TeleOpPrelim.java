package org.firstinspires.ftc.teamcode.teleop;

import android.text.TextUtils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.teamcode.utilities.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utilities.TurretAligner;

import java.util.ArrayList;
import java.util.List;

/*
 * TeleOpPrelim: everything we have so far in one program. scuffed on purpose.
 * every piece of hardware is loaded with tryGet, so anything not built / not in the config
 * just gets skipped (shows up as MISSING in telemetry) instead of crashing the opmode.
 *
 * ===== PRIMARY CONTROLLER (gamepad1, sketch v4.11.1) =====
 *  sticks        drive
 *  LT (hold)     shoot pollen  (actL + ramp, + autoalign pollen turret unless BACK toggled)
 *  RT (hold)     shoot nectar  (actR + ramp, + autoalign nectar turret unless BACK toggled)
 *  LB (hold)     autoalign pollen turret only
 *  RB (hold)     autoalign nectar turret only
 *  BACK          toggle: shoot buttons stop autoaligning (only run motors)
 *  dpad up/down  flower releaser
 *  A             toggle intake
 *  B             toggle outtake
 *  X (hold)      drive to position 1
 *  Y (hold)      drive to position 2
 *
 *  during INIT (before start): B = red alliance, X = blue alliance (either controller)
 *
 * ===== SECONDARY CONTROLLER (gamepad2) =====
 *  TODO: not designed yet, see the gamepad2 block in loop()
 *
 * ===== ROBOT CONFIG NAMES =====
 *  these must match the driver station config. old code used different names
 *  (intake, pollen_launch, main_servo, mirror_servo) so the config needs renaming.
 *  exception: the pollen turret still uses "turret_servo" / "turret_encoder" because
 *  TurretAligner has them hardcoded (see TODO TURRET-NAMES).
 */
@TeleOp(name = "TeleOp Prelim", group = "Main")
public class TeleOpPrelim extends OpMode {

    // ---------------- config names ----------------
    static final String TL = "tl", TR = "tr", BL = "bl", BR = "br";
    static final String INTK = "intk";
    static final String PUSH = "push";
    static final String ACT_L = "actL"; // pollen shooter
    static final String ACT_R = "actR"; // nectar shooter
    static final String FLR = "flr", FLL = "fll";
    static final String PPC = "ppc";
    static final String VIZ1 = "viz1"; // pollen camera (on left turret)
    static final String VIZ2 = "viz2"; // nectar camera (on right turret)

    // ---------------- tuning / constants ----------------
    static final double DEADZONE = 0.05;
    static final boolean SQUARE_INPUTS = true;

    static final double INTAKE_POWER = 1.0;
    static final double OUTTAKE_POWER = -1.0;

    // TODO SHOOTER-VELOCITY: open loop power for now (0.60 is what MotorDriveTest0 used for pollen).
    //  needs: RUN_USING_ENCODER + setVelocity(), shooter motor ticks/rev, then a distance -> velocity table
    //  (distance comes from the camera's ftcPose.range). shot distance drifts with battery until then.
    static final double POLLEN_SHOOT_POWER = 0.60;
    static final double NECTAR_SHOOT_POWER = 0.60; // TODO: placeholder, nectar shooter isn't built

    // TODO RAMP: whole ramp/push setup is getting redesigned.
    //  right now: one "push" motor, starts RAMP_DELAY sec after a shoot button is pressed.
    //  needs: final ramp design, does one motor feed both canals or not, direction per canal,
    //  and maybe "wait until shooter is at speed" instead of a fixed delay (needs SHOOTER-VELOCITY)
    static final double RAMP_DELAY = 0.30;
    static final double PUSH_POWER = 1.0;

    // flower servos, 270 deg servos (same as MotorDriveTest0)
    static final double FLOWER_RANGE_DEG = 270;
    static final double FLOWER_START_DEG = 135;
    // TODO FLOWER-PRESETS: dpad up/down step the servos (proven code from MotorDriveTest0).
    //  needs: measured open + closed angles (use flower_eater_servo_controller to find them),
    //  then switch up = open preset, down = closed preset
    static final double FLOWER_STEP_DEG = 15;

    // camera exposure. TODO EXPOSURE: tune with ConceptAprilTagOptimizeExposure, these are guesses
    static final int EXPOSURE_MS = 6;
    static final int GAIN = 250;

    // drive to position (X / Y)
    // TODO POSITIONS: needs pinpoint configured (PINPOINT-CONFIG), a known start pose, and measured
    //  positions per alliance. {x inches, y inches, heading degrees}. null = not measured = button does nothing
    static final double[] RED_POS_1 = null;
    static final double[] RED_POS_2 = null;
    static final double[] BLUE_POS_1 = null;
    static final double[] BLUE_POS_2 = null;
    // TODO POSITION-GAINS: untested guesses. if it drives AWAY from the target, flip signs in driveToPose()
    static final double POS_KP = 0.04;       // power per inch
    static final double HEADING_KP = 0.015;  // power per degree
    static final double POS_MAX_POWER = 0.5;
    static final double POS_TOLERANCE_IN = 1.0;
    static final double HEADING_TOLERANCE_DEG = 2.0;

    // ---------------- hardware ----------------
    DcMotor tl, tr, bl, br;
    DcMotor intk, push;
    DcMotorEx actL, actR;
    ServoImplEx fll, flr;
    GoBildaPinpointDriver ppc;
    AprilTagWebcam pollenCam, nectarCam;
    TurretAligner pollenTurret;
    // TODO NECTAR-TURRET: needs the right turret built (mvR + encR), viz2 mounted, and
    //  TurretAligner changed to take servo/encoder names (TURRET-NAMES). then:
    //  nectarTurret = new TurretAligner(...) and use it in alignNectar()
    TurretAligner nectarTurret = null;

    List<String> missing = new ArrayList<>();

    // ---------------- state ----------------
    enum Alliance { RED, BLUE }
    Alliance alliance = Alliance.RED;

    boolean alignOnShoot = true; // BACK flips this
    boolean intakeOn = false, outtakeOn = false;
    double flowerTarget = FLOWER_START_DEG;

    boolean pollenShooting = false, nectarShooting = false;
    ElapsedTime pollenShootTimer = new ElapsedTime();
    ElapsedTime nectarShootTimer = new ElapsedTime();

    // previous button states (edge detection)
    boolean pBack, pA, pB, pUp, pDown;

    String pollenAlignStatus = "off", nectarAlignStatus = "off";
    String posStatus = "manual";

    // =========================================================
    @Override
    public void init() {
        // ----- drive (directions from MotorDriveTest0) -----
        tl = motor(TL, DcMotorSimple.Direction.REVERSE);
        tr = motor(TR, DcMotorSimple.Direction.FORWARD);
        bl = motor(BL, DcMotorSimple.Direction.REVERSE);
        br = motor(BR, DcMotorSimple.Direction.FORWARD);

        // ----- intake -----
        intk = motor(INTK, DcMotorSimple.Direction.FORWARD);

        // ----- push / ramp (not built) -----
        push = motor(PUSH, DcMotorSimple.Direction.FORWARD);

        // ----- shooters -----
        actL = shooter(ACT_L, DcMotorSimple.Direction.REVERSE); // REVERSE = what pollen_launch used
        actR = shooter(ACT_R, DcMotorSimple.Direction.FORWARD); // TODO: direction unknown, not built

        // ----- flower servos (mirrored pair, same as MotorDriveTest0) -----
        // TODO FLOWER-MIRROR: the new sketch swapped ports (flr D0, fll D1). check they move together,
        //  if they fight each other swap which one is REVERSE
        fll = flowerServo(FLL, Servo.Direction.FORWARD);
        flr = flowerServo(FLR, Servo.Direction.REVERSE);

        // ----- pinpoint -----
        ppc = hardwareMap.tryGet(GoBildaPinpointDriver.class, PPC);
        if (ppc != null) {
            configurePinpoint();
        } else {
            missing.add(PPC);
        }

        // ----- cameras -----
        initCameras();

        // ----- pollen turret -----
        // TODO TURRET-NAMES: TurretAligner hardcodes "turret_servo" + "turret_encoder". when rosy's
        //  working version gets merged, make init take (servoName, encoderName) so we can use
        //  mvL/encL here and mvR/encR for nectar. encoder name = the MOTOR port its cable is plugged into.
        // TODO TURRET-LIMITS: min_ticks / max_ticks in TurretAligner are still 0, which blocks motion
        //  both ways. needs the real turret range in ticks (should be fixed in rosy's version)
        pollenTurret = new TurretAligner();
        try {
            pollenTurret.init(hardwareMap);
        } catch (Exception e) {
            pollenTurret = null;
            missing.add("pollen turret (turret_servo / turret_encoder)");
        }

        telemetry.addLine("init done");
    }

    @Override
    public void init_loop() {
        // ----- alliance select (red/blue buttons on either controller) -----
        if (gamepad1.b || gamepad2.b) alliance = Alliance.RED;
        if (gamepad1.x || gamepad2.x) alliance = Alliance.BLUE;

        // TODO INIT-MENU: other init-phase controls go here (start position select, etc)

        // ----- exposure (cameras need to be streaming first, so keep trying) -----
        if (pollenCam != null && !pollenCam.isExposureApplied()) pollenCam.setManualExposure(EXPOSURE_MS, GAIN);
        if (nectarCam != null && !nectarCam.isExposureApplied()) nectarCam.setManualExposure(EXPOSURE_MS, GAIN);

        telemetry.addData("ALLIANCE", "%s   (B = red, X = blue)", alliance);
        telemetry.addData("Pollen cam exposure", pollenCam == null ? "no cam" : (pollenCam.isExposureApplied() ? "set" : "waiting"));
        telemetry.addData("Nectar cam exposure", nectarCam == null ? "no cam" : (nectarCam.isExposureApplied() ? "set" : "waiting"));
        telemetry.addData("MISSING", missing.isEmpty() ? "nothing" : TextUtils.join(", ", missing));

        // so buttons held while pressing start don't count as a fresh press
        pBack = gamepad1.back; pA = gamepad1.a; pB = gamepad1.b; pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
    }

    @Override
    public void start() {
        if (pollenTurret != null) pollenTurret.resetTimer();
        if (nectarTurret != null) nectarTurret.resetTimer();
    }

    // =========================================================
    @Override
    public void loop() {
        if (ppc != null) ppc.update();
        if (pollenCam != null) pollenCam.update();
        if (nectarCam != null) nectarCam.update();

        // ----- buttons -----
        boolean back = gamepad1.back;
        boolean a = gamepad1.a, b = gamepad1.b;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean lt = gamepad1.left_trigger > 0.5;
        boolean rt = gamepad1.right_trigger > 0.5;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        // ----- BACK: toggle autoalign on shoot -----
        if (back && !pBack) alignOnShoot = !alignOnShoot;

        // ----- drive -----
        double[] target = null;
        if (gamepad1.x) target = alliance == Alliance.RED ? RED_POS_1 : BLUE_POS_1;
        else if (gamepad1.y) target = alliance == Alliance.RED ? RED_POS_2 : BLUE_POS_2;

        if ((gamepad1.x || gamepad1.y) && target != null && ppc != null) {
            driveToPose(target);
        } else {
            posStatus = (gamepad1.x || gamepad1.y) ? "position not set / no pinpoint" : "manual";
            drive(shape(gamepad1.left_stick_x), shape(gamepad1.left_stick_y), shape(gamepad1.right_stick_x));
        }

        // ----- intake / outtake toggles -----
        if (a && !pA) { intakeOn = !intakeOn; if (intakeOn) outtakeOn = false; }
        if (b && !pB) { outtakeOn = !outtakeOn; if (outtakeOn) intakeOn = false; }
        double intakePower = intakeOn ? INTAKE_POWER : (outtakeOn ? OUTTAKE_POWER : 0);
        setPower(intk, intakePower);

        // ----- flower releaser -----
        double flowerStep = 0;
        if (up && !pUp && !down) flowerStep += FLOWER_STEP_DEG;
        if (down && !pDown && !up) flowerStep -= FLOWER_STEP_DEG;
        if (flowerStep != 0) {
            flowerTarget = Range.clip(flowerTarget + flowerStep, 0, FLOWER_RANGE_DEG);
            setFlower(flowerTarget);
        }

        // ----- pollen: LT shoot, LB align -----
        shootPollen(lt);
        alignPollen(lb || (lt && alignOnShoot));

        // ----- nectar: RT shoot, RB align -----
        shootNectar(rt);
        alignNectar(rb || (rt && alignOnShoot));

        // ----- ramp / push -----
        boolean pollenFeed = pollenShooting && pollenShootTimer.seconds() > RAMP_DELAY;
        boolean nectarFeed = nectarShooting && nectarShootTimer.seconds() > RAMP_DELAY;
        setPower(push, (pollenFeed || nectarFeed) ? PUSH_POWER : 0);

        // ----- gamepad2 (secondary) -----
        // TODO GAMEPAD2: not designed yet. planned:
        //  - manual turret override (stick) for when the camera loses the tag. needs TURRET-NAMES + encoder control
        //  - shooter power/velocity trim. needs SHOOTER-VELOCITY
        //  - reset pinpoint pose. needs PINPOINT-CONFIG
        //  - flower fine adjust
        //  - alliance swap mid-match (probably not needed)

        pBack = back; pA = a; pB = b; pUp = up; pDown = down;

        doTelemetry();
    }

    @Override
    public void stop() {
        for (DcMotor m : new DcMotor[]{tl, tr, bl, br, intk, push, actL, actR}) setPower(m, 0);
        if (pollenCam != null) pollenCam.stop();
        if (nectarCam != null) nectarCam.stop();
    }

    // =========================================================
    // actions
    // =========================================================

    // hold to shoot: shooter on now, ramp starts after RAMP_DELAY (handled in loop)
    void shootPollen(boolean held) {
        if (held && !pollenShooting) pollenShootTimer.reset();
        pollenShooting = held;
        setPower(actL, held ? POLLEN_SHOOT_POWER : 0);
    }

    void shootNectar(boolean held) {
        if (held && !nectarShooting) nectarShootTimer.reset();
        nectarShooting = held;
        setPower(actR, held ? NECTAR_SHOOT_POWER : 0);
    }

    void alignPollen(boolean active) {
        if (pollenTurret == null) { pollenAlignStatus = "no turret"; return; }
        if (!active || pollenCam == null) {
            pollenTurret.update(null); // null = stop the servo
            pollenAlignStatus = active ? "no camera" : "off";
            return;
        }
        AprilTagClusterDetection t = pollenCam.getBestClusterStartingWith(alliance.name());
        pollenTurret.update(t);
        pollenAlignStatus = t == null ? "no tag" : String.format("%s  bearing %.1f", t.metadata.name, t.ftcPose.bearing);
    }

    void alignNectar(boolean active) {
        // TODO NECTAR-TURRET: same as alignPollen with nectarTurret + nectarCam once it exists
        if (nectarTurret == null) { nectarAlignStatus = active ? "not built yet" : "off"; return; }
        if (!active || nectarCam == null) {
            nectarTurret.update(null);
            nectarAlignStatus = active ? "no camera" : "off";
            return;
        }
        AprilTagClusterDetection t = nectarCam.getBestClusterStartingWith(alliance.name());
        nectarTurret.update(t);
        nectarAlignStatus = t == null ? "no tag" : String.format("%s  bearing %.1f", t.metadata.name, t.ftcPose.bearing);
    }

    // drive math straight from MotorDriveTest0 (known working)
    // strafe = left stick x, fwd = left stick y (raw, up is negative), turn = right stick x
    void drive(double strafe, double fwd, double turn) {
        double pTL = strafe + fwd + turn;
        double pTR = strafe - fwd - turn;
        double pBL = strafe - fwd + turn;
        double pBR = strafe + fwd - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(pTL), Math.abs(pTR)),
                Math.max(Math.abs(pBL), Math.abs(pBR))));

        setPower(tl, pTL / max);
        setPower(tr, pTR / max);
        setPower(bl, pBL / max);
        setPower(br, pBR / max);
    }

    // hold X/Y: P controller to a field pose using pinpoint. untested.
    void driveToPose(double[] t) {
        Pose2D p = ppc.getPosition();
        double x = p.getX(DistanceUnit.INCH);
        double y = p.getY(DistanceUnit.INCH);
        double h = p.getHeading(AngleUnit.DEGREES);

        double dx = t[0] - x;
        double dy = t[1] - y;
        double dh = AngleUnit.normalizeDegrees(t[2] - h);

        if (Math.hypot(dx, dy) < POS_TOLERANCE_IN && Math.abs(dh) < HEADING_TOLERANCE_DEG) {
            drive(0, 0, 0);
            posStatus = "at position";
            return;
        }

        // field error -> robot frame (pinpoint: x forward, y left, heading ccw+)
        double hr = Math.toRadians(h);
        double fwd = dx * Math.cos(hr) + dy * Math.sin(hr);
        double left = -dx * Math.sin(hr) + dy * Math.cos(hr);

        double f = Range.clip(fwd * POS_KP, -POS_MAX_POWER, POS_MAX_POWER);
        double l = Range.clip(left * POS_KP, -POS_MAX_POWER, POS_MAX_POWER);
        double turn = Range.clip(dh * HEADING_KP, -POS_MAX_POWER, POS_MAX_POWER);

        // convert to the same "stick" convention drive() uses:
        // stick y up = negative = forward, stick x right = strafe right, right stick x right = turn cw
        drive(-l, -f, -turn);
        posStatus = String.format("driving  err %.1f in, %.1f deg", Math.hypot(dx, dy), dh);
    }

    // =========================================================
    // setup helpers
    // =========================================================

    DcMotor motor(String name, DcMotorSimple.Direction dir) {
        DcMotor m = hardwareMap.tryGet(DcMotor.class, name);
        if (m == null) { missing.add(name); return null; }
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setPower(0);
        return m;
    }

    DcMotorEx shooter(String name, DcMotorSimple.Direction dir) {
        DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, name);
        if (m == null) { missing.add(name); return null; }
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO SHOOTER-VELOCITY
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // flywheel coasts down instead of slamming
        m.setPower(0);
        return m;
    }

    ServoImplEx flowerServo(String name, Servo.Direction dir) {
        ServoImplEx s = hardwareMap.tryGet(ServoImplEx.class, name);
        if (s == null) { missing.add(name); return null; }
        s.setDirection(dir);
        s.setPwmRange(new PwmControl.PwmRange(500, 2500));
        s.setPwmDisable(); // limp until the first dpad press, same as before
        return s;
    }

    void configurePinpoint() {
        // TODO PINPOINT-CONFIG: all of this is placeholder.
        //  needs: dw1/dw2 offsets from the tracking center in mm (x pod = sideways offset, left is +,
        //  y pod = forward offset, forward is +), pod type, and each pod's direction
        //  (x should go up pushing forward, y should go up pushing left). test with SensorGoBildaPinpoint
        ppc.setOffsets(0, 0, DistanceUnit.MM);
        ppc.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        ppc.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // TODO START-POSE: this sets wherever the robot is at init to (0,0,0).
        //  POSITIONS only mean something if we always start in the same spot, or auto hands
        //  its final pose to teleop (blackboard, see ConceptBlackboard sample)
        ppc.resetPosAndIMU();
    }

    void initCameras() {
        boolean has1 = hardwareMap.tryGet(WebcamName.class, VIZ1) != null;
        boolean has2 = hardwareMap.tryGet(WebcamName.class, VIZ2) != null;
        if (!has1) missing.add(VIZ1);
        if (!has2) missing.add(VIZ2);

        // two cameras need a split live view
        int view1 = -1, view2 = -1;
        if (has1 && has2) {
            int[] ids = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);
            view1 = ids[0];
            view2 = ids[1];
        }
        // TODO TWO-CAMS: running 2 webcams on one control hub is untested, check loop time / fps

        if (has1) {
            pollenCam = new AprilTagWebcam();
            pollenCam.init(hardwareMap, telemetry, VIZ1, view1);
        }
        if (has2) {
            nectarCam = new AprilTagWebcam();
            nectarCam.init(hardwareMap, telemetry, VIZ2, view2);
        }
    }

    // =========================================================
    // small helpers
    // =========================================================

    void setPower(DcMotor m, double p) {
        if (m != null) m.setPower(p);
    }

    void setFlower(double deg) {
        for (ServoImplEx s : new ServoImplEx[]{fll, flr}) {
            if (s == null) continue;
            s.setPosition(deg / FLOWER_RANGE_DEG);
            s.setPwmEnable();
        }
    }

    double shape(double v) {
        if (Math.abs(v) < DEADZONE) return 0;
        v = Math.signum(v) * (Math.abs(v) - DEADZONE) / (1.0 - DEADZONE);
        if (SQUARE_INPUTS) v = v * Math.abs(v);
        return v;
    }

    void doTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Autoalign on shoot", alignOnShoot ? "ON (BACK to turn off)" : "OFF (shoot = motors only)");
        telemetry.addData("Intake", intakeOn ? "IN" : (outtakeOn ? "OUT" : "off"));
        telemetry.addData("Pollen", "%s  align: %s", pollenShooting ? "SHOOTING" : "-", pollenAlignStatus);
        telemetry.addData("Nectar", "%s  align: %s", nectarShooting ? "SHOOTING" : "-", nectarAlignStatus);
        if (pollenTurret != null) telemetry.addData("Pollen turret ticks", pollenTurret.getTurretTicks());
        telemetry.addData("Flower target", "%.0f deg", flowerTarget);
        telemetry.addData("Drive", posStatus);
        if (ppc != null) {
            Pose2D p = ppc.getPosition();
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                    p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), p.getHeading(AngleUnit.DEGREES));
        }
        if (pollenCam != null) telemetry.addData("Pollen cam sees", pollenCam.getVisibleClusterNames());
        if (nectarCam != null) telemetry.addData("Nectar cam sees", nectarCam.getVisibleClusterNames());
        if (!missing.isEmpty()) telemetry.addData("MISSING", TextUtils.join(", ", missing));
    }
}
