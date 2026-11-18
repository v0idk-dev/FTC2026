package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/*
 * ============================================================================================
 *  TeleOpPrelim  (program v1.7.2, diagram v4.12.5, config v4-12-4)
 * ============================================================================================
 *  the "one teleop" that will become the final. it's built so it runs on a half-built robot:
 *
 *   - every device is loaded with tryGet(). if it's not in the config (not built / not plugged
 *     in / typo), that part of the code just turns itself off instead of crashing.
 *   - during INIT the driver station shows a full hardware report (OK / MISSING / STUB).
 *   - after START the report goes away. problems show up in the telemetry LOG (the scrolling
 *     area at the bottom of the screen), so they don't eat the normal telemetry lines.
 *   - if a part throws an error while running (cable pulled, etc), that part gets disabled,
 *     a log message tells you which one, and everything else keeps running.
 *
 *  ---------------- PRIMARY CONTROLLER (gamepad1) ----------------
 *   left stick         drive (forward/back + strafe)
 *   right stick x      turn
 *   RT (hold)          SHOOT POLLEN: actR spins up, push (ramp) starts after POLLEN_REV_TIME,
 *                      turret autoaligns while held (unless BACK turned that off)
 *   LT (hold)          SHOOT NECTAR: actL spins up, then the stopper servo opens/closes to let
 *                      nectar through one at a time
 *   RB (hold)          intake
 *   LB (hold)          outtake
 *   A                  toggle intake
 *   B                  toggle outtake
 *                      (holding RB/LB always wins over the toggles)
 *   dpad up            flower releaser ENGAGE  (open, 155 deg)
 *   dpad down          flower releaser DISENGAGE (closed, 70 deg)
 *   dpad left/right    turret manual left/right (mvR)
 *   BACK               toggle autoalign-on-shoot (STUB, turret/camera ignored for now)
 *   X (hold)           drive to position 1 (STUB, odometry ignored for now)
 *   Y (hold)           drive to position 2 (STUB, odometry ignored for now)
 *   START, MODE        unused (MODE can't be read by code on logitech pads)
 *
 *   during INIT (before pressing start): B = RED alliance, X = BLUE alliance (either pad)
 *
 *  ---------------- SECONDARY CONTROLLER (gamepad2) ----------------
 *   STUB, ignored for now. see secondaryController().
 *
 *  ---------------- CONFIG (must match driver station config v4-12-4) ----------------
 *   motors   tl D0, tr D1, bl D2, br D3, intk E0, push E1, actR E2, actL E3
 *   servos   flr D0, fll D1, mvR D2 (CR servo), stp D3 (nectar stopper, **FUTURE DIAGRAM/CONFIG**)
 *   other    ppc (pinpoint, i2c), viz1 (webcam)
 *   encR is a through-bore encoder. it has no config name of its own, it's read through
 *   whichever MOTOR port its cable plugs into.
 * ============================================================================================
 */
@TeleOp(name = "TeleOp Prelim", group = "0Main")
public class TeleOpPrelim extends OpMode {

    // =========================================================================================
    //  CONFIG NAMES
    // =========================================================================================
    static final String TL = "tl", TR = "tr", BL = "bl", BR = "br";
    static final String INTK = "intk";
    static final String PUSH = "push";   // pollen ramp motor
    static final String ACT_R = "actR";  // POLLEN shooter (right side, has the turret)
    static final String ACT_L = "actL";  // NECTAR shooter (left side, no turret)
    static final String FLR = "flr";     // flower servo, main
    static final String FLL = "fll";     // flower servo, mirror of flr
    static final String MV_R = "mvR";    // pollen turret CR servo
    // nectar stopper servo. NOT on the diagram yet, "stp" on servo port D3 is a guess.
    // once it's added to the robot config, change this name to match.
    static final String STP = "stp";
    static final String PPC = "ppc";     // pinpoint (ignored for now)
    static final String VIZ1 = "viz1";   // pollen turret camera (ignored for now)

    // =========================================================================================
    //  TUNING  (everything you'd want to change is up here)
    // =========================================================================================

    // ---- drive ----
    // pt drove too fast, so everything gets scaled down. 1.0 = full speed.
    static final double DRIVE_SPEED = 0.70;  // forward/back + strafe
    static final double TURN_SPEED = 0.80;   // rotation
    // turning was reversed in pt, this flips it. if it's backwards again, set to false.
    static final boolean INVERT_TURN = true;
    static final double DEADZONE = 0.05;
    static final boolean SQUARE_INPUTS = true;  // finer control near center of the stick
    // (before 11/18 both speeds were 1.0, pt has no scaling)

    // ---- smooth drive (ramping) ----
    // instead of jumping straight to the stick value, drive power moves toward it at a max rate.
    // stops the jerk when you let go of the stick (and when you slam it).
    //   rate = how much power can change per second. drive power goes 0 to 1.
    //   e.g. RAMP_DOWN_PER_SEC = 4.0 -> from DRIVE_SPEED 0.6 to stopped in 0.6 / 4 = 0.15s
    //   lower number = smoother but floatier. higher = snappier. too low and the robot overshoots.
    // SMOOTH_DRIVE = false turns it off completely (instant, like pt).
    static final boolean SMOOTH_DRIVE = true;
    static final double RAMP_DOWN_PER_SEC = 2.5;  // slowing down / stopping / reversing
    static final double RAMP_UP_PER_SEC = 7.5;    // speeding up (higher so it still feels responsive)

    // ---- intake ----
    static final double INTAKE_POWER = 1.0;
    static final double OUTTAKE_POWER = -1.0;

    // ---- flower releaser ----
    static final double FLOWER_RANGE_DEG = 270;     // these servos are 270 deg servos
    static final double FLOWER_ENGAGED_DEG = 155;   // open / engaged
    static final double FLOWER_DISENGAGED_DEG = 70; // closed / disengaged

    // ---- pollen shooter (actR + push) ----
    // hold RT: actR runs at REV power, after REV_TIME it switches to SHOOT power and push turns on.
    // pt used 0.60 for the shooter and 0.67s delay. ~1s delay is what we want now.
    static final double POLLEN_REV_POWER = 0.60;
    static final double POLLEN_SHOOT_POWER = 0.60;
    static final double POLLEN_REV_TIME = 1.0;   // seconds before push starts
    static final double PUSH_POWER = 1.0;

    // ---- nectar shooter (actL + stopper) ----
    // PARTIAL INFO, not built yet. all numbers are placeholders until it's on the robot.
    // hold LT:
    //   1. actL at NECTAR_REV_POWER for NECTAR_REV_TIME, stopper closed
    //   2. actL switches to NECTAR_SHOOT_POWER, stopper opens for NECTAR_OPEN_TIME (one nectar goes)
    //   3. stopper closes, waits NECTAR_RECOVER_TIME so the flywheel gets speed back
    //   4. back to step 2 while LT is still held
    // release LT: shooter off, stopper closed.
    static final double NECTAR_REV_POWER = 0.60;
    static final double NECTAR_SHOOT_POWER = 0.60;
    static final double NECTAR_REV_TIME = 1.0;
    static final double STOPPER_RANGE_DEG = 270;     // TODO: check what servo the stopper actually is
    static final double STOPPER_CLOSED_DEG = 135;    // TODO: measure (closed = blocking)
    static final double STOPPER_OPEN_DEG = 180;      // TODO: measure (open = lets nectar through)
    static final double NECTAR_OPEN_TIME = 0.25;     // TODO: tune, long enough for exactly 1 nectar
    static final double NECTAR_RECOVER_TIME = 0.50;  // TODO: tune, flywheel recovery between shots

    // ---- turret (manual) ----
    // dpad left/right spins mvR at this power. NO LIMITS yet (encR isn't read), so don't wrap the wires.
    // if left/right are backwards, set INVERT_TURRET to true.
    static final double TURRET_MANUAL_POWER = 0.30;
    static final boolean INVERT_TURRET = false;

    // =========================================================================================
    //  HARDWARE  (null = not in config, that part is skipped)
    // =========================================================================================
    DcMotor tl, tr, bl, br;
    DcMotor intk, push, actR, actL;
    ServoImplEx flr, fll, stp;
    CRServo mvR;

    // devices that are disabled because they threw an error while running
    final Set<String> crashed = new HashSet<>();
    // hardware report shown during init
    final List<String> report = new ArrayList<>();
    int missingCount = 0;

    // =========================================================================================
    //  STATE
    // =========================================================================================
    enum Alliance { RED, BLUE }
    Alliance alliance = Alliance.RED;

    boolean intakeToggle = false, outtakeToggle = false;
    String flowerState = "untouched (limp)";
    boolean alignOnShoot = true; // BACK flips this (stub for now)

    boolean pollenHeld = false;
    final ElapsedTime pollenTimer = new ElapsedTime();

    enum NectarPhase { IDLE, REVVING, OPEN, RECOVER }
    NectarPhase nectarPhase = NectarPhase.IDLE;
    final ElapsedTime nectarTimer = new ElapsedTime();
    int nectarShots = 0;

    // smooth drive state: the power actually being sent right now (ramps toward the stick)
    double curStrafe = 0, curFwd = 0, curTurn = 0;
    final ElapsedTime driveTimer = new ElapsedTime();

    String turretStatus = "idle";
    String positionStatus = "-";

    // previous button states, for detecting a fresh press
    boolean pA, pB, pBack, pUp, pDown;

    // =========================================================================================
    //  INIT
    // =========================================================================================
    @Override
    public void init() {
        telemetry.log().setCapacity(6);

        // ---- drive (directions from pt, which drives correctly) ----
        tl = motor(TL, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        tr = motor(TR, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        bl = motor(BL, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        br = motor(BR, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- intake (from pt) ----
        intk = motor(INTK, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- pollen: shooter + ramp (from pt) ----
        // shooter stays BRAKE like pt. FLOAT would let the flywheel coast down instead of
        // slamming to a stop, try it if the shooter mount takes a beating.
        actR = motor(ACT_R, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        // pt comment: "CHANGE THIS TO REVERSE IF IT SPINS THE WRONG WAY"
        push = motor(PUSH, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- nectar: shooter + stopper (not built) ----
        // TODO: direction unknown until it's built. flip if it shoots backwards.
        actL = motor(ACT_L, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        stp = servo(STP, Servo.Direction.FORWARD, false);
        // stopper starts closed so nectar can't roll into the shooter
        if (stp != null) setServoDeg(stp, STOPPER_CLOSED_DEG, STOPPER_RANGE_DEG);

        // ---- flower releaser (mirrored pair, same setup as pt) ----
        // flr is the main one, fll is mounted mirrored so it runs REVERSE.
        // both start limp (PWM off) until the first dpad press, same as pt.
        flr = servo(FLR, Servo.Direction.FORWARD, true);
        fll = servo(FLL, Servo.Direction.REVERSE, true);

        // ---- turret ----
        mvR = hardwareMap.tryGet(CRServo.class, MV_R);
        if (mvR != null) {
            mvR.setDirection(INVERT_TURRET ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            mvR.setPower(0);
            reportLine(MV_R, "OK");
        } else {
            reportMissing(MV_R);
        }

        // ---- ignored for now: only check if they're in the config, don't use them ----
        reportStub(PPC, hardwareMap.tryGet(HardwareDevice.class, PPC) != null, "odometry ignored for now");
        reportStub(VIZ1, hardwareMap.tryGet(WebcamName.class, VIZ1) != null, "camera ignored for now");
        report.add("encR: STUB (read through a motor port, turret encoder ignored for now)");
    }

    @Override
    public void init_loop() {
        // ---- alliance select (only matters once positions/autoalign exist, but pick it anyway) ----
        if (gamepad1.b || gamepad2.b) alliance = Alliance.RED;
        if (gamepad1.x || gamepad2.x) alliance = Alliance.BLUE;

        telemetry.addData("ALLIANCE", "%s   (B = red, X = blue)", alliance);
        telemetry.addData("Hardware", missingCount == 0 ? "everything found" : missingCount + " missing (skipped)");
        for (String line : report) telemetry.addLine(line);

        // remember buttons held during init so they don't count as a press right at start
        pA = gamepad1.a; pB = gamepad1.b; pBack = gamepad1.back;
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
    }

    @Override
    public void start() {
        driveTimer.reset();
        // one short log line instead of the whole report
        telemetry.log().clear();
        telemetry.log().add("alliance " + alliance + ", " +
                (missingCount == 0 ? "all hardware found" : missingCount + " devices missing, see init report"));
    }

    // =========================================================================================
    //  LOOP
    // =========================================================================================
    @Override
    public void loop() {
        // each part runs inside safe(), so one broken part can't take down the rest
        safe("drive", this::drive);
        safe("intake", this::intake);
        safe("flower", this::flower);
        safe("pollen", this::pollen);
        safe("nectar", this::nectar);
        safe("turret", this::turret);
        safe("positions", this::positions);
        safe("secondary", this::secondaryController);

        pA = gamepad1.a; pB = gamepad1.b; pBack = gamepad1.back;
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;

        showTelemetry();
    }

    @Override
    public void stop() {
        for (DcMotor m : new DcMotor[]{tl, tr, bl, br, intk, push, actR, actL}) {
            if (m != null) m.setPower(0);
        }
        if (mvR != null) mvR.setPower(0);
    }

    // =========================================================================================
    //  DRIVE
    // =========================================================================================
    // same mecanum mix as pt (which works). changes: speed scaling + turn flipped.
    // stick naming is from pt: "strafe" = left stick x, "fwd" = left stick y (up is negative).
    void drive() {
        double wantStrafe = shape(gamepad1.left_stick_x) * DRIVE_SPEED;
        double wantFwd = shape(gamepad1.left_stick_y) * DRIVE_SPEED;
        double wantTurn = shape(gamepad1.right_stick_x) * TURN_SPEED * (INVERT_TURN ? -1 : 1);

        // time since last loop. capped so a laggy loop can't cause one big jump.
        double dt = Math.min(driveTimer.seconds(), 0.1);
        driveTimer.reset();

        // ramp each axis separately so the robot keeps its direction while slowing down
        curStrafe = ramp(curStrafe, wantStrafe, dt);
        curFwd = ramp(curFwd, wantFwd, dt);
        curTurn = ramp(curTurn, wantTurn, dt);

        double strafe = curStrafe, fwd = curFwd, turn = curTurn;

        double pTL = strafe + fwd + turn;
        double pTR = strafe - fwd - turn;
        double pBL = strafe - fwd + turn;
        double pBR = strafe + fwd - turn;

        // keep ratios the same if anything goes over 1
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(pTL), Math.abs(pTR)),
                Math.max(Math.abs(pBL), Math.abs(pBR))));

        power(tl, pTL / max);
        power(tr, pTR / max);
        power(bl, pBL / max);
        power(br, pBR / max);
    }

    // moves current toward target, but no faster than the ramp rate allows.
    // speeding up (same direction, bigger) uses RAMP_UP, everything else uses RAMP_DOWN.
    // reversing ramps down through 0 first, then ramps up the other way.
    double ramp(double current, double target, double dt) {
        if (!SMOOTH_DRIVE) return target;
        boolean speedingUp = current == 0
                || (Math.signum(target) == Math.signum(current) && Math.abs(target) > Math.abs(current));
        double maxStep = (speedingUp ? RAMP_UP_PER_SEC : RAMP_DOWN_PER_SEC) * dt;
        return current + Range.clip(target - current, -maxStep, maxStep);
    }

    // =========================================================================================
    //  INTAKE
    // =========================================================================================
    // RB hold = intake, LB hold = outtake (holds always win).
    // A / B = toggles. turning one toggle on turns the other off.
    void intake() {
        if (gamepad1.a && !pA) { intakeToggle = !intakeToggle; if (intakeToggle) outtakeToggle = false; }
        if (gamepad1.b && !pB) { outtakeToggle = !outtakeToggle; if (outtakeToggle) intakeToggle = false; }

        double p;
        if (gamepad1.right_bumper) p = INTAKE_POWER;
        else if (gamepad1.left_bumper) p = OUTTAKE_POWER;
        else if (intakeToggle) p = INTAKE_POWER;
        else if (outtakeToggle) p = OUTTAKE_POWER;
        else p = 0;

        power(intk, p);
    }

    // =========================================================================================
    //  FLOWER RELEASER
    // =========================================================================================
    // dpad up = engaged (155), dpad down = disengaged (70). both servos get the same angle,
    // fll is REVERSE so it mirrors flr.
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

    // =========================================================================================
    //  POLLEN SHOOTER  (RT hold)
    // =========================================================================================
    // RT pressed  -> actR at rev power, timer starts
    // after POLLEN_REV_TIME -> actR at shoot power, push (ramp) on
    // RT released -> both off
    // also autoaligns the turret while held (stub, see turret())
    void pollen() {
        boolean held = gamepad1.right_trigger > 0.5;
        if (held && !pollenHeld) pollenTimer.reset();
        pollenHeld = held;

        if (!held) {
            power(actR, 0);
            power(push, 0);
            return;
        }

        boolean revved = pollenTimer.seconds() >= POLLEN_REV_TIME;
        power(actR, revved ? POLLEN_SHOOT_POWER : POLLEN_REV_POWER);
        power(push, revved ? PUSH_POWER : 0);
    }

    // =========================================================================================
    //  NECTAR SHOOTER  (LT hold)
    // =========================================================================================
    // state machine, see the TUNING section for the full sequence.
    void nectar() {
        boolean held = gamepad1.left_trigger > 0.5;

        if (!held) {
            nectarPhase = NectarPhase.IDLE;
            power(actL, 0);
            stopper(false);
            return;
        }

        switch (nectarPhase) {
            case IDLE:
                // just pressed
                nectarPhase = NectarPhase.REVVING;
                nectarTimer.reset();
                nectarShots = 0;
                power(actL, NECTAR_REV_POWER);
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
        power(actL, NECTAR_SHOOT_POWER);
        stopper(true);
        nectarPhase = NectarPhase.OPEN;
        nectarTimer.reset();
        nectarShots++;
    }

    void stopper(boolean open) {
        if (stp != null) setServoDeg(stp, open ? STOPPER_OPEN_DEG : STOPPER_CLOSED_DEG, STOPPER_RANGE_DEG);
    }

    // =========================================================================================
    //  TURRET  (dpad left/right manual, autoalign STUB)
    // =========================================================================================
    // manual always wins over autoalign.
    //
    // STUB: autoalign. turret + camera are ignored for now. when we get to it:
    //   needs: viz1 exposure tuned, encR plugged into a free motor encoder port, turret tick limits
    //   measured, rosy's working TurretAligner merged into utilities/ (taking servo + encoder names).
    //   then: init AprilTagWebcam on VIZ1 and a TurretAligner on mvR/encR in init(), and in
    //   autoalign() pick the alliance's cluster (pollenCam.getBestClusterStartingWith(alliance.name()))
    //   and call aligner.update(cluster). the utilities/ classes already have most of this.
    void turret() {
        if (gamepad1.back && !pBack) {
            alignOnShoot = !alignOnShoot;
            telemetry.log().add("autoalign on shoot " + (alignOnShoot ? "ON" : "OFF") + " (stub)");
        }

        double manual = 0;
        if (gamepad1.dpad_right && !gamepad1.dpad_left) manual = TURRET_MANUAL_POWER;
        if (gamepad1.dpad_left && !gamepad1.dpad_right) manual = -TURRET_MANUAL_POWER;

        if (manual != 0) {
            if (mvR != null) mvR.setPower(manual);
            turretStatus = mvR == null ? "manual (no mvR)" : (manual > 0 ? "manual RIGHT" : "manual LEFT");
            return;
        }

        if (pollenHeld && alignOnShoot) {
            autoalign();
            return;
        }

        if (mvR != null) mvR.setPower(0);
        turretStatus = "idle";
    }

    void autoalign() {
        // STUB (see comment above turret())
        if (mvR != null) mvR.setPower(0);
        turretStatus = "autoalign (stub, does nothing yet)";
    }

    // =========================================================================================
    //  DRIVE TO POSITION  (X / Y hold)  -- STUB
    // =========================================================================================
    // deadwheels / imu are ignored for now, so X and Y only show a status and the robot
    // keeps driving normally.
    //   needs: ppc configured (pod offsets, pod type, directions) + calibrated, a known start
    //   pose (or auto passes its end pose through the blackboard), and 2 measured field
    //   positions per alliance. then: P controller from ppc pose to the target, same idea as the
    //   driveToPose() from the previous prelim (still in git history, commit 6b4ec21).
    void positions() {
        if (gamepad1.x) positionStatus = "position 1 (" + alliance + ") STUB";
        else if (gamepad1.y) positionStatus = "position 2 (" + alliance + ") STUB";
        else positionStatus = "-";
    }

    // =========================================================================================
    //  SECONDARY CONTROLLER (gamepad2)  -- STUB
    // =========================================================================================
    // ignored for now. planned: turret manual override with a stick, shooter power trim,
    // pinpoint reset, flower fine adjust.
    void secondaryController() {
    }

    // =========================================================================================
    //  TELEMETRY
    // =========================================================================================
    void showTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Intake", gamepad1.right_bumper ? "IN (held)" : gamepad1.left_bumper ? "OUT (held)"
                : intakeToggle ? "IN" : outtakeToggle ? "OUT" : "off");
        telemetry.addData("Flower", flowerState);
        telemetry.addData("Pollen", !pollenHeld ? "off"
                : pollenTimer.seconds() < POLLEN_REV_TIME
                ? String.format("revving %.1fs", POLLEN_REV_TIME - pollenTimer.seconds())
                : "FIRING");
        telemetry.addData("Nectar", nectarPhase == NectarPhase.IDLE ? "off" : nectarPhase + "  shots " + nectarShots);
        telemetry.addData("Turret", "%s  | align on shoot: %s", turretStatus, alignOnShoot ? "ON" : "OFF");
        if (!positionStatus.equals("-")) telemetry.addData("Position", positionStatus);
    }

    // =========================================================================================
    //  HELPERS
    // =========================================================================================

    // load a motor, or report it missing and return null
    DcMotor motor(String name, DcMotorSimple.Direction dir, DcMotor.ZeroPowerBehavior zpb) {
        DcMotor m = hardwareMap.tryGet(DcMotor.class, name);
        if (m == null) { reportMissing(name); return null; }
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(zpb);
        m.setPower(0);
        reportLine(name, "OK");
        return m;
    }

    // load a positional servo (500-2500us = full 270 deg range on goBILDA/REV servos)
    // startLimp = PWM off until first command, so it doesn't jump on init
    ServoImplEx servo(String name, Servo.Direction dir, boolean startLimp) {
        ServoImplEx s = hardwareMap.tryGet(ServoImplEx.class, name);
        if (s == null) { reportMissing(name); return null; }
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

    void power(DcMotor m, double p) {
        if (m != null) m.setPower(p);
    }

    // deadzone + optional squaring, same as pt
    double shape(double v) {
        if (Math.abs(v) < DEADZONE) return 0;
        v = Math.signum(v) * (Math.abs(v) - DEADZONE) / (1.0 - DEADZONE);
        if (SQUARE_INPUTS) v = v * Math.abs(v);
        return v;
    }

    // runs one part of the loop. if it throws, that part is turned off for the rest of the
    // match and a log line says so, everything else keeps going.
    void safe(String part, Runnable r) {
        if (crashed.contains(part)) return;
        try {
            r.run();
        } catch (Exception e) {
            crashed.add(part);
            telemetry.log().add(part + " DISABLED: " + e.getClass().getSimpleName() + " " + e.getMessage());
            // make sure whatever that part was running doesn't stay stuck on
            try { stopPart(part); } catch (Exception ignored) { }
        }
    }

    void stopPart(String part) {
        switch (part) {
            case "drive":  power(tl, 0); power(tr, 0); power(bl, 0); power(br, 0);
                           curStrafe = 0; curFwd = 0; curTurn = 0; break;
            case "intake": power(intk, 0); break;
            case "pollen": power(actR, 0); power(push, 0); break;
            case "nectar": power(actL, 0); break;
            case "turret": if (mvR != null) mvR.setPower(0); break;
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
        report.add(name + ": STUB, " + (found ? "in config" : "not in config") + ", " + why);
    }
}
