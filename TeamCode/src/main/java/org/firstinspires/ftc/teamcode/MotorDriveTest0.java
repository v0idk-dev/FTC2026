package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Port0 Driving Motor Test", group = "Test")
public class MotorDriveTest0 extends LinearOpMode {

    // flip any of these to true/false if a wheel spins the wrong way
    static final boolean REVERSE_TL = true;
    static final boolean REVERSE_TR = false;
    static final boolean REVERSE_BL = true;
    static final boolean REVERSE_BR = false;

    static final double SHOVEL_RANGE_DEG = 270;
    static final double SHOVEL_START_TARGET_DEG = 135;
    static final double DEADZONE = 0.05;
    static final boolean SQUARE_INPUTS = true; // finer control at low push

    @Override
    public void runOpMode() {
        // Get driving motors
        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        tl.setDirection(REVERSE_TL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        tr.setDirection(REVERSE_TR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        bl.setDirection(REVERSE_BL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        br.setDirection(REVERSE_BR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{tl, tr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Get shovel servos
        ServoImplEx mainServo =
                hardwareMap.tryGet(ServoImplEx.class, "main_servo");
        ServoImplEx mirrorServo =
                hardwareMap.tryGet(ServoImplEx.class, "mirror_servo");

        telemetry.addData("main_servo",
                mainServo != null ? "FOUND in configuration" : "MISSING");
        telemetry.addData("mirror_servo",
                mirrorServo != null ? "FOUND in configuration" : "MISSING");
        telemetry.addLine("Physical cable connection cannot be detected.");
        telemetry.update();

        if (mainServo == null || mirrorServo == null) {
            throw new RuntimeException("SERVO(S) NOT FOUND.\nmain shovel servo: " + mainServo + "\nmirror shovel servo: " + mirrorServo);
        }

        mainServo.setDirection(Servo.Direction.FORWARD);
        mirrorServo.setDirection(Servo.Direction.REVERSE);

        mainServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        mirrorServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        double target = SHOVEL_START_TARGET_DEG;
        double current = Double.NaN;

        boolean pRB = false, pLB = false;
        boolean pR = false, pL = false, pU = false, pD = false;
        boolean pA = false, pB = false;

        // the Expansion Hub is the hub that is NOT the parent (Control Hub) module
        int expansionAddress = -1;
        for (LynxModule mod : hardwareMap.getAll(LynxModule.class)) {
            if (!mod.isParent()) expansionAddress = mod.getModuleAddress();
        }
        // end shovel servo setup

        // find the test motor on port 0 of the Expansion Hub, ignoring its name
        DcMotor intakeMotor = null;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
//        for (DcMotor m : hardwareMap.getAll(DcMotor.class)) {
//            if (m.getPortNumber() != 0) continue;
//            if (m.getController() instanceof LynxDcMotorController) {
//                int addr = ((LynxDcMotorController) m.getController()).getModule().getModuleAddress();
//                if (addr == expansionAddress) {
//                    intakeMotor = m;
//                    break;
//                }
//            }
//        }
        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();

        telemetry.addData("main_servo", "Configured | Port %d (expected 0)",
                mainServo.getPortNumber());
        telemetry.addData("mirror_servo", "Configured | Port %d (expected 2)",
                mirrorServo.getPortNumber());
        telemetry.addLine("Ready. D-pad or A will command both servos.");
        telemetry.addLine("Physical cable connection cannot be detected.");
        telemetry.update();

        while (opModeIsActive()) {
            // ---------------- driving ----------------
            // left stick axes are swapped relative to the original code:
            // stick up/down now feeds "x", stick left/right now feeds "y"
            // if a direction ends up backwards, add or remove the minus sign on that line
            double y  = shape(gamepad1.left_stick_x);   // stick left/right
            double x  = shape(gamepad1.left_stick_y);  // stick up/down
            double rx = shape(gamepad1.right_stick_x);  // turn (right = clockwise)
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            boolean r = gamepad1.dpad_right;
            boolean l = gamepad1.dpad_left;
            boolean u = gamepad1.dpad_up;
            boolean d = gamepad1.dpad_down;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            double pTL = y + x + rx;
            double pTR = y - x - rx;
            double pBL = y - x + rx;
            double pBR = y + x - rx;
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(pTL), Math.abs(pTR)),
                    Math.max(Math.abs(pBL), Math.abs(pBR))));
            pTL /= max; pTR /= max; pBL /= max; pBR /= max;

            // ---------------- intake (X = forward, Y = reverse, 100%) ----------------
            double testPower = 0;
            if (gamepad1.x && !gamepad1.y) testPower = 1.0;
            else if (gamepad1.y && !gamepad1.x) testPower = -1.0;

            // ---------------- send power ----------------
            tl.setPower(pTL);
            tr.setPower(pTR);
            bl.setPower(pBL);
            br.setPower(pBR);
            intakeMotor.setPower(testPower);   // always set, so release = stop

            // ---------------- shovel ----------------
            // B: disable PWM
            if (b && !pB) {
                mainServo.setPwmDisable();
                mirrorServo.setPwmDisable();
                current = Double.NaN;
            }

            // bumpers adjust the target
            if (rb && !pRB) target += 10;
            if (lb && !pLB) target -= 10;
            target = Range.clip(target, 0, SHOVEL_RANGE_DEG);

            // A: go to target
            if (a && !pA && !b) {
                current = target;
                mainServo.setPosition(current / SHOVEL_RANGE_DEG);
                mirrorServo.setPosition(current / SHOVEL_RANGE_DEG);
                mainServo.setPwmEnable();
                mirrorServo.setPwmEnable();
            }

            // D-pad: jog the shovel (works even if A has not been pressed yet)
            if (!b) {
                double step = 0;
                if (r && !pR) step += 5;
                if (l && !pL) step -= 5;
                if (u && !pU) step += 15;
                if (d && !pD) step -= 15;
                if (step != 0) {
                    // no position commanded yet: start jogging from the target
                    if (Double.isNaN(current)) current = target;
                    current = Range.clip(current + step, 0, SHOVEL_RANGE_DEG);
                    target = current;
                    mainServo.setPosition(current / SHOVEL_RANGE_DEG);
                    mirrorServo.setPosition(current / SHOVEL_RANGE_DEG);
                    mainServo.setPwmEnable();
                    mirrorServo.setPwmEnable();
                }
            }

            pRB = rb; pLB = lb;
            pR = r; pL = l; pU = u; pD = d;
            pA = a; pB = b;

            // ---------------- telemetry ----------------
            telemetry.addData("Test motor", intakeMotor != null ? "Expansion Hub port 0" : "NOT FOUND");
            telemetry.addData("Current power", "%.2f", testPower);
            telemetry.addLine("");
            telemetry.addData("Drive TL / TR", "%.2f / %.2f", pTL, pTR);
            telemetry.addData("Drive BL / BR", "%.2f / %.2f", pBL, pBR);
            telemetry.addLine("");
            telemetry.addLine("Left stick: move | Right stick L/R: turn");
            telemetry.addLine("X: intake fwd 100% | Y: intake rev 100% (release = stop)");
            // Shovel telemetry
            telemetry.addData("main_servo", "Configured | Port %d | PWM %s",
                    mainServo.getPortNumber(),
                    mainServo.isPwmEnabled() ? "ON" : "OFF");

            telemetry.addData("mirror_servo", "Configured | Port %d | PWM %s",
                    mirrorServo.getPortNumber(),
                    mirrorServo.isPwmEnabled() ? "ON" : "OFF");

            telemetry.addData("Physical connection", "Verify by observing movement");

            telemetry.addData(
                    "Last commanded position",
                    Double.isNaN(current)
                            ? "unknown / PWM disabled"
                            : String.format("%.1f", current)
            );
            telemetry.addData("Target position", "%.1f", target);
            telemetry.addLine("RB / LB: shovel target +/- 10");
            telemetry.addLine("A: go to target | B: disable PWM");
            telemetry.addLine("D-pad: jog shovel 5 (L/R) or 15 (U/D)");
            telemetry.update();
        }

        tl.setPower(0);
        tr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    // deadzone, then rescale so output still starts at 0, then optional squaring
    private double shape(double v) {
        if (Math.abs(v) < DEADZONE) return 0;
        v = Math.signum(v) * (Math.abs(v) - DEADZONE) / (1.0 - DEADZONE);
        return SQUARE_INPUTS ? v * Math.abs(v) : v;
    }
}