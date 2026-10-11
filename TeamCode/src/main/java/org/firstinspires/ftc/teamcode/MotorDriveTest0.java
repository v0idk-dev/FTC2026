package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Port0 Driving Motor Test", group = "Test")
public class MotorDriveTest0 extends LinearOpMode {

    // flip any of these to true/false if a wheel spins the wrong way
    static final boolean REVERSE_TL = true;
    static final boolean REVERSE_TR = false;
    static final boolean REVERSE_BL = true;
    static final boolean REVERSE_BR = false;

    static final double DEADZONE = 0.05;
    static final boolean SQUARE_INPUTS = true; // finer control at low push

    @Override
    public void runOpMode() {
        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        Servo shovelServo = hardwareMap.get(Servo.class, "shovel");
        // TODO: IMPLEMENT SERVO CONTROLS!!

        tl.setDirection(REVERSE_TL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        tr.setDirection(REVERSE_TR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        bl.setDirection(REVERSE_BL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        br.setDirection(REVERSE_BR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{tl, tr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // the Expansion Hub is the hub that is NOT the parent (Control Hub) module
        int expansionAddress = -1;
        for (LynxModule mod : hardwareMap.getAll(LynxModule.class)) {
            if (!mod.isParent()) expansionAddress = mod.getModuleAddress();
        }

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

        int targetPercent = 50;
        boolean pU = false, pD = false, pR = false, pL = false;

        waitForStart();

        while (opModeIsActive()) {
            // ---------------- driving ----------------
            double y  = shape(-gamepad1.left_stick_y);  // forward
            double x  = shape(gamepad1.left_stick_x);   // strafe
            double rx = shape(gamepad1.right_stick_x);  // turn (right = clockwise)

            double pTL = y + x + rx;
            double pTR = y - x - rx;
            double pBL = y - x + rx;
            double pBR = y + x - rx;

            // scale all together so direction is kept when something goes over 1
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(pTL), Math.abs(pTR)),
                    Math.max(Math.abs(pBL), Math.abs(pBR))));
            pTL /= max; pTR /= max; pBL /= max; pBR /= max;

            // ---------------- target percentage (d-pad up/down = 5, right/left = 1) ----------------
            boolean u = gamepad1.dpad_up, d = gamepad1.dpad_down;
            boolean r = gamepad1.dpad_right, l = gamepad1.dpad_left;
            if (u && !pU) targetPercent += 5;
            if (d && !pD) targetPercent -= 5;
            if (r && !pR) targetPercent += 1;
            if (l && !pL) targetPercent -= 1;
            targetPercent = (int) Range.clip(targetPercent, 0, 100);
            pU = u; pD = d; pR = r; pL = l;
            double targetSpeed = targetPercent / 100.0;

            // ---------------- X/B speed: Y = 100%, A = 75%, default 50% ----------------
            double xbSpeed = gamepad1.y ? 1.0 : (gamepad1.a ? 0.75 : 0.5);

            double xbPower = 0;
            if (gamepad1.x && !gamepad1.b) xbPower = xbSpeed;
            else if (gamepad1.b && !gamepad1.x) xbPower = -xbSpeed;

            // ---------------- triggers run at the target percentage ----------------
            boolean rt = gamepad1.right_trigger > 0.5;
            boolean lt = gamepad1.left_trigger > 0.5;
            double trigPower = 0;
            if (rt && !lt) trigPower = targetSpeed;
            else if (lt && !rt) trigPower = -targetSpeed;

            // if X/B is held it wins, otherwise the triggers drive
            double testPower = (xbPower != 0) ? xbPower : trigPower;

            // ---------------- send power ----------------
            tl.setPower(pTL);
            tr.setPower(pTR);
            bl.setPower(pBL);
            br.setPower(pBR);

            // test motor overrides drive power on that motor while a test input is held
            if (intakeMotor != null && testPower != 0) {
                intakeMotor.setPower(testPower);
            }

            // ---------------- telemetry ----------------
            telemetry.addData("Test motor", intakeMotor != null ? "Expansion Hub port 0" : "NOT FOUND");
            telemetry.addData("Current power", "%.2f", testPower);
            telemetry.addData("Target power (X/B)", "%.2f", xbSpeed);
            telemetry.addData("Target percentage (triggers)", "%d%%", targetPercent);
            telemetry.addLine("");
            telemetry.addData("Drive TL / TR", "%.2f / %.2f", pTL, pTR);
            telemetry.addData("Drive BL / BR", "%.2f / %.2f", pBL, pBR);
            telemetry.addLine("");
            telemetry.addLine("Left stick: move | Right stick L/R: turn");
            telemetry.addLine("X: forward | B: backward (release = stop)");
            telemetry.addLine("Hold Y: 100% | Hold A: 75% | Default: 50%");
            telemetry.addLine("Right trigger: forward | Left trigger: backward (at target %)");
            telemetry.addLine("D-pad U/D: target +/- 5% | D-pad R/L: target +/- 1%");
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