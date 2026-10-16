package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Port0 Servo Test")
public class flower_eater_servo_controller extends LinearOpMode {

    static final double RANGE_DEG = 270;
    static final double START_TARGET_DEG = 135;

    @Override
    public void runOpMode() {
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
            waitForStart();
            return;
        }

        mainServo.setDirection(Servo.Direction.FORWARD);
        mirrorServo.setDirection(Servo.Direction.REVERSE);

        mainServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        mirrorServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        double target = START_TARGET_DEG;
        double current = Double.NaN;

        boolean pRB = false, pLB = false, pRT = false, pLT = false;
        boolean pR = false, pL = false, pU = false, pD = false;
        boolean pA = false, pB = false;

        waitForStart();

        telemetry.addData("main_servo", "Configured | Port %d (expected 0)",
                mainServo.getPortNumber());
        telemetry.addData("mirror_servo", "Configured | Port %d (expected 2)",
                mirrorServo.getPortNumber());
        telemetry.addLine("Ready. Press START, then A to command both servos.");
        telemetry.addLine("Physical cable connection cannot be detected.");
        telemetry.update();

        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            boolean rt = gamepad1.right_trigger > 0.5;
            boolean lt = gamepad1.left_trigger > 0.5;
            boolean r = gamepad1.dpad_right;
            boolean l = gamepad1.dpad_left;
            boolean u = gamepad1.dpad_up;
            boolean d = gamepad1.dpad_down;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            // stop everything
            if (b && !pB) {
                mainServo.setPwmDisable();
                mirrorServo.setPwmDisable();
                current = Double.NaN;
            }

            // gamepad asjusts
            if (rb && !pRB) target += 1;
            if (lb && !pLB) target -= 1;
            if (rt && !pRT) target += 10;
            if (lt && !pLT) target -= 10;
            target = Range.clip(target, 0, RANGE_DEG);

            // set current to target
            if (a && !pA && !b) {
                current = target;

                mainServo.setPosition(current / RANGE_DEG);
                mirrorServo.setPosition(current / RANGE_DEG);

                mainServo.setPwmEnable();
                mirrorServo.setPwmEnable();
            }

            // move in increments
            if (!Double.isNaN(current) && !b) {
                double step = 0;
                if (r && !pR) step += 5;
                if (l && !pL) step -= 5;
                if (u && !pU) step += 15;
                if (d && !pD) step -= 15;

                if (step != 0) {
                    current = Range.clip(current + step, 0, RANGE_DEG);
                    target = current;

                    mainServo.setPosition(current / RANGE_DEG);
                    mirrorServo.setPosition(current / RANGE_DEG);
                }
            }

            pRB = rb;
            pLB = lb;
            pRT = rt;
            pLT = lt;
            pR = r;
            pL = l;
            pU = u;
            pD = d;
            pA = a;
            pB = b;

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
            telemetry.addLine("Bumpers: target +/- 1");
            telemetry.addLine("Triggers: target +/- 10");
            telemetry.addLine("A: go to target");
            telemetry.addLine("D-pad L/R: move 5 | U/D: move 15");
            telemetry.addLine("B: disable PWM (releases holding torque)");
            telemetry.update();

            idle();
        }
    }
}