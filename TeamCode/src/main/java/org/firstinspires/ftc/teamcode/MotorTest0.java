package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Port0 Motor", group = "Test")
public class Port0Motor extends LinearOpMode {

    @Override
    public void runOpMode() {
        // find the motor on port 0, ignoring its configured name
        DcMotor motor = null;
        for (DcMotor m : hardwareMap.getAll(DcMotor.class)) {
            if (m.getPortNumber() == 0) {
                motor = m;
                break;
            }
        }

        if (motor == null) {
            telemetry.addLine("No motor configured on port 0");
            telemetry.update();
            waitForStart();
            return;
        }

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean targetMode = false;   // false = X/B mode, true = target percentage mode
        int targetPercent = 50;
        boolean pStart = false, pU = false, pD = false, pR = false, pL = false;

        waitForStart();

        while (opModeIsActive()) {
            boolean start = gamepad1.start;
            boolean u = gamepad1.dpad_up, d = gamepad1.dpad_down;
            boolean r = gamepad1.dpad_right, l = gamepad1.dpad_left;

            // START: switch modes
            if (start && !pStart) targetMode = !targetMode;

            double power = 0;
            double shownTarget;

            if (!targetMode) {
                // Y = 100%, A = 75%, default 50%
                double speed = gamepad1.y ? 1.0 : (gamepad1.a ? 0.75 : 0.5);
                shownTarget = speed;

                if (gamepad1.x && !gamepad1.b) power = speed;
                else if (gamepad1.b && !gamepad1.x) power = -speed;
            } else {
                // d-pad: up/down = 5%, right/left = 1%
                if (u && !pU) targetPercent += 5;
                if (d && !pD) targetPercent -= 5;
                if (r && !pR) targetPercent += 1;
                if (l && !pL) targetPercent -= 1;
                targetPercent = (int) Range.clip(targetPercent, 0, 100);
                shownTarget = targetPercent / 100.0;

                boolean rt = gamepad1.right_trigger > 0.5;
                boolean lt = gamepad1.left_trigger > 0.5;
                if (rt && !lt) power = shownTarget;
                else if (lt && !rt) power = -shownTarget;
            }

            pStart = start; pU = u; pD = d; pR = r; pL = l;

            motor.setPower(power);

            telemetry.addData("Mode", targetMode ? "Motor (target percentage)" : "Motor (X/B)");
            telemetry.addData("Current power", "%.2f", power);
            telemetry.addData("Target power", "%.2f", shownTarget);
            telemetry.addLine("");
            telemetry.addLine("START: switch mode");
            if (!targetMode) {
                telemetry.addLine("X: forward | B: backward (release = stop)");
                telemetry.addLine("Hold Y: 100% | Hold A: 75% | Default: 50%");
            } else {
                telemetry.addLine("Right trigger: forward | Left trigger: backward");
                telemetry.addLine("D-pad U/D: target +/- 5% | D-pad R/L: target +/- 1%");
            }
            telemetry.update();
        }

        motor.setPower(0);
    }
}
