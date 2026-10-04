package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Port0 Motor Test", group = "Test")
public class MotorTest0 extends LinearOpMode {

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

        waitForStart();

        while (opModeIsActive()) {
            // speed modifier: Y = 100%, A = 75%, default 50% (Y wins if both held)
            double speed = gamepad1.y ? 1.0 : (gamepad1.a ? 0.75 : 0.5);

            double power = 0;
            if (gamepad1.x && !gamepad1.b) {
                power = speed;       // forward
            } else if (gamepad1.b && !gamepad1.x) {
                power = -speed;      // backward
            }                        // neither, or both: stop

            motor.setPower(power);

            telemetry.addData("Power", power);
            telemetry.update();
        }

        motor.setPower(0);
    }
}
