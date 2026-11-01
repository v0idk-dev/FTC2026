package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

// gamepad controls
// joystick: drive
// button:
// x and y to move lift_servo
// b to disable pwm lift_servo
// a to activate (set lift_servo to target angle)
// dpad:
// left and right for 5 degree change
// up and down for 15 degree change
// WHEN YOU CONTROL IT FOR THE FIRST TIME, NO MATTER WHICH BUTTON YOU PRESS, THE FIRST TARGET ANGLE WILL BE START_TARGET_DEG + whatever you commanded
// trigger:
// right for 50% of misc motor (named intake)
// left for 75% of misc motor (named intake)

@TeleOp(name = "Port0 Driving Motor Test", group = "Test")
public class MotorDriveTest0 extends LinearOpMode {

    // both servos have a limit of 270 degrees btw
    static final double shovel_range_deg = 270;
    static final double shovel_start_target_deg = 135;

    static final double lift_range_deg = 270;
    static final double lift_start_target_deg = 135;

    static final double deadzone = 0.05;
    static final boolean square_inputs = true;

    @Override
    public void runOpMode() {

        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        tl.setDirection(DcMotorSimple.Direction.REVERSE);
        tr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{tl, tr, bl, br}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // for flower collector (port 0 and 2)
        ServoImplEx mainServo =
                hardwareMap.get(ServoImplEx.class, "main_servo");

        ServoImplEx mirrorServo =
                hardwareMap.get(ServoImplEx.class, "mirror_servo");

        mainServo.setDirection(Servo.Direction.FORWARD);
        mirrorServo.setDirection(Servo.Direction.REVERSE);

        mainServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        mirrorServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        ServoImplEx[] shovelServos = {
                mainServo, mirrorServo
        };

        double shovelTarget = shovel_start_target_deg;


        // TODO add turret servo w/ enocder on port 3
        // literally just copy code for EncoderPositionController into here (no visipn yet)

        // for servo that feeds into turret (port 4)
        ServoImplEx liftServo =
                hardwareMap.get(ServoImplEx.class, "lift_servo");


        liftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        liftServo.setDirection(Servo.Direction.FORWARD);

        double liftTarget = lift_start_target_deg;

        mainServo.setPwmDisable();
        mirrorServo.setPwmDisable();
        liftServo.setPwmDisable();


        // port 0 on expansion hub (could be for intake or shooting)
        // make sure the expansion hub light is green or else this entire program won't run
        // might need to change battery
        DcMotor intakeMotor =
                hardwareMap.get(DcMotor.class, "intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        boolean pRight = false;
        boolean pLeft = false;
        boolean pUp = false;
        boolean pDown = false;

        boolean pX = false;
        boolean pY = false;
        boolean pA = false;
        boolean pB = false;

        waitForStart();

        try {
            while (opModeIsActive()) {

                // drove
                double y = shape(gamepad1.left_stick_x);
                double x = shape(gamepad1.left_stick_y);
                double rx = shape(gamepad1.right_stick_x);

                double pTL = y + x + rx;
                double pTR = y - x - rx;
                double pBL = y - x + rx;
                double pBR = y + x - rx;

                double max = Math.max(
                        1.0,
                        Math.max(
                                Math.max(Math.abs(pTL), Math.abs(pTR)),
                                Math.max(Math.abs(pBL), Math.abs(pBR))
                        )
                );

                pTL /= max;
                pTR /= max;
                pBL /= max;
                pBR /= max;

                tl.setPower(pTL);
                tr.setPower(pTR);
                bl.setPower(pBL);
                br.setPower(pBR);

                // intake/shooter/misc motor
                // right tigger for 50 percent
                // left trigger for 75 percent
                double intakePower = 0;

                if (gamepad1.left_trigger > 0.5) {
                    intakePower = 0.75;
                } else if (gamepad1.right_trigger > 0.5) {
                    intakePower = 0.50;
                }

                intakeMotor.setPower(intakePower);

                // buttons
                boolean right = gamepad1.dpad_right;
                boolean left = gamepad1.dpad_left;
                boolean up = gamepad1.dpad_up;
                boolean down = gamepad1.dpad_down;

                boolean buttonX = gamepad1.x;
                boolean buttonY = gamepad1.y;
                boolean buttonA = gamepad1.a;
                boolean buttonB = gamepad1.b;

                // flower eater (dpad)
                double shovelStep = 0;

                if (right && !pRight && !left) {
                    shovelStep += 5;
                }

                if (left && !pLeft && !right) {
                    shovelStep -= 5;
                }

                if (up && !pUp && !down) {
                    shovelStep += 15;
                }

                if (down && !pDown && !up) {
                    shovelStep -= 15;
                }

                if (shovelStep != 0) {
                    shovelTarget = Range.clip(
                            shovelTarget + shovelStep,
                            0,
                            shovel_range_deg
                    );

                    for (ServoImplEx servo : shovelServos) {
                        servo.setPosition(
                                shovelTarget / shovel_range_deg
                        );
                        servo.setPwmEnable();
                    }

                }

                // lift servo (xyab)
                // x and y for up and down (idk which is which)
                // b to stop
                // a to go to target
                if (buttonB && !pB) {
                    liftServo.setPwmDisable();
                }

                if (!buttonB) {
                    double liftStep = 0;

                    if (buttonX && !pX && !buttonY) {
                        liftStep = 5;
                    }

                    if (buttonY && !pY && !buttonX) {
                        liftStep = -5;
                    }


                    boolean goToTarget = buttonA && !pA;

                    if (liftStep != 0 || goToTarget) {
                        liftTarget = Range.clip(
                                liftTarget + liftStep,
                                0,
                                lift_range_deg
                        );

                        liftServo.setPosition(
                                liftTarget / lift_range_deg
                        );
                        liftServo.setPwmEnable();

                    }
                }

                pRight = right;
                pLeft = left;
                pUp = up;
                pDown = down;

                pX = buttonX;
                pY = buttonY;
                pA = buttonA;
                pB = buttonB;


                telemetry.addData("Drive TL / TR","%.2f / %.2f", pTL, pTR);
                telemetry.addData("Drive BL / BR", "%.2f / %.2f", pBL, pBR);

                telemetry.addData("Main/mirror target", "%.1f degrees", shovelTarget);
                telemetry.addData("Lift target", "%.1f degrees", liftTarget);
                telemetry.addData("Intake power", "%.0f%%", intakePower * 100);

                telemetry.update();
            }
        } finally {
            tl.setPower(0);
            tr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            intakeMotor.setPower(0);
        }
    }

    private double shape(double v) {
        if (Math.abs(v) < deadzone) {
            return 0;
        }

        v = Math.signum(v)
                * (Math.abs(v) - deadzone)
                / (1.0 - deadzone);

        if (square_inputs) {
            v = v * Math.abs(v);
        }

        return v;
    }
}