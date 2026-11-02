package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// gamepad1 controls
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
// left for powering the intake motor
// right for powering the pollen launch motor

// gamepad2 controls
// joystick left: nectar servo (not implemented yet)
// joystick left: direct pollen servo

@TeleOp(name = "Port0 Driving Motor Test", group = "Test")
public class MotorDriveTest0 extends LinearOpMode {

    // both servos have a limit of 270 degrees btw
    static final double shovel_range_deg = 270;
    static final double shovel_start_target_deg = 135;

    static final double lift_start_percent = 0;
    static final double lift_end_percent = 1; // change this value based on experimentation
    double transferServoUpTime;

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

        // Set up turret encoder controller(s)

//        ThroughBoreEncoder pollenThroughBoreEncoder = new ThroughBoreEncoder(hardwareMap, "pollen_bore_encoder");
//        CRServo pollenServo = hardwareMap.get(CRServo.class, "pollen_servo");

//        EncoderPositionController pollenTurretController = new EncoderPositionController(pollenThroughBoreEncoder, pollenServo);

        // for servo that feeds into turret (port 4)
        ServoImplEx liftServo =
                hardwareMap.get(ServoImplEx.class, "lift_servo");

        liftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        liftServo.setDirection(Servo.Direction.FORWARD);

        mainServo.setPwmDisable();
        mirrorServo.setPwmDisable();
        liftServo.setPwmDisable();


        // make sure the expansion hub light is green or else this entire program won't run
        // might need to change battery
        DcMotor intakeMotor =
                hardwareMap.get(DcMotor.class, "intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        DcMotor pollenLaunchMotor =
                hardwareMap.get(DcMotor.class, "pollen_launch");

        pollenLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pollenLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenLaunchMotor.setPower(0);

        boolean pRight = false;
        boolean pLeft = false;
        boolean pUp = false;
        boolean pDown = false;

        boolean pX = false;
        boolean pY = false;
        boolean pA = false;
        boolean pB = false;

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        runtime.reset();

        liftServo.setPosition(0);

        try {
            while (opModeIsActive()) {

                // drove
                double y = shape(gamepad1.left_stick_x);
                double x = shape(gamepad1.left_stick_y);
                double rx = shape(gamepad1.right_stick_x);

                // 2nd gamepad for shooting
                // problem: we don't know the range of the turret servos.
                double x2 = shape(gamepad2.left_stick_x);
                double y2 = shape(gamepad2.left_stick_y);
                double rx2 = shape(gamepad2.right_stick_x);
                double ry2 = shape(gamepad2.right_stick_y);

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

                // left trigger gives you 100% on intake motor

                double intakePower = 0;
                if (gamepad1.left_trigger > 0.5) {
                    intakePower = 1.00;
                }
                intakeMotor.setPower(intakePower);

                // right trigger gives you 75% on launch motor

                double pollenLaunchPower = 0;
                if (gamepad1.right_trigger > 0.5) {
                    pollenLaunchPower = 0.60;
                }
                pollenLaunchMotor.setPower(pollenLaunchPower);

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

                // lift servo (xy/ab)
                // x and y for up and down (idk which is which)
                // b to stop
                // a to go to target
                if (buttonB && !pB) {
                    liftServo.setPwmDisable();
                }

                // If X is held, go up. Otherwise, return to 0
                if (!buttonB && buttonX) {
                    transferServoUpTime = runtime.seconds();
                    liftServo.setPosition(lift_end_percent);
                } else if (transferServoUpTime > 2.0) {
                    liftServo.setPosition(lift_start_percent);
                }

//                // Code to rotate the turret servos using gamepad2's sticks
//
//                // get target angle.
//
//                // I don't know what the "0" angle of the turret servo is
//                // so the target angle may need to be offset by a constant
//
//                double pollenAngle = Math.atan2(rx2, ry2);
//                pollenTurretController.setTarget(pollenAngle);
//
//                pollenTurretController.update();

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
                telemetry.addData("Intake power", "%.0f%%", intakePower * 100);

                telemetry.update();
            }
        } finally {
            tl.setPower(0);
            tr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            intakeMotor.setPower(0);
            pollenLaunchMotor.setPower(0);
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