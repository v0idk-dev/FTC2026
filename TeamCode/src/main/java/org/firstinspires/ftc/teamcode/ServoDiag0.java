package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name = "Port0 Servo Diagnose", group = "2STest")
public class ServoDiag0 extends LinearOpMode {

	@Override
	public void runOpMode() {
		// grab port 0 no matter if config says servo or cr
		ServoControllerEx c = null;
		String info = "";
		int n = 0;
		for (Servo s : hardwareMap.getAll(Servo.class)) {
			if (s.getPortNumber() != 0) continue;
			c = (ServoControllerEx) s.getController();
			info += "servo on " + s.getConnectionInfo() + "\n";
			n++;
		}
		for (CRServo s : hardwareMap.getAll(CRServo.class)) {
			if (s.getPortNumber() != 0) continue;
			c = (ServoControllerEx) s.getController();
			info += "cr servo on " + s.getConnectionInfo() + "\n";
			n++;
		}

		if (c == null) {
			telemetry.addLine("nothing configured on port 0");
			telemetry.update();
			waitForStart();
			return;
		}

		telemetry.addData("found on port 0", n);
		telemetry.addLine(info);
		if (n > 1) telemetry.addLine("WARNING: more than one hub has port 0 configured");
		telemetry.addLine("press start, servo will swing back and forth");
		telemetry.update();
		waitForStart();

		// raw pulses, skips all the degree math
		c.setServoPwmRange(0, new PwmControl.PwmRange(500, 2500));
		c.setServoPwmEnable(0);

		boolean hi = false;
		while (opModeIsActive()) {
			int us = hi ? 1800 : 1200;
			c.setServoPosition(0, (us - 500) / 2000.0);

			telemetry.addData("pulse (us)", us);
			telemetry.addLine(info);
			telemetry.addLine("swings ~540 deg and stops = positional mode");
			telemetry.addLine("spins one way then the other = continuous mode");
			telemetry.addLine("nothing = power/signal/servo problem");
			telemetry.update();

			hi = !hi;
			sleep(2500);
		}
		c.setServoPwmDisable(0);
	}
}