/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Robo_Hodags_TeleOp extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.
	DcMotor motorRight = null;
	DcMotor motorLeft = null;
	DcMotor motorSweep = null;
	DcMotor motorTape = null;
	DcMotor motorArm = null;
	DcMotor motorLift = null;

	Servo bucketRight = null;
	Servo bucketLeft = null;
	Servo switchRight = null;
	Servo switchLeft = null;
	Servo tapeAngle = null;
	Servo ramp = null;

	final double BUCKET_RIGHT_MIN = 0.55;
	final double BUCKET_RIGHT_MAX = 1;
	final double BUCKET_LEFT_MIN = 0.07;
	final double BUCKET_LEFT_MAX = 0.475;
	final double SWITCH_RIGHT_MIN = 0.2;
	final double SWITCH_RIGHT_MAX = 0.9;
	final double SWITCH_LEFT_MIN = 0.1;
	final double SWITCH_LEFT_MAX = 0.75;
	final double TAPE_ANGLE_MIN = 0.1;
	final double TAPE_ANGLE_MAX = 0.8;
	final double RAMP_MIN = 0.3;
	final double RAMP_MAX = 0.65;

	double switchRightPosition;
	double switchLeftPosition;
	double bucketLeftPosition;
	double bucketRightPosition;
	double tapeAnglePosition;
	double rampPosition;

	double sweepPower;
	double motorArmPower;
	double motorLiftPower;


	/**
	 * Constructor
	 */
	public Robo_Hodags_TeleOp() {
	}

	@Override
	public void init() {
		motorRight = hardwareMap.dcMotor.get("right_drive");
		motorLeft = hardwareMap.dcMotor.get("left_drive");
		motorSweep = hardwareMap.dcMotor.get("sweep_motor");
		motorTape = hardwareMap.dcMotor.get("spool_motor");
		motorArm = hardwareMap.dcMotor.get("arm_motor");
		motorLift = hardwareMap.dcMotor.get("lift_motor");

		motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorRight.setDirection(DcMotor.Direction.REVERSE);
		motorSweep.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorTape.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorTape.setDirection(DcMotor.Direction.REVERSE);
		motorArm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorLift.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

		bucketRight = hardwareMap.servo.get("right_bucket");
		bucketLeft = hardwareMap.servo.get("left_bucket");
		switchRight = hardwareMap.servo.get("right_switch");
		switchLeft = hardwareMap.servo.get("left_switch");
		tapeAngle = hardwareMap.servo.get("tape_angle");
		ramp = hardwareMap.servo.get("ramp");

		bucketRight.setPosition(BUCKET_RIGHT_MIN);
		bucketLeft.setPosition(BUCKET_LEFT_MAX);
		switchRight.setPosition(SWITCH_RIGHT_MAX);
		switchLeft.setPosition(SWITCH_LEFT_MIN);
		tapeAngle.setPosition(TAPE_ANGLE_MAX);
		ramp.setPosition(RAMP_MAX);
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		float leftPower = gamepad1.left_stick_y;
		float rightPower = gamepad1.right_stick_y;
		motorRight.setPower(rightPower);
		motorLeft.setPower(leftPower);
		if (gamepad1.dpad_up) {
			switchLeftPosition = SWITCH_LEFT_MAX;
			switchLeft.setPosition(switchLeftPosition);
		}
		if (gamepad1.dpad_left) {
			switchLeftPosition = 0.35;
			switchLeft.setPosition(switchLeftPosition);
		}
		if (gamepad1.dpad_down) {
			switchLeftPosition = SWITCH_LEFT_MIN;
			switchLeft.setPosition(switchLeftPosition);
		}
		if (gamepad1.y) {
			switchRightPosition = SWITCH_RIGHT_MIN;
			switchRight.setPosition(switchRightPosition);
		}
		if (gamepad1.b) {
			switchRightPosition = 0.6;
			switchRight.setPosition(switchRightPosition);
		}
		if (gamepad1.a) {
			switchRightPosition = SWITCH_RIGHT_MAX;
			switchRight.setPosition(switchRightPosition);
		}

		float tapePower = gamepad2.left_stick_y;
		float armPower = gamepad2.right_stick_y;
		float liftPower = gamepad2.right_stick_x;
		motorTape.setPower(tapePower);
		motorArm.setPower(armPower);
		motorLift.setPower(liftPower);
		if (gamepad2.y) {
			rampPosition = RAMP_MAX;
			ramp.setPosition(rampPosition);
			sweepPower = 1;
			motorSweep.setPower(sweepPower);
		}
		if (gamepad2.b) {
			sweepPower = 0;
			motorSweep.setPower(sweepPower);
		}
		if (gamepad2.a) {
			rampPosition = RAMP_MAX;
			ramp.setPosition(rampPosition);
			sweepPower = -0.5;
			motorSweep.setPower(sweepPower);
		}
		if (gamepad2.left_bumper) {
			bucketLeftPosition = BUCKET_LEFT_MIN;
			bucketLeft.setPosition(bucketLeftPosition);
		}
		if (gamepad2.left_trigger > 0.5) {
			bucketLeftPosition = BUCKET_LEFT_MAX;
			bucketLeft.setPosition(bucketLeftPosition);
		}
		if (gamepad2.right_bumper) {
			bucketRightPosition = BUCKET_RIGHT_MAX;
			bucketRight.setPosition(bucketRightPosition);
		}
		if (gamepad2.right_trigger > 0.5) {
			bucketRightPosition = BUCKET_RIGHT_MIN;
			bucketRight.setPosition(bucketRightPosition);
		}
		if (gamepad2.dpad_up) {
			tapeAnglePosition = TAPE_ANGLE_MIN;
			tapeAngle.setPosition(tapeAnglePosition);
		}
		if (gamepad2.dpad_down) {
			tapeAnglePosition = TAPE_ANGLE_MAX;
			tapeAngle.setPosition(tapeAnglePosition);
		}
		if (gamepad2.dpad_left) {
			rampPosition = RAMP_MIN;
			ramp.setPosition(rampPosition);
		}
		if (gamepad2.dpad_right) {
			rampPosition = RAMP_MAX;
			ramp.setPosition(rampPosition);
		}
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("switchLeftPosition", "switchLeftPosition:  " + String.format("%.2f", switchLeftPosition));
        telemetry.addData("switchRightPosition", "switchRightPosition:  " + String.format("%.2f", switchRightPosition));
		telemetry.addData("bucketLeftPosition", "bucketLeftPosition:  " + String.format("%.2f", bucketLeftPosition));
		telemetry.addData("bucketRightPosition", "bucketRightPosition:  " + String.format("%.2f", bucketRightPosition));
		telemetry.addData("tapeAnglePosition", "tapeAnglePosition:  " + String.format("%.2f", tapeAnglePosition));
		telemetry.addData("rampPosition", "rampPosition:  " + String.format("%.2f", rampPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", leftPower));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", rightPower));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", sweepPower));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", motorArmPower));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", motorLiftPower));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
