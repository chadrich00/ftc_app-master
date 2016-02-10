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
 * Example autonomous program.
 * <p>
 * This example program uses elapsed time to determine how to move the robot.
 * The OpMode.java class has some class members that provide time information
 * for the current op mode.
 * The public member variable 'time' is updated before each call to the run() event.
 * The method getRunTime() returns the time that has elapsed since the op mode
 * starting running to when the method was called.
 */
public class AutonomousBlue1 extends OpMode {

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

	double leftPower;
	double rightPower;
	double sweepPower;
	double armPower;
	double liftPower;

	double counter;

	/**
	 * Constructor
	 */
	public AutonomousBlue1() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

		/*
		 * Use the hardwareMap to get the dc motors and servos by name.
		 * Note that the names of the devices must match the names used
		 * when you configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot..
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		motorRight = hardwareMap.dcMotor.get("right_drive");
		motorLeft = hardwareMap.dcMotor.get("left_drive");
		motorSweep = hardwareMap.dcMotor.get("sweep_motor");
		motorTape = hardwareMap.dcMotor.get("spool_motor");
		motorArm = hardwareMap.dcMotor.get("arm_motor");
		motorLift = hardwareMap.dcMotor.get("lift_motor");

		motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorLeft.setDirection(DcMotor.Direction.REVERSE);
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

		counter = 0;
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		if (counter < 1) {
			resetStartTime();
		}
        /*
         * Use the 'time' variable of this op mode to determine
         * how to adjust the motor power.
         */
		if (this.time <= 6) {
			// from 0 to 1 seconds, run the motors for five seconds.
			leftPower = 1;
			rightPower = 0.1;
			sweepPower = 1;
			armPower = 0.0;
			liftPower = 0.0;
			switchLeftPosition = SWITCH_LEFT_MAX;
			switchRightPosition = SWITCH_RIGHT_MIN;
			getRuntime();
		} else if (this.time > 6 && this.time <= 7.1) {
			//between 5 and 8.5 seconds, point turn right.
			leftPower = 1;
			rightPower = -1;
			sweepPower = -1;
			armPower = 0.0;
			liftPower = 0.0;
			getRuntime();
		} else if (this.time > 7.1 && this.time <= 11.1) {
			// between 8 and 15 seconds, idle.
			leftPower = 1;
			rightPower = 1;
			sweepPower = 1;
			armPower = 0.0;
			liftPower = 0.0;
			getRuntime();
		}
		else if (this.time > 11.1 && this.time <= 14) {
			// between 15 and 20.75 seconds, point turn left.
			leftPower = 0.0;
			rightPower = 0.0;
			sweepPower = 0.0;
			armPower = -0.25;
			liftPower = 1;
			getRuntime();
		}
		else {
			//after 20.75 seconds, stop.
			leftPower = 0.0;
			rightPower = 0.0;
			sweepPower = 0.0;
			armPower = 0.0;
			liftPower = 0.0;
			getRuntime();
		}

		/*
		 * set the motor power
		 */
		motorRight.setPower(rightPower);
		motorLeft.setPower(leftPower);
		motorSweep.setPower(sweepPower);
		motorArm.setPower(armPower);
		motorLift.setPower(liftPower);

		counter += 1;

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
		telemetry.addData("left tgt pwr",  "left  pwr: " + Double.toString(leftPower));
		telemetry.addData("right tgt pwr", "right pwr: " + Double.toString(rightPower));
		telemetry.addData("sweepPower", "sweepPower: " + Double.toString(sweepPower));
		telemetry.addData("armPower", "armPower: " + Double.toString(armPower));
		telemetry.addData("liftPower", "liftPower: " + Double.toString(liftPower));
		telemetry.addData("switchLeftPosition", "switchLeftPosition:  " + String.format("%.2f", switchLeftPosition));
		telemetry.addData("switchRightPosition", "switchRightPosition:  " + String.format("%.2f", switchRightPosition));
		telemetry.addData("bucketLeftPosition", "bucketLeftPosition:  " + String.format("%.2f", bucketLeftPosition));
		telemetry.addData("bucketRightPosition", "bucketRightPosition:  " + String.format("%.2f", bucketRightPosition));
		telemetry.addData("tapeAnglePosition", "tapeAnglePosition:  " + String.format("%.2f", tapeAnglePosition));
		telemetry.addData("rampPosition", "rampPosition:  " + String.format("%.2f", rampPosition));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}
}


