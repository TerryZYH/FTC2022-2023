/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto19759_3", group="Robot")

public class Auto extends LinearOpMode {
    //
    NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    //28 * 20 / (2ppi * 4.125)
    double width = 16.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    double diameter = 4.125;
    double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    double bias = 0.7;//default 0.8
    double meccyBias = 0.9;//change to adjust only strafing movement
    //
    double conversion = cpi * bias;
    Boolean exit = false;

    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private DcMotorEx liftMotor = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private DcMotorEx yuntai = null;

    private int liftHighPos = -2200;
    private int liftMidPos = -1520;
    private int liftLowPos = -900;
    private int liftGrabPos = -700;
    private int liftRestPos = 0;
    private double lastTime = 0;

    private int autoPos;

    //
    public void runOpMode(){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        //
        initGyro();
        //
        frontleft = hardwareMap.dcMotor.get("lf");
        frontright = hardwareMap.dcMotor.get("rf");
        backleft = hardwareMap.dcMotor.get("lb");
        backright = hardwareMap.dcMotor.get("rb");

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        yuntai = hardwareMap.get(DcMotorEx.class, "yuntai");

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        yuntai.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yuntai.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawClose();
        waitForStartify();
        colorSensorInit();        //

        // smoothMove(0, 50, 5, 5, 1);
        // smoothMove(50, 0, 5, 5, 1);

        putPreload();

    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void encoderInspect(){
        while (true){
            telemetry.addData("backleft", backleft.getCurrentPosition());
            telemetry.addData("frontleft", frontleft.getCurrentPosition());
            telemetry.addData("backright", backright.getCurrentPosition());
            telemetry.addData("frontright", frontright.getCurrentPosition());
            telemetry.update();
        }
    }

    public void linearAcc(double timeDuration){
        double timeStart = time;
        double timeCurrent = time;
        double power;
        while ((timeCurrent - timeStart) < timeDuration){
            power = (timeCurrent - timeStart) / timeDuration;
        }
    }

    public void linearDec(double timeDuration){

    }

    public void smoothMove(double vStart, double vTarget, double flexible, double tMax, double speed){

        double baseTime = time;
        int blp = backleft.getCurrentPosition();
        int flp = frontleft.getCurrentPosition();
        int brp = backright.getCurrentPosition();
        int frp = frontright.getCurrentPosition();

        // double vStart = 0;
        // double vTarget = 5;
        // double flexible = 3;
        // double tMax = 10;

        double d = 0;
        double currentTime = time;
        while (currentTime - baseTime < tMax){
            double dt = currentTime - lastTime;
            double v = vStart + (vTarget - vStart) / (1 + Math.pow(Math.E, - flexible * (2 * currentTime - tMax) / tMax));
            d += v * dt;

            int move = (int)(Math.round(d * conversion));

            backleft.setTargetPosition(blp + move);
            frontleft.setTargetPosition(flp + move);
            backright.setTargetPosition(brp + move);
            frontright.setTargetPosition(frp + move);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            backleft.setPower(speed);
            frontright.setPower(speed);
            backright.setPower(speed);
            //
            lastTime = currentTime;
        }

    }

    //移动
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }


    //移动
    public void routateToPosition(int inches, double speed){
        yuntai.setPower(speed);
        yuntai.setTargetPosition(inches);
        //
        yuntai.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        return;
    }
    //转弯
    public void turnTo(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    //横移
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify(){
        waitForStart();
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }

    public void liftPosition(int target,double speed){
        liftMotor.setPower(speed);
        liftMotor.setTargetPosition(target);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void clawOpen(){
        leftClaw.setPosition(0.7);
        rightClaw.setPosition(0.3);
    }

    public void clawClose(){
        leftClaw.setPosition(0.98);
        rightClaw.setPosition(0.02);
    }

    public void colorSensorInit(){
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.update();
    }

    public void getAutoPos(){
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[0] < 60){
            // red
            autoPos = 1;
        }else if(hsvValues[0] < 180){
            // green
            autoPos = 2;
        }else{
            // blue
            autoPos = 3;
        }
        telemetry.addData("AutoPos", autoPos);
        telemetry.update();
    }

    public void gotoAutoPos(){
        if (autoPos == 1){
            // 1 号区域
            liftPosition(liftLowPos,0.7);
            moveToPosition(-20, 0.8);
            sleep(200);
            liftPosition(liftRestPos,-0.3);
            sleep(200);
            turnTo(24,0.5);
            sleep(200);
            liftPosition(liftRestPos,-0.3);
            sleep(200);
            liftPosition(liftRestPos,0.7);
            sleep(200);
            moveToPosition(33, 0.6);
            sleep(200);
            clawOpen();
            sleep(200);
            liftPosition(-350,0.7);
            sleep(200);
            moveToPosition(5, 0.9);
            sleep(200);
            clawClose();
            sleep(800);
            liftPosition(liftGrabPos,0.7);
            sleep(200);
            moveToPosition(-5, 0.9);
            sleep(200);
            turnTo(-24,0.5);
            sleep(200);
            moveToPosition(5, 0.9);
            sleep(200);
            strafeToPosition(5, 0.4);
            sleep(200);
            moveToPosition(23, 0.9);
            sleep(200);
            liftPosition(0,0.7);
            sleep(200);
            clawOpen();
            sleep(200);
            liftPosition(-350,0.7);
            sleep(200);
            moveToPosition(-26, 0.9);
            sleep(200);
            strafeToPosition(-68, 0.7);
            sleep(200);
            liftPosition(liftRestPos,0.7);
            sleep(200);
        }else if(autoPos == 2){
            // 2号区域
            moveToPosition(-10, 0.9);
            sleep(300);
            turnTo(24,0.5);
            sleep(500);
            moveToPosition(33, 0.9);
            sleep(300);
            clawOpen();
            sleep(500);
            liftPosition(-350,0.7);
            sleep(300);
            moveToPosition(5, 0.9);
            sleep(300);
            clawClose();
            sleep(1000);
            liftPosition(liftGrabPos,0.7);
            sleep(300);
            moveToPosition(-5, 0.9);
            sleep(300);
            turnTo(-24,0.5);
            sleep(300);
            moveToPosition(5, 0.9);
            sleep(300);
            strafeToPosition(5, 0.4);
            sleep(300);
            moveToPosition(23, 0.9);
            sleep(300);
            liftPosition(0,0.7);
            sleep(300);
            clawOpen();
            sleep(300);
            liftPosition(-350,0.7);
            sleep(300);
            moveToPosition(-24, 0.9);
            sleep(300);
            strafeToPosition(-30, 0.4);
            sleep(300);
            liftPosition(liftRestPos,0.7);
            sleep(300);
        }else{
            // 3号区域
            moveToPosition(-10, 0.9);
            sleep(300);
            turnTo(24,0.5);
            sleep(500);
            moveToPosition(33, 0.9);
            sleep(300);
            clawOpen();
            sleep(500);
            liftPosition(-350,0.7);
            sleep(300);
            moveToPosition(5, 0.9);
            sleep(300);
            clawClose();
            sleep(1000);
            liftPosition(liftGrabPos,0.7);
            sleep(300);
            moveToPosition(-5, 0.9);
            sleep(300);
            turnTo(-24,0.5);
            sleep(300);
            moveToPosition(5, 0.9);
            sleep(300);
            strafeToPosition(5, 0.4);
            sleep(300);
            moveToPosition(23, 0.9);
            sleep(300);
            liftPosition(0,0.7);
            sleep(300);
            clawOpen();
            sleep(300);
            liftPosition(-350,0.7);
            sleep(300);
            moveToPosition(-24, 0.9);
            sleep(300);
            liftPosition(liftRestPos,0.7);
            sleep(300);
        }
    }

    public void putPreload(){
        // clawClose();
        // sleep(600);
        // while(true){
        //     telemetry.addData("time", time);
        //     telemetry.update();
        // }


        // 启动
        liftPosition(liftGrabPos,0.6);
        moveToPosition(58, 0.4);

        // 测锥桶颜色
        getAutoPos();
        sleep(300);

        // 放置第一个中位锥桶
        liftPosition(liftMidPos, 0.6);
        routateToPosition(-260, 0.2);
        strafeToPosition(-6, 0.8);
        sleep(300);
        clawOpen();
        sleep(500);


        // 推颜色指示锥桶
        routateToPosition(0, 0.3);
        sleep(200);
        liftPosition(liftLowPos,0.6);
        moveToPosition(35, 0.4);
        sleep(500);

        // 位移到取锥桶位置
        liftPosition(-300,0.5);
        moveToPosition(-15, 0.4);
        sleep(500);
        turnTo(24,0.3);
        sleep(500);
        moveToPosition(38, 0.4);

        // 抓取锥桶
        clawClose();
        sleep(500);
        liftPosition(liftLowPos,0.6);
        sleep(500);

        // 从取锥桶位置去放右上角放置锥桶
        moveToPosition(-10, 0.5);
        sleep(200);
        turnTo(-15,0.4);
        sleep(200);
        liftPosition(-100,0.7);
        moveToPosition(20, 0.4);
        clawOpen();
        sleep(200);

        // 回到取锥桶位置
        moveToPosition(-20, 0.4);
        sleep(500);
        liftPosition(-250,0.7);
        turnTo(15,0.4);
        sleep(200);
        moveToPosition(10, 0.5);

        // 抓取锥桶
        clawClose();
        sleep(500);
        liftPosition(liftLowPos, 0.6);
        sleep(500);

        // 根据位置指示锥桶 位移到相应位置
        if (autoPos == 1){
            moveToPosition(-70, 0.4);
        }else if(autoPos == 2){
            moveToPosition(-30, 0.4);
        }else{
            moveToPosition(-5, 0.4);
        }
        sleep(500);
        turnTo(24,0.4);
        sleep(500);

        // 放下锥桶
        liftPosition(0, 0.6);
        sleep(1500);

    }

    public void takeTopCone(){
        liftPosition(-400,0.7);
        sleep(800);
        moveToPosition(4, 0.4);
        sleep(400);
        clawClose();
        sleep(800);
        liftPosition(-1000,0.7);
        sleep(700);
    }
}
