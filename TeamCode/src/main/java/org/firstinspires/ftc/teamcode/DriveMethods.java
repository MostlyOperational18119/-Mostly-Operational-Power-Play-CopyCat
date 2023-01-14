package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static org.firstinspires.ftc.teamcode.Variables.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class DriveMethods extends LinearOpMode{

    @Override
    public void runOpMode() {}

    /*
    public void driveForDistance(double distanceMeters, boolean doStrafe, double power) { // distance: 2, strafe: false, power: 0.5
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) (distanceMeters * clicksPerRotation * rotationsPerMeter);
        motorFL.setTargetPosition((targetPos));
        motorBL.setTargetPosition((targetPos));
        motorFR.setTargetPosition((targetPos));
        motorBR.setTargetPosition((targetPos));



        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (doStrafe) {
            motorFL.setPower(power);
            motorBL.setPower(-power);
            motorFR.setPower(-power);
            motorBR.setPower(power);
        } else {
            motorFL.setPower(power);
            motorBL.setPower(power);
            motorFR.setPower(power);
            motorBR.setPower(power);
        }
        //targetPos = motorFL.getTargetPosition();
        int currentPos = motorFL.getCurrentPosition();
        int FLPosition;
        int BLPosition;
        int FRPosition;
        int BRPosition;
        int avgPosition = 0;
        boolean hasNotReachedTarget = true;
        while (targetPos >= avgPosition) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Mat
            h.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());
            avgPosition = (int)(FLPosition + BLPosition + FRPosition + BRPosition)/4;
            telemetry.addLine("Current Position: " + avgPosition);                      
            telemetry.addLine("targetPos " + targetPos);
            telemetry.update();
        }
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    */
    public void stopMotors() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    /*
    public void driveForDistance(double distanceMeters, Direction movementDirection, double power) { // distance: 2, strafe: false, power: 0.5
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) ((distanceMeters * clicksPerRotation * rotationsPerMeter)/1.15);
        motorFL.setTargetPosition((targetPos));
        motorBL.setTargetPosition((targetPos));
        motorFR.setTargetPosition((targetPos));
        motorBR.setTargetPosition((targetPos));



        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = Math.abs(power);
        switch(movementDirection) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;
            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;
            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(power);
                break;
            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
                break;
            case ROTATE_LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;
            case ROTATE_RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;
                
        }

        //if (doStrafe) {
        //    motorFL.setPower(power);
        //    motorBL.setPower(-power);
        //    motorFR.setPower(-power);
        //    motorBR.setPower(power);
        //} else {
        //    motorFL.setPower(power);
        //    motorBL.setPower(power);
        //    motorFR.setPower(power);
        //    motorBR.setPower(power);
        //}
        //targetPos = motorFL.getTargetPosition();

        int currentPos = motorFL.getCurrentPosition();
        int FLPosition;
        int BLPosition;
        int FRPosition;
        int BRPosition;
        int avgPosition = 0;
        boolean hasNotReachedTarget = true;
        while(targetPos >= avgPosition) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Math.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());
            avgPosition = (int)(FLPosition + BLPosition + FRPosition + BRPosition)/4;
            telemetry.addLine("Current Position: " + avgPosition);                      
            telemetry.addLine("targetPos " + targetPos);
            telemetry.update();
        }
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    */

    /**
     * This is a REV internal IMU calibration below
     */
//    public void CalibrateIMU() {
//
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//    parameters.loggingEnabled = true;
//    parameters.loggingTag = "IMU";
//    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//    // and named "imu".
//    imu = hardwareMap.get(BNO055IMU.class, "imu");
//    imu.initialize(parameters);
//
//    telemetry.addLine("imu should be calibrated!");
//    telemetry.update();
//    isImuCalibrated = true;
//    sleep(1000);
//
//
//    }


//    public double getCurrentZ() {
//        if(!isImuCalibrated){
//            CalibrateIMU();
//        }

//        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//        double currentZ = currentAngle.firstAngle;
//        return currentZ;
//    }



    /**
     * Above code is for REV internal IMU
     * BELOW is code for NavX IMU
     */

    public void calibrateNavXIMU(){
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;

        while (navxMicro.isCalibrating())  {
            telemetry.addLine("calibrating...");
            telemetry.update();
            sleep(50);
        }
        telemetry.addLine("calibrated!");
        telemetry.update();
    }

    public double getCurrentZ(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getCumulativeZ(){
        double currentHeading = getCurrentZ();
        double deltaHeading = currentHeading - previousHeading;
        if(deltaHeading <= -180) {
            deltaHeading += 360;
        } else if(deltaHeading >= 180) {
            deltaHeading -=360;
        }

        intergratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return intergratedHeading;

    }

    /**
     *Above is NavX IMU stuff
     **/
//    public void recenterRobotZRotation(double targetRotationZ) {
//        double FLPower = motorFL.getPower();
//        double BLPower = motorBL.getPower();
//        double FRPower = motorFR.getPower();
//        double BRPower = motorBR.getPower();
//        targetZ = getCurrentZ();
//        while(Math.floor(getCurrentZ()) != targetZ) {
//            double currentZ = getCurrentZ();
//            double rotateError = targetZ - currentZ;
//            //motorFL.setPower();
//            motorFL.setPower(FLPower - (rotateError / 100));
//            motorBL.setPower(BLPower - (rotateError / 100));
//            motorFR.setPower(FRPower + (rotateError / 100));
//            motorBR.setPower(BRPower + (rotateError / 100));
//        }
//    }


    public void driveForDistance(double distanceMeters, Direction movementDirection, double power, double heading) { // distance: 2, strafe: false, power: 0.5
        targetZ = heading;
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) ((distanceMeters * clicksPerRotation * rotationsPerMeter) / 1.15);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int doRotateOnly = 0;
        power = Math.abs(power);
        switch (movementDirection) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                //targetZ = 0;
                break;
            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                //targetZ = 0;
                break;
            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(power);
                //targetZ = getCurrentZ();
                break;
            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
                //targetZ = getCurrentZ();
                break;

        }
        /*
        if(rotateToTargetRotation) {
            targetZ = targetRotation;
        }
        */
        int currentPos = 0;
        int FLPosition;
        int BLPosition;
        int FRPosition;
        int BRPosition;
        int avgPosition = 0;
        double FLPower = motorFL.getPower();
        double BLPower = motorBL.getPower();
        double FRPower = motorFR.getPower();
        double BRPower = motorBR.getPower();

        double currentZ = getCurrentZ();
        double rotateError = targetZ - currentZ;

        while ((targetPos >= avgPosition)) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Math.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());

            currentZ = getCurrentZ();
            rotateError = targetZ - currentZ;

            avgPosition = (int) (FLPosition + BLPosition + FRPosition + BRPosition) / 4;
            motorFL.setPower(FLPower - (rotateError / 150));
                motorBL.setPower(BLPower - (rotateError / 150));
                motorFR.setPower(FRPower + (rotateError / 150));
                motorBR.setPower(BRPower + (rotateError / 150));

            telemetry.addLine("MotorFL Power " + motorFL.getPower());
            telemetry.addLine("MotorBL Power " + motorBL.getPower());
            telemetry.addLine("MotorFR Power " + motorFR.getPower());
            telemetry.addLine("MotorBR Power " + motorBR.getPower());

            telemetry.addLine("Current Position: " + avgPosition);
            telemetry.addLine("targetPos " + targetPos);

            telemetry.addLine("Cumulative Z " + getCumulativeZ());
            telemetry.addLine("Current Z " + getCurrentZ());
            telemetry.addLine("Error " + rotateError);
            telemetry.update();
        }

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

//    //This is a universal heading
//    public void rotateToHeading(int angleHeading){
//        telemetry.addLine("Trying to rotate!");
//        telemetry.update();
//        double target = angleHeading;
//        double current = getCumulativeZ();
//        double error = target- current;
//        double aggresivness = 100;
//
//        while(Math.abs(error) > 2) {
//            current = getCumulativeZ();
//            error = target - current;
//            if (Math.abs(error) > 5) {
//                aggresivness = 40;
//            } else {
//                aggresivness = 100;
//            }
//            motorFL.setPower(-(error / aggresivness)/* + 0.05*/);
//            motorBL.setPower(-(error / aggresivness)/* + 0.05*/);
//            motorFR.setPower((error / aggresivness)/* + 0.05*/);
//            motorBR.setPower((error / aggresivness)/* + 0.05*/);
//
//
//            telemetry.addLine("Current (Cumulative) Z:  " + current);
//            telemetry.addLine("Target Z: " + target);
//            telemetry.addLine("Error " + error);
//            telemetry.addLine("Power: " + (error/120));
//            telemetry.update();
//
//        }
//    }

    public void rotateAngle(int angle) {
        double target = angle;
        double error = target - getCumulativeZ();
        double power = 0;
        while (Math.abs(error) > 1) {
            error = target - getCumulativeZ();
           if (Math.abs(error) > 20) {
               power = error / 120;
           } else {
               power = error / 40 + .12;
           }
               motorFL.setPower(-power);
               motorBL.setPower(-power);
               motorFR.setPower(power);
               motorBR.setPower(power);
               telemetry.addLine("Current (Cumulative) Z:  " + getCumulativeZ());
               telemetry.addLine("Rotating power: " + power);
               //This is a universal heading
        }
    }
//    public void rotateToHeading ( int angleHeading, double power){
//        telemetry.addLine("Trying to rotate!");
//        telemetry.update();
//        double target = angleHeading;
//        double current = getCumulativeZ();
//        double error = target - current;
//        double aggresivness = 120;
//        globalTargetRotation = target;
//
//        while (Math.abs(error) > 2) {
//            current = getCumulativeZ();
//            error = target - current;
//
//            if (Math.abs(error) > 5) {
//                aggresivness = 60;
//            } else {
//                aggresivness = 120;
//            }
//            motorFL.setPower(-(error / aggresivness)/* + 0.05*/);
//            motorBL.setPower(-(error / aggresivness)/* + 0.05*/);
//            motorFR.setPower((error / aggresivness)/* + 0.05*/);
//            motorBR.setPower((error / aggresivness)/* + 0.05*/);
//
//
//            motorFL.setPower(((error / 120) + 0.05) * power);
//            motorBL.setPower(((error / 120) + 0.05) * power);
//            motorFR.setPower((-((error / 120) + 0.05)) * power);
//            motorBR.setPower((-((error / 120) + 0.05)) * power);
//
//            telemetry.addLine("Current (Cumulative) Z:  " + current);
//            telemetry.addLine("Target Z: " + target);
//            telemetry.addLine("Error " + error);
//            telemetry.addLine("Power: " + power);
//            telemetry.update();
//        }
//        while (Math.abs(error) > 1) {
//            error = target - getCumulativeZ();
//            power = (Math.abs(error) / error) * 0.15;
//            motorFL.setPower(-power);
//            motorBL.setPower(-power);
//            motorFR.setPower(power);
//            motorBR.setPower(power);
//            telemetry.addLine("Current (Cumulative) Z:  " + getCumulativeZ());
//            telemetry.addLine("Error " + error);
//            telemetry.addLine("Power: " + power);
//            telemetry.update();
//        }
//        stopMotors();
//        telemetry.addLine("Current (Cumulative) Z:  " + getCumulativeZ());
//        telemetry.addLine("Error " + error);
//        telemetry.addLine("Power: " + power);
//        telemetry.update();
//
//
//    }


            public void GoToHeight ( int Clicks){
                int target = (Clicks);
                int dif = (target - Math.abs(motorSlide.getCurrentPosition()));
                double aggressiveness = 2700;
                double holdingPower = 0;
                if (dif < 0) {
                    aggressiveness = 1500;
                    holdingPower = 0;
                }
                if (dif > 0) {
                    aggressiveness = 1600;
                    holdingPower = 0.18;
                }
                motorSlide.setPower((dif / aggressiveness));

                while (Math.abs(dif) >= 150) { // doesn't work when trying to go down
                    telemetry.addLine(dif + "..difference");
                    telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
                    telemetry.addLine(target + "..target");
                    telemetry.addLine(((dif / aggressiveness) + holdingPower) + "..power");
                    telemetry.update();
                    dif = (target - Math.abs(motorSlide.getCurrentPosition()));
                    motorSlide.setPower(((dif / aggressiveness) + holdingPower));
                    if (target == 0 && motorSlide.getCurrentPosition() < 150) {
                        aggressiveness = 250;
                    }
                }
                motorSlide.setPower(holdingPower);
            }

    public void GoToHeightFast(int Clicks1) {
        int target1 = (Clicks1);
        int dif1 = (target1 - Math.abs(motorSlide.getCurrentPosition()));
        double holdingPower1 = 0;
        if (dif1 < 0) {
            holdingPower1 = 0;
        } if (dif1 > 0) {
            holdingPower1 = 0.18;
            motorSlide.setPower(dif1);
        }
        motorSlide.setPower(holdingPower1);
    }

//    public void clawClamp(){
//        servoGrabberThing.setPosition(0.76);
//    }
//    public void clawRelease(){
//        servoGrabberThing.setPosition(0.66);
//    }
            public void clawClamp () {
                servoGrabberThing.setPosition(Clamp);
            }
            public void clawRelease () {
                servoGrabberThing.setPosition(Release);
            }
            public void goToDown () {
                GoToHeight(downHeight);
            }
            public void goToCollect () {
                GoToHeight(collectHeight);
            }
            public void goToLow () {
                GoToHeight(lowHeight);
            }
            public void goToMid () {
                GoToHeight(midHeight);
            }
            public void goToHigh () {
                GoToHeight(highHeight);
            }
            public void goToFifth () {
                GoToHeight(fifthHeight);
            }
            public void goToFourth () {
                GoToHeight(fourthHeight);
            }


            public void initMotorsSecondBot () {
                motorFL = hardwareMap.get(DcMotor.class, "motorFL");
                motorBL = hardwareMap.get(DcMotor.class, "motorBL");
                motorFR = hardwareMap.get(DcMotor.class, "motorFR");
                motorBR = hardwareMap.get(DcMotor.class, "motorBR");

                motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
                motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            public void initMotorsBlue () {
                motorFL = hardwareMap.get(DcMotor.class, "motorFL");
                motorBL = hardwareMap.get(DcMotor.class, "motorBL");
                motorFR = hardwareMap.get(DcMotor.class, "motorFR");
                motorBR = hardwareMap.get(DcMotor.class, "motorBR");
                motorSlide = hardwareMap.get(DcMotor.class, "motorLS");
                servoGrabberThing = hardwareMap.get(Servo.class, "grabber");

                motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
                calibrateNavXIMU();
            }
            public void initMotorsBlueBlinkin () {
                motorFL = hardwareMap.get(DcMotor.class, "motorFL");
                motorBL = hardwareMap.get(DcMotor.class, "motorBL");
                motorFR = hardwareMap.get(DcMotor.class, "motorFR");
                motorBR = hardwareMap.get(DcMotor.class, "motorBR");
                motorSlide = hardwareMap.get(DcMotor.class, "motorLS");
                servoGrabberThing = hardwareMap.get(Servo.class, "grabber");

                blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

                motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
                calibrateNavXIMU();

                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                blinkinLedDriver.setPattern(pattern);
            }
            public void initBlinkinOnly () {
                blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                blinkinLedDriver.setPattern(pattern);
            }
            public void setBlinkinColor (BlinkinColor colorEnum){
                switch (colorEnum) {
                    case RAINBOW:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                        break;
                    case RED:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                        break;
                    case ORANGE:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                        break;
                    case GREEN:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                        break;
                    case YELLOW:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        break;
                    case BLUE:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                        break;
                    case PURPLE:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                        break;
                    case PINK:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                        break;
                    case GREEN_PULSE:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
                        break;
                    case ORANGE_PULSE:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;
                    case RED_PULSE:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                }
                sleep(1000);
                blinkinLedDriver.setPattern(pattern);
            }
        }
