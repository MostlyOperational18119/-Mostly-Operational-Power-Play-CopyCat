package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getCenterX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLargestObjectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLargestSize;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel1Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Capable;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2FullyAssigned;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel3Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevelString;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getPercentColor;

import static org.firstinspires.ftc.teamcode.Variables.*;
import static org.firstinspires.ftc.teamcode.Variables.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.Variables.Direction.FORWARD;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="VisionTeleopQT", group="A")
public class VisionTeleopQT extends DriveMethods {
    String level = "one";
    int levelCounter = 1;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;



    //The unit here is pixels
    int targetX;
    int targetWidth;
    double errorX;
    int errorWidth;
    double currentWidth;
    double dividerX = 500;
    double alignPowerAddedX;
    double alignPowerAddedWidth;
    int slidePosition = 0;
    int targetHeight = 0;
    double leftX;
    double leftY;
    double rightX;
    double addedDistance = 0;


    //The unit here is boxes


    @Override
    public void runOpMode() {

        initMotorsBlue();
        calibrateNavXIMU();
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");



        pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
        blinkinLedDriver.setPattern(pattern);



        /**
         * Camera initialization stuff below
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        PipePoleTracker pipePoleTracker = new PipePoleTracker(level);
        camera.setPipeline(pipePoleTracker);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        /**
         * Camera initialization stuff above
         */


        //INITIALIZATION
        waitForStart();
        //PLAY PRESSED





        double leftY;
        double leftX;
        double rightX;
        double speedDiv = 1.66;
        double aggressiveness = 3000;
        double holdingPower = 0.05;
        int slideTarget = 0;
        int slideDifference = 0;
        int targetHeight = 0;
        double sPosition = motorSlide.getCurrentPosition();
        boolean isManualControl = true;
        int coneStackHeight = 6;
        /**
         * Vision variables below, regular Teleop variables above
         */

        targetX = 225; 
        targetWidth = 15;
        level1Aligned = false;
        level2Aligned = false;
        level3Aligned = false;
        visionAutoActivated = false;
        slidePosition = motorSlide.getCurrentPosition();


        while (opModeIsActive()) {

            sPosition = motorSlide.getCurrentPosition();

            //update doubles
            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;


            if (gamepad2.x) {
                clawClamp();
            }
            if (gamepad2.a) {
                clawRelease();
            }

            if (!gamepad1.right_bumper) {
                motorFL.setPower((leftY + leftX + rightX) / speedDiv);
                motorBL.setPower((leftY - leftX + rightX) / speedDiv);
                motorFR.setPower((leftY - leftX - rightX) / speedDiv);
                motorBR.setPower((leftY + leftX - rightX) / speedDiv);
            } else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
            if (gamepad1.a) {
                speedDiv = 1.66;
            }
            if(gamepad1.b) {
                speedDiv = 2;
            }

            if(gamepad2.left_trigger==1) {
                isManualControl = false;
                coneStackHeight = 6;
                slideTarget = 1150;
                aggressiveness = 1200;
                holdingPower = 0.18;
            }
            if(gamepad2.right_trigger==1) {
                isManualControl = true;
                slideTarget = lowHeight;
                aggressiveness = 1000;
                holdingPower = 0.06;
                targetHeight = 2;
            }
            if((gamepad2.dpad_up || gamepad2.dpad_down) & isManualControl){
                if(gamepad2.dpad_up) {
                    targetHeight++;
                    if (targetHeight > 4) {
                        targetHeight = 4;
                    }
                    sleep(150);
                }
                if (gamepad2.dpad_down) {
                    targetHeight--;
                    if (targetHeight < 0) {
                        targetHeight = 0;
                    }
                    sleep(150);
                }
                switch (targetHeight) {
                    case 0:
                        slideTarget = downHeight;
                        aggressiveness = 2000;
                        holdingPower = 0.0;
                        break;
                    case 1:
                        slideTarget = collectHeight;
                        aggressiveness = 1000;
                        holdingPower = 0.06;
                        break;
                    case 2:
                        slideTarget = lowHeight;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;
                    case 3:
                        slideTarget = midHeight;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;
                    case 4:
                        slideTarget = highHeight;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;

                }
            }
            if((gamepad2.dpad_up || gamepad2.dpad_down) & !isManualControl) {
                if(gamepad2.dpad_up && coneStackHeight!=6) {
                    coneStackHeight++;
                    sleep(150);
                }
                if(gamepad2.dpad_down && coneStackHeight!=1) {
                    coneStackHeight--;
                    sleep(150);
                }

                switch (coneStackHeight) {
//                    case 0:
                    case 1:
                        slideTarget = 0;
                        aggressiveness = 1200;
                        holdingPower = 0;
                        break;
                    case 2:
                        slideTarget = /*1*/40; // Originally 190
                        aggressiveness = 1200;
                        holdingPower = 0.06;
                        break;
                    case 3:
                        slideTarget = 110;
                        aggressiveness = 1200;
                        holdingPower = 0.18;
                        break;
                    case 4:
                        slideTarget = 290;
                        aggressiveness = 1200;
                        holdingPower = 0.18;
                        break;
                    case 5:
                        slideTarget = 460;
                        aggressiveness = 1200;
                        holdingPower = 0.18;
                        break;
                    case 6:
                        slideTarget = 1150;
                        aggressiveness = 1200;
                        holdingPower = 0.18;
                        break;
//                    case 7:
//                        slideTarget = 1300;
//                        aggressiveness = 1800;
//                        holdingPower = 0.18;
//                        break;
                }
            }
            //Change the target height based on the height of the linear slide at the time.

            if(gamepad2.right_bumper){
                targetHeight = 4;
                sleep(50);
            }
            if(gamepad2.left_bumper){
                targetHeight = 0;
                sleep(50);
            }




            if(gamepad2.left_stick_y != 0){
                slideTarget += (int) -gamepad2.left_stick_y * 40;
                aggressiveness = 1250;
                sleep(50);
                if (sPosition<300 && sPosition>0){
                    targetHeight = 1;
                }
                if (sPosition<1900 && sPosition>300){
                    targetHeight = 2;
                }
                if (sPosition<3150 && sPosition>1900){
                    targetHeight = 3;
                }
                if (sPosition>3150 && sPosition<4300){
                    targetHeight = 4;
                }
            }
            if(slideTarget<0){
                slideTarget=0;
            }
            if(slideTarget>4400){
                slideTarget = 4400;
            }

//            if(motorSlide.getCurrentPosition()<150){
//                aggressiveness = 1750;
//            }

            if(slideTarget == 0 && motorSlide.getCurrentPosition() < 150 && motorSlide.getCurrentPosition() >= 50){
                aggressiveness = 700;
                holdingPower = 0;
            }

            if(slideTarget == 0 && motorSlide.getCurrentPosition() < 50){
                aggressiveness = 400;
                holdingPower = 0;
            }


            slideDifference = (slideTarget - Math.abs(motorSlide.getCurrentPosition()));

            motorSlide.setPower(((slideDifference / aggressiveness) + holdingPower));

            telemetry.addLine(slideDifference + "..difference");
            telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
            telemetry.addLine(slideTarget + "..target");
            telemetry.addLine(((slideDifference / aggressiveness) + holdingPower) + "..power");
            telemetry.addLine("Target Height: " + targetHeight);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.addLine("SpeedDiv: " + speedDiv);
            telemetry.addLine("linear slide position " + motorSlide.getCurrentPosition());
            telemetry.addLine("left_stick_y_2: " + gamepad2.left_stick_y);


            /**
             * Vision stuff below, regular Teleop above
             */



            if (level2Capable == false && visionAutoActivated == false) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(pattern);
            }
            if (level2Capable == true && visionAutoActivated == false) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }
            if (visionAutoActivated && levelCounter == 1 || levelCounter == 2) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }

            if (visionAutoActivated && levelCounter == 3) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                blinkinLedDriver.setPattern(pattern);
            }

            //Button triggers
            if (gamepad2.y && getLevel2Capable()) {
                visionAutoActivated = true;
            }


            if (gamepad2.b) {
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
                stopMotors();
                motorSlide.setPower(0);
            }

            //this means we are no longer looking at our object of choice
            if (levelCounter != 3 && getLargestSize() == 0) {
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
            }

            errorX = targetX - getCenterX();
            errorWidth = targetWidth - getLargestObjectWidth();

            dividerX = 200;


            if (visionAutoActivated) {

                alignPowerAddedX = errorX / dividerX;

                alignPowerAddedWidth = (double) errorWidth / 45;


                if (Math.abs(alignPowerAddedX) > 0.145) {
                    alignPowerAddedX = (errorX / (Math.abs(errorX))) * 0.145;
                }

                if (levelCounter == 1 && Math.abs(errorX) < 32) {//TODO will need to add distance condition
                    level1Aligned = true;
                    imuHeading = getCumulativeZ() + 1.5;
                    levelCounter = 2;
                    telemetry.addLine("level1 complete!");
                    telemetry.addLine("IMU Heading: " + imuHeading);
                    telemetry.addLine("errorX: " + errorX);
                    telemetry.addLine("errorX divide thingy: " + (errorX / (Math.abs(errorX))));

                    telemetry.update();
                    stopMotors();
                    //Robot is in front of pole well enough, entering level2...
                }

                if (levelCounter == 1 && level1Aligned == false) {

                    targetHeading = getCumulativeZ() + errorX*(0.04559197) + (0.007277*errorX) - 1; //The 0.045591... constant is derived from the width of camera view (angle) divided by the wide of the frame (pixels) to get degrees/pixel
                    telemetry.addLine("Target heading: " + targetHeading);
                    telemetry.addLine("Error heading: " + (errorX*(0.04559197)));
                    telemetry.addLine("Actual heading: " + getCumulativeZ());

                        rotateSmallWithBrake(targetHeading);

                }

                //Level2 below (untested at the moment - 1/17/23)
                if (levelCounter == 2 && getLevel2FullyAssigned() == true) {
                    rotateSmallWithBrake(imuHeading);
                    sleep(150);
                    currentWidth = getLargestObjectWidth();                                                             //5.2 is an error adjustment
                    targetDistance = (((640.0/(currentWidth*getBoxWidth()))*1.27)/(0.260284))-Math.pow(0.93,currentWidth-50) - 3 ; //This is the full distancefrom the pole in CENTImeters!
//                    telemetry.addLine("Target Distance: " + targetDistance + " cm");
//                    telemetry.addLine("Boxes Width: " + currentWidth);
//                    telemetry.addLine("Level2Assigment: " + getLevel2Assigment());
//                    telemetry.update();
//                    sleep(10000);
                    //After curve fitting, this is some simple double-read code
                    if(currentWidth < 25){
                        driveForDistanceBrake((targetDistance/100) - 0.25, FORWARD, 0.35, imuHeading);
                        level2Aligned = false;
                    }else if(currentWidth > 40){
                        driveForDistanceBrake(0.1, BACKWARD, 0.35, imuHeading);
                        level2Aligned = false;
                    }else{                                          // 1.5 is camera vs grabber differene, 15 is for reading height well
                        driveForDistanceBrake((targetDistance- 1.5 - 15)/100, FORWARD, 0.35, imuHeading);
                        level2Aligned = true;
                        levelCounter = 3;
                    }

                    telemetry.addLine("Target Distance: " + targetDistance + " cm");
                    telemetry.addLine("Boxes Width: " + currentWidth);


                }

                if (levelCounter == 3 && level3Assignment && getPercentColor() < 10) {
                    level3Aligned = true;
                    telemetry.addLine("We're at the top of the pole!");
                    telemetry.addLine("level3Aligned: " + level3Aligned);
                    telemetry.addLine("Percent Color: " + getPercentColor());
                    telemetry.update();


                }

                if (levelCounter == 3 && level3Aligned == false) {
                    clawClamp();
                    motorSlide.setPower(1);
                    slidePosition = motorSlide.getCurrentPosition();
                    telemetry.addLine("Measuring the pole height!");
                    telemetry.addLine("Slide Position: " + motorSlide.getCurrentPosition());
                    telemetry.addLine("Percent Color: " + getPercentColor());

                }

                //For all the marbles, this is the sequence that stacks
                if (level3Aligned == true) {
                    slidePosition = motorSlide.getCurrentPosition();
                    stopMotors();
                    telemetry.addLine("We going to the top babeeeeeeee");
                    telemetry.addLine("Slide position: " + slidePosition);
                    telemetry.addLine("targetHeight: " + targetHeight);
                    telemetry.update();
//                    sleep(500);
                    if (slidePosition >= 0 && slidePosition <= 1300) {
                        targetHeight = lowHeight;
                    } else if (slidePosition > 1300 && slidePosition <= 2500) {
                        targetHeight = midHeight;
                    } else if (slidePosition > 2500) {
                        targetHeight = highHeight;
                    }
                    if(targetHeight == lowHeight){
                        addedDistance = 0.03;
                    }else{
                        addedDistance = 0;
                    }

                    rotateSmallWithBrake(imuHeading);
                    clawClamp();
                    GoToHeight(targetHeight);
                    sleep(300);
                    driveForDistance(0.15 + addedDistance, FORWARD, 0.2, imuHeading);
//                    sleep(250);
                    GoToHeight(targetHeight - 60);
                    sleep(300);
                    clawRelease();
                    sleep(200);
                    GoToHeight(targetHeight);
                    sleep(250);
                    driveForDistance(0.15 + addedDistance, BACKWARD, 0.2, imuHeading);
                    goToDown();

                    levelCounter = 1;
                    level1Aligned = false;
                    level2Aligned = false;
                    level3Aligned = false;
                    visionAutoActivated = false;
                    targetX = 225; //TODO Avoid hard coding this value? Or maybe just take from the original resolution setting above
//                    dividerX = 200;
                    //Back to manual driving!!!

                }
            }


            if (levelCounter == 1) {
                level = "one";
            }

            if (levelCounter == 2) {
                level = "two";
            }

            if (levelCounter == 3) {
                level = "three";
            }

            telemetry.addLine("Target Distance: " + targetDistance);
            telemetry.addLine("Current Level: " + getLevelString());
            telemetry.addLine("Level 2 Capable?: " + getLevel2Capable());
            telemetry.addLine("Current Width (boxes): " + currentWidth);
            telemetry.addLine("errorX: " + errorX);//
            telemetry.addLine("Percent Col]or: " + getPercentColor());
            telemetry.addLine("Power Applied X: " + alignPowerAddedX);
            telemetry.addLine("Activated?: " + visionAutoActivated);
            telemetry.addLine("Level1Assigment: " + getLevel1Assigment());
            telemetry.addLine("Level2Assigment: " + getLevel2Assigment());
            telemetry.addLine("Level3Assigment: " + getLevel3Assigment());
            telemetry.update();

            pipePoleTracker = new PipePoleTracker(level);
            camera.setPipeline(pipePoleTracker);
        }
    }








    public void driveForDistanceBrake(double distanceMeters, Variables.Direction movementDirection, double power, double heading) { // distance: 2, strafe: false, power: 0.5
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
                break;
            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
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

        double currentZ = getCumulativeZ();
        double rotateError = targetZ - currentZ;

        double errorPosition = targetPos - avgPosition;
//        int brakeWindow = (int)(errorPosition/500)*100; //This might also be a way of managing the break window
        int brakeWindow = Math.abs((int)(power)*100);

//        double maxPower = Math.abs(errorPosition/500);
//        int reversalTime = (int)(errorPosition/500)*55;

        if(brakeWindow > 100){
            brakeWindow = 100;
        }

        while ((targetPos >= avgPosition)) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Math.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());

            currentZ = getCumulativeZ();
            rotateError = targetZ - currentZ;

            avgPosition = (int) (FLPosition + BLPosition + FRPosition + BRPosition) / 4;
            errorPosition = targetPos - avgPosition;

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

            if(errorPosition < brakeWindow){
                motorFL.setPower(-FLPower);
                motorBL.setPower(-BLPower);
                motorFR.setPower(-FRPower);
                motorBR.setPower(-BRPower);
                sleep(65);
                break;
            }
        }

        stopMotors();

    }
}
