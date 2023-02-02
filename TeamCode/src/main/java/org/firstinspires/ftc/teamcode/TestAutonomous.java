package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.clicksPerRotation;
import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;
import static org.firstinspires.ftc.teamcode.Variables.rotationsPerMeter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestAutonomous", group = "A")
public class TestAutonomous extends DriveMethods {
    double angle = 0;

    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();

        waitForStart();

        driveForDistanceBrake(0.75, Variables.Direction.FORWARD, 0.6, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.BACKWARD, 0.6, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.FORWARD, 0.4, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.BACKWARD, 0.4, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.FORWARD, 0.3, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.BACKWARD, 0.3, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.FORWARD, 0.2, 0);
        sleep(5000);
        driveForDistanceBrake(0.75, Variables.Direction.BACKWARD, 0.2, 0);
        sleep(5000);


        while (opModeIsActive()) {


        }
    }


    public void rotateWithBrake(double heading) {
        double target = heading;
        double current = getCumulativeZ();
        double error = target - current;
        double power = 0;
        int sign = 0;

        while (Math.abs(error) > 1) {
            current = getCumulativeZ();
            error = target - current;
            sign = (int) (error / (Math.abs(error))); //This equals 1, which will be used for full power

//
//            power = error / 25 + 0.15;

            motorFL.setPower(-sign);
            motorBL.setPower(-sign);
            motorFR.setPower(sign);
            motorBR.setPower(sign);

            telemetry.addLine("Target: " + target);
            telemetry.addLine("Current: " + current);
            telemetry.addLine("Error: " + error);
            telemetry.update();

            if (Math.abs(error) < 19) {
                sign = (int) (error / (Math.abs(error))); //This equals 1, which will be used for full power
                motorFL.setPower(sign);
                motorBL.setPower(sign);
                motorFR.setPower(-sign);
                motorBR.setPower(-sign);
                sleep(55);
                stopMotors();
                break;
            }

        }

        target = angle;
        error = target - getCumulativeZ();
        power = 0;
        while (Math.abs(error) > 1) {
            error = target - getCumulativeZ();
            if (Math.abs(error) > 20) {
                power = error / 120;
            } else {
                power = error / 40 + .15 * (Math.abs(error) / error);
            }
            motorFL.setPower(-power);
            motorBL.setPower(-power);
            motorFR.setPower(power);
            motorBR.setPower(power);
            telemetry.addLine("Current (Cumulative) Z:  " + getCumulativeZ());
            telemetry.addLine("Rotating power: " + power);
            telemetry.update();
            //This is a universal heading
        }
        stopMotors();
    }


    public void driveForDistanceBrake(double distanceMeters, Variables.Direction movementDirection, double power, double heading) { // distance: 2, strafe: false, power: 0.5
        double targetZ = heading;
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) ((distanceMeters * clicksPerRotation * rotationsPerMeter) * 1.1 / 1.15);

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

            if (gamepad2.a) {
                telemetry.addLine("Breaking out of drive for distance command!");
                telemetry.update();
                break;
            }

            if(power > 0.5 && Math.abs(errorPosition) < 125){
                FLPower = motorFL.getPower() / (Math.abs(motorFL.getPower()));
                BLPower = motorBL.getPower() / (Math.abs(motorBL.getPower()));
                FRPower = motorFR.getPower() / (Math.abs(motorFR.getPower()));
                BRPower = motorBR.getPower() / (Math.abs(motorBR.getPower()));

                motorFL.setPower(-FLPower * power * 2);
                motorBL.setPower(-BLPower * power * 2);
                motorFR.setPower(-FRPower * power * 2);
                motorBR.setPower(-BRPower * power * 2);
                sleep(50);
                break;
            }

            if(power >= 0.3 && Math.abs(errorPosition) < 70){
                FLPower = motorFL.getPower() / (Math.abs(motorFL.getPower()));
                BLPower = motorBL.getPower() / (Math.abs(motorBL.getPower()));
                FRPower = motorFR.getPower() / (Math.abs(motorFR.getPower()));
                BRPower = motorBR.getPower() / (Math.abs(motorBR.getPower()));

                motorFL.setPower(-FLPower * power * 2);
                motorBL.setPower(-BLPower * power * 2);
                motorFR.setPower(-FRPower * power * 2);
                motorBR.setPower(-BRPower * power * 2);
                sleep(40);
                break;
            }
        }

        stopMotors();

    }


}



