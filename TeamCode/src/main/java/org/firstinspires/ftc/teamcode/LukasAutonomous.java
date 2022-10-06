package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Variables.*;

import android.graphics.drawable.GradientDrawable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="LukasAuto", group = "A")

public class LukasAutonomous extends DriveMethods {


    boolean imuIsCalibrated = false;
    double previousHeading = 0;
    double integratedHeading = 0;
    BNO055IMU imu;

    @Override
    public void runOpMode() {


        waitForStart();


        while (opModeIsActive()) {

        }
    }


    public enum Direction {
        FORWARD,
        BACKWARD,
        ROTATE_LEFT,
        ROTATE_RIGHT,
        RIGHT,
        LEFT,
    }

    public void driveDirection(Direction direction, double power) {
        switch (direction) {
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


    }


    public void driveForTime(int seconds, double power, Direction direction) { // seconds: 10, power: 1, direction: FORWARD

        driveDirection(direction, power);

        sleep(seconds * 1000);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }


    public void calibrateIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        imuIsCalibrated = true;

    }

    public double getCurrentZ() {
        if(imuIsCalibrated == false){
            calibrateIMU();
        }

        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double currentZ = currentAngle.firstAngle;
        return currentZ;

    }

    public double getCumulativeZ(){
        double currentHeading = getCurrentZ();
        double deltaHeading = currentHeading - previousHeading;

        if(deltaHeading <= -180){
            deltaHeading +=360;
        }else if( deltaHeading >= 180){
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return  integratedHeading;
    }
}
