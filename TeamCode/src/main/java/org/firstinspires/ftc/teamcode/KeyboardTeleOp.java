package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.json.JSONException;
import org.json.JSONObject;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;


import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;

@TeleOp(name="KeyboardTeleOp")
public class KeyboardTeleOp extends DriveMethods {
    private HashMap<String, Boolean> getPressedKeysMap() throws JSONException {
        JSONObject reader = new JSONObject(Variables.curJSONdata);
        HashMap keysMap = new Gson().fromJson(String.valueOf(reader), HashMap.class);
        HashMap<String, Boolean> parsedJson = new HashMap();

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            keysMap.forEach((key,val) -> {
                if(key.toString().length()==1) {
                    if (val.toString() == "true") {
                        parsedJson.put((String) key, true);
                    } else if (val.toString() == "false") {
                        parsedJson.put((String) key, false);
                    }
                }
            });
            return parsedJson;
        } else {
            return null;
        }
    }
    private HashMap<String, String> getMiscValsMap() throws JSONException {
        JSONObject reader = new JSONObject(Variables.curJSONdata);
        HashMap keysMap = new Gson().fromJson(String.valueOf(reader), HashMap.class);
        HashMap<String, String> parsedJson = new HashMap();

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            keysMap.forEach((key,val) -> {
                if(key.toString().length()!=1) {
                    parsedJson.put((String) key, (String) val);
                }
            });
            return parsedJson;
        } else {
            return null;
        }
    }
    private Boolean isKeyPressed(String key, HashMap<String, Boolean> pressedKeysMap) {
        if(pressedKeysMap.containsKey(key)) {
            return pressedKeysMap.get(key);
        } else {
            return false;
        }
    }
    private String getMiscVal(String val, HashMap<String, String> misvalmap) {
        if(misvalmap.containsKey(val)) {
            return misvalmap.get(val);
        } else {
            return "";
        }
    }
    private double convertKeysToEmuJoystick(String key1, String key2, HashMap<String, Boolean> pressedKeysMap, double minmaxvalue) {
        double emuvalue = 0.0;
        if(isKeyPressed(key1,pressedKeysMap)) {
            emuvalue-=0.5;
        }
        if(isKeyPressed(key2,pressedKeysMap)) {
            emuvalue+=0.5;
        }
        return emuvalue;
    }
    private double convertKeysToEmuJoystick(String key1, String key2, HashMap<String, Boolean> pressedKeysMap) {
        return convertKeysToEmuJoystick(key1,key2,pressedKeysMap,0.5);
    }
    double leftx = 0.0;
    double lefty = 0.0;
    double rightX = 0.0;
    int speedDiv = 1;
    int hasusedebefore = 0;
    int height = 0;
    boolean isclamp = false;
    boolean isStacking = false;

    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

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

        clawRelease();
        waitForStart();
        TinyWebServer.startServer("192.168.43.1",6969,"/");
        GoToHeight(0);
        HashMap<String, Boolean> pressedKeys = null;
        HashMap<String, String> miscVals = null;
        while(opModeIsActive()) {
            try {
                pressedKeys = getPressedKeysMap();
                miscVals = getMiscValsMap();
            } catch (JSONException e) {
//            throw new RuntimeException(e);
                telemetry.addLine("JSONERROR: " + e.toString());
                pressedKeys = new HashMap<String, Boolean>();
                miscVals = new HashMap<String, String>();
            }
            assert pressedKeys != null;

            lefty = convertKeysToEmuJoystick("w","s",pressedKeys);
            leftx = convertKeysToEmuJoystick("a","d",pressedKeys);

            height += (int) (convertKeysToEmuJoystick("i","k",pressedKeys,50));
            GoToHeight(height);

            if(isKeyPressed("e", pressedKeys) && hasusedebefore==0) {
                hasusedebefore = 1;
                if(isclamp) {
                    isclamp = false;
                    clawRelease();
                } else {
                    isclamp = true;
                    clawClamp();
                }
            }
            switch(getMiscVal("targetheight",miscVals)) {
                case "lowpole":
                    goToLow();
                case "mediumpole":
                    goToMid();
                case "highpole":
                    goToHigh();
                case "collect":
                    goToCollect();
                case "ground":
                    GoToHeight(0);
            }
            if(isKeyPressed("q",pressedKeys) && !isStacking) {
                isStacking = true;
                alignToPole(camera);
                isStacking = false;
            }

            rightX = convertKeysToEmuJoystick("r","t",pressedKeys);

            telemetry.addLine("Height: " + height);
            telemetry.addLine("Left X: " + leftx + ", Left Y: " + lefty);
            telemetry.addLine("Right X: " + rightX);
            telemetry.update();
            motorFL.setPower((lefty + leftx + rightX) / speedDiv);
            motorBL.setPower((lefty - leftx + rightX) / speedDiv);
            motorFR.setPower((lefty - leftx - rightX) / speedDiv);
            motorBR.setPower((lefty + leftx - rightX) / speedDiv);
            if(hasusedebefore>0 && !isKeyPressed("e",pressedKeys)) {
                hasusedebefore--;
            }
        }
    }
}
