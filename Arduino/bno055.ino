#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

adafruit_bno055_offsets_t CALIB_DATA = {
    -1, 19, -21, 0, 0, 0, 2, -1, 2, 1000, 480
};

struct Eulers {
    float roll, pitch, yaw;
};

Eulers init_rpy;

void substractInitValue(Eulers *eulers){
    eulers->pitch -= init_rpy.pitch;
    eulers->roll -= init_rpy.roll;
    eulers->yaw -= init_rpy.yaw; 
}

Eulers getEulerAnglesFromQuaternion(imu::Quaternion quaternion){
    Eulers eulers;
    float q1 = quaternion.x();
    float q2 = quaternion.y();
    float q3 = quaternion.z();
    float q0 = quaternion.w();

    eulers.pitch = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    eulers.yaw = asin(2 * (q0 * q2 - q3 * q1)) * (-1);
    eulers.roll = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    substractInitValue(&eulers);

    return eulers;
}

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void){
    Serial.begin(9600);

    if (!bno.begin(OPERATION_MODE_CONFIG)){
        Serial.print("No BNO055 detected.");
        while(1);
    }

    // Load calibration.
    bno.setSensorOffsets(CALIB_DATA);
    // Put in IMU mode (relative heading).
    bno.setMode(OPERATION_MODE_IMUPLUS);

    delay(1000);

    // Remap axis to match spider's orientation on the wall.
    bno.setAxisRemap(bno.REMAP_CONFIG_P1);
    bno.setAxisSign(bno.REMAP_SIGN_P7);

    bno.setExtCrystalUse(true);

    Serial.println("CALIBRATION LOADED. PLACE SENSOR IN STARTING POSITION.");
    delay(5000);
    Serial.println("SENSOR IN STARTING POSITION.");
    // Read initial values.
    init_rpy = getEulerAnglesFromQuaternion(bno.getQuat());
}

void loop(void){
    imu::Quaternion quaternion;
    quaternion = bno.getQuat();
    
    Eulers eulers = getEulerAnglesFromQuaternion(quaternion);

    Serial.print(eulers.roll);
    Serial.print("\t");
    Serial.print(eulers.pitch);
    Serial.print("\t");
    Serial.print(eulers.yaw);
    Serial.println("");


    delay(100);
}