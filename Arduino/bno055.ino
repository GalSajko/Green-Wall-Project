#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

struct Eulers {
    float roll, pitch, yaw;
};

imu::Vector<3> getGlobalRpy(float x_reading, float y_reading, float z_reading){
    imu::Vector<3> global_rpy;
    global_rpy[0] = y_reading;
    global_rpy[1] = z_reading;
    global_rpy[2] = x_reading;

    return global_rpy;
}

imu::Vector<3> getLocalRpy(imu::Vector<3> global_rpy){
    imu::Vector<3> local_rpy;
    local_rpy[0] = global_rpy[2];
    local_rpy[1] = global_rpy[1];
    local_rpy[2] = global_rpy[0];

    return local_rpy;
}

Eulers getEulerAnglesFromQuaternion(imu::Quaternion quaternion){
    Eulers eulers;
    float q1 = quaternion.x();
    float q2 = quaternion.y();
    float q3 = quaternion.z();
    float q0 = quaternion.w();

    eulers.roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    eulers.pitch = asin(2 * (q0 * q2 - q3 * q1));
    eulers.yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    return eulers;
}

void setup(void){
    Serial.begin(9600);

    if (!bno.begin()){
        Serial.print("No BNO055 detected.");
        while(1);
    }

    delay(1000);

    bno.setAxisRemap(bno.REMAP_CONFIG_CUSTOM);
    bno.setAxisSign(bno.REMAP_SIGN_P6);

    bno.setExtCrystalUse(true);
}

void loop(void){
    sensors_event_t global_rpy_reader;
    bno.getEvent(&global_rpy_reader);

    imu::Quaternion quaternion;
    quaternion = bno.getQuat();
    
    Eulers eulers = getEulerAnglesFromQuaternion(quaternion);
    Serial.print(quaternion.x());
    Serial.print("\t");
    Serial.print(quaternion.y());
    Serial.print("\t");
    Serial.print(quaternion.z());
    Serial.print("\t");
    Serial.print(quaternion.w());
    Serial.println("");


    delay(100);
}