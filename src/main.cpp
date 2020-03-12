#include <Arduino.h>
#include <Wire.h>

#include <DHT.h>
#include <DHT_U.h>
#include <SHTSensor.h>
#include <HIHReader.h>

#define BAUDRATE                            115200

#define DHT22_PIN                           A1
#define DHT22_READ_RETRIES                  100

#define HH10D_PIN                           7

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);

typedef struct {
    float dht22_t   = .0;
    float dht22_h   = .0;
    float sht85_t   = .0;
    float sht85_h   = .0;
    float hih8121_t = .0;
    float hih8121_h = .0;
    float hh10d     = .0;
} mkr_sensor_data_t;

void setup_hh10d();

void read_dht22(mkr_sensor_data_t *sensor_data);
void read_sht85(mkr_sensor_data_t *sensor_data);
void read_hih8121(mkr_sensor_data_t *sensor_data);
void read_hh10d(mkr_sensor_data_t *sensor_data);

int sens;
int ofs;

void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin();

    dht.begin();
    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);

    setup_hh10d();
}

mkr_sensor_data_t sensor_data;

void loop() {
    read_dht22(&sensor_data);
    read_sht85(&sensor_data);
    read_hih8121(&sensor_data);
    read_hh10d(&sensor_data);

    Serial.println("DHT22 : ");
    Serial.println(sensor_data.dht22_h);
    Serial.println(sensor_data.dht22_t);

    Serial.println("SHT85 : ");
    Serial.println(sensor_data.sht85_h);
    Serial.println(sensor_data.sht85_t);

    Serial.println("HIH8121 : ");
    Serial.println(sensor_data.hih8121_h);
    Serial.println(sensor_data.hih8121_t);

    Serial.println("HH10D : ");
    Serial.println(sensor_data.hh10d);

    delay(1000);
}

// function to intitialize HH10D
int i2cRead2bytes(int deviceaddress, byte address) {
    // SET ADDRESS
    Wire.beginTransmission(deviceaddress);
    Wire.write(address); // address for sensitivity
    Wire.endTransmission();

    Wire.requestFrom(deviceaddress, 2);

    int rv = 0;
    for (int c = 0; c < 2; c++ ) {
        if (Wire.available()) {
            rv = rv * 256 + Wire.read();
        }
    }

    return rv;
}

void setup_hh10d() {
    const int HH10D_I2C_ADDRESS = 81;
    sens = i2cRead2bytes(HH10D_I2C_ADDRESS, 10); 
	  ofs  = i2cRead2bytes(HH10D_I2C_ADDRESS, 12);
}

void read_dht22(mkr_sensor_data_t *sensor_data) {
    uint8_t retries = DHT22_READ_RETRIES;

    do {
        delay(100);
        sensor_data->dht22_t = dht.readTemperature();
        sensor_data->dht22_h = dht.readHumidity();
        if (isnan(sensor_data->dht22_t) || isnan(sensor_data->dht22_h)) {
            Serial.println("Failed to read from DHT sensor!");
            delay(100);
        }
        retries--;
    } while ((isnan(isnan(sensor_data->dht22_t)) || isnan(sensor_data->dht22_h)) && retries);
}

void read_sht85(mkr_sensor_data_t *sensor_data) {
    sht85.readSample();
    sensor_data->sht85_t = sht85.getTemperature();
    sensor_data->sht85_h = sht85.getHumidity();
}

void read_hih8121(mkr_sensor_data_t *sensor_data) {
    hih8121.read(&sensor_data->hih8121_t, &sensor_data->hih8121_h);
}

void read_hh10d(mkr_sensor_data_t *sensor_data) {
    const int HH10D_FOUT_PIN    = HH10D_PIN;
    float freq = .0;
      for (int j=0; j < 256; j++) {
          freq += 500000/pulseIn(HH10D_FOUT_PIN, HIGH, 250000);
      }
    freq /= 256;

    sensor_data->hh10d = float((ofs - freq)* sens)/float(4096);
}