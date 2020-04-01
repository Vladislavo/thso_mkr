#include <Arduino.h>
#include <Wire.h>

#include <DHT.h>
#include <DHT_U.h>
#include <SHTSensor.h>
#include <HIHReader.h>

#include <bus_protocol/bus_protocol.h>

#include <util/log/log.h>

#define BAUDRATE                            115200

#define DHT22_PIN                           A1
#define DHT22_WAIT_RESP_MS                  500

#define HH10D_PIN                           7

#define BUS_PROTOCOL_MAX_PACKET_SIZE        128

#define LED_SERIAL                          6

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

void read_sensors(mkr_sensor_data_t *sensor_data);

uint8_t bus_protocol_serial_receive(
    Stream *serial, 
    uint8_t *data, 
    uint8_t *data_length, 
    const uint32_t timeout);
void bus_protocol_sensor_data_payload_encode (
    const mkr_sensor_data_t *sensors_data,
    uint8_t *packet,
    uint8_t *packet_length);

int sens;
int ofs;

uint8_t payload[BUS_PROTOCOL_MAX_PACKET_SIZE];
uint8_t payload_length = 0;
uint8_t buffer[BUS_PROTOCOL_MAX_PACKET_SIZE];
uint8_t buffer_length = 0;

void setup() {
    Serial.begin(BAUDRATE);
    Serial1.begin(BAUDRATE);
    Wire.begin();

    dht.begin();
    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);

    setup_hh10d();

    pinMode(LED_SERIAL, OUTPUT);
    digitalWrite(LED_SERIAL, LOW);

    LOG_D("init done !\r\n");
}

mkr_sensor_data_t sensor_data;

void loop() {
    LOG_D("LISTEN TO CHANNEL\r\n");
    if (bus_protocol_serial_receive(&Serial1, buffer, &buffer_length, 500)) {
        switch (bus_protocol_packet_decode(buffer, buffer_length, buffer, &buffer_length)) {
            case BUS_PROTOCOL_PACKET_TYPE_DATA_REQUEST :
                digitalWrite(LED_SERIAL, HIGH);
                LOG_D("DATA REQUEST RECEIVED\r\n");

                read_sensors(&sensor_data);

                /* prepare data for sending */
                bus_protocol_sensor_data_payload_encode(&sensor_data, payload, &payload_length);
                bus_protocol_data_send_encode(payload, payload_length, buffer, &buffer_length);

                Serial1.write(buffer, buffer_length);

                digitalWrite(LED_SERIAL, LOW);
                break;
            default:
                break;
        }
    } else {
        LOG_D("NO REQUEST RECEIVED ESP\r\n");
    }
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
    uint32_t wait_ms = DHT22_WAIT_RESP_MS + millis();

    do {
        delay(100);
        sensor_data->dht22_t = dht.readTemperature();
        sensor_data->dht22_h = dht.readHumidity();
        if (isnan(sensor_data->dht22_t) || isnan(sensor_data->dht22_h)) {
            LOG_E("Failed to read from DHT sensor!");
            delay(100);
        }
    } while ((isnan(isnan(sensor_data->dht22_t)) || isnan(sensor_data->dht22_h)) && wait_ms > millis());
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

void read_sensors(mkr_sensor_data_t *sensor_data) {
    read_dht22(sensor_data);
    read_sht85(sensor_data);
    read_hih8121(sensor_data);
    read_hh10d(sensor_data);

    Serial.println("DHT22 : ");
    Serial.print(sensor_data->dht22_h); Serial.print(", "); Serial.println(sensor_data->dht22_t);

    Serial.println("SHT85 : ");
    Serial.print(sensor_data->sht85_h); Serial.print(", "); Serial.println(sensor_data->sht85_t);

    Serial.println("HIH8121 : ");
    Serial.print(sensor_data->hih8121_h); Serial.print(", "); Serial.println(sensor_data->hih8121_t);

    Serial.println("HH10D : ");
    Serial.println(sensor_data->hh10d);
}

uint8_t bus_protocol_serial_receive(
    Stream *serial, 
    uint8_t *data, 
    uint8_t *data_length, 
    const uint32_t timeout)
{
    *data_length = 0;
    // uint32_t start_millis = millis();
    uint32_t start_millis = UINT32_MAX - timeout - 1;
    while(start_millis + timeout > millis() && *data_length < BUS_PROTOCOL_MAX_PACKET_SIZE) {
        if (serial->available()) {
            data[(*data_length)++] = serial->read();
            // update wating time
            start_millis = millis();
        }
    }

    return *data_length;
}

void bus_protocol_sensor_data_payload_encode (
    const mkr_sensor_data_t *sensors_data,
    uint8_t *packet,
    uint8_t *packet_length) 
{
    *packet_length = 0;

    packet[*packet_length] = BUS_PROTOCOL_BOARD_ID_MKR;
    (*packet_length)++;

    memcpy(&packet[*packet_length], &sensors_data->dht22_t, sizeof(sensors_data->dht22_t));
    (*packet_length) += sizeof(sensors_data->dht22_t);

    memcpy(&packet[*packet_length], &sensors_data->dht22_h, sizeof(sensors_data->dht22_h));
    (*packet_length) += sizeof(sensors_data->dht22_h);

    memcpy(&packet[*packet_length], &sensors_data->sht85_t, sizeof(sensors_data->sht85_t));
    (*packet_length) += sizeof(sensors_data->sht85_t);

    memcpy(&packet[*packet_length], &sensors_data->sht85_h, sizeof(sensors_data->sht85_h));
    (*packet_length) += sizeof(sensors_data->sht85_h);

    memcpy(&packet[*packet_length], &sensors_data->hih8121_t, sizeof(sensors_data->hih8121_t));
    (*packet_length) += sizeof(sensors_data->hih8121_t);

    memcpy(&packet[*packet_length], &sensors_data->hih8121_h, sizeof(sensors_data->hih8121_h));
    (*packet_length) += sizeof(sensors_data->hih8121_h);

    memcpy(&packet[*packet_length], &sensors_data->hh10d, sizeof(sensors_data->hh10d));
    (*packet_length) += sizeof(sensors_data->hh10d);
}