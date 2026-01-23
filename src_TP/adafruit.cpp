#include "mbed.h"
#include "rtos.h"
#include "bme280.h"
#include <MQTTClientMbedOs.h>
#include <nsapi_dns.h>

using namespace sixtron;


#define IO_USERNAME  "segroupe7"
#define IO_KEY       "aio_JDUC41UdUemIDC292LCTcczBFTyJ"

#define TOPIC_TEMP   IO_USERNAME "/feeds/temperature"
#define TOPIC_HUM    IO_USERNAME "/feeds/humidite"
#define TOPIC_PRES   IO_USERNAME "/feeds/pression"
#define TOPIC_LED    IO_USERNAME "/feeds/led"

DigitalOut led(LED1);
I2C i2c(I2C1_SDA, I2C1_SCL);
BME280 bme(&i2c, BME280::I2CAddress::Address1);

NetworkInterface *network;
MQTTClient *client;
const char* hostname = "io.adafruit.com"; 
int port = 1883;


void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;

    char buffer[20];
    if (message.payloadlen >= sizeof(buffer)) return; 
    memcpy(buffer, message.payload, message.payloadlen);
    buffer[message.payloadlen] = '\0'; 

    printf("Message recu sur LED: %s\n", buffer);

    if (strcmp(buffer, "ON") == 0) {
        led = 1;
    } else if (strcmp(buffer, "OFF") == 0) {
        led = 0;
    }
}

void publish_value(const char* topic, float value) {
    char payload[32];
    sprintf(payload, "%.2f", value); 

    MQTT::Message message;
    message.qos = MQTT::QOS1;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)payload;
    message.payloadlen = strlen(payload);

    client->publish(topic, message);
}


int main() {
    printf("Demarrage Station Meteo Cloud...\n");

    if (!bme.initialize()) {
        printf("Erreur BME280\n");
        return -1;
    }
    bme.set_sampling();


    network = NetworkInterface::get_default_instance();
    if (!network) { printf("Erreur Interface Reseau\n"); return -1; }
    
    printf("Connexion au routeur...\n");
    if (network->connect() != 0) { printf("Echec connexion Reseau\n"); return -1; }
    

    nsapi_addr_t new_dns = {NSAPI_IPv6, {0xfd, 0x9f, 0x59, 0x0a, 0xb1, 0x58, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x01}};
    nsapi_dns_add_server(new_dns, "LOWPAN");

    TCPSocket socket;
    SocketAddress address;
    network->gethostbyname(hostname, &address);
    address.set_port(port);
    
    socket.open(network);
    if (socket.connect(address) != 0) { printf("Echec connexion TCP\n"); return -1; }

    client = new MQTTClient(&socket);
    
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 4;
    data.keepAliveInterval = 60;
    

    data.username.cstring = (char*)IO_USERNAME;
    data.password.cstring = (char*)IO_KEY;


    if (client->connect(data) != 0) { printf("Echec Auth MQTT\n"); return -1; }
    printf("Connecté a Adafruit IO !\n");

    client->subscribe(TOPIC_LED, MQTT::QOS0, messageArrived);

    while (true) {
        float t = bme.temperature();
        float h = bme.humidity();
        float p = bme.pressure() / 100.0f; 

        printf("Envoi: T=%.2f H=%.2f P=%.2f\n", t, h, p);

     
        publish_value(TOPIC_TEMP, t);
        publish_value(TOPIC_HUM, h);
        publish_value(TOPIC_PRES, p);

        for (int i = 0; i < 100; i++) {
            client->yield(100); 
        }
    
    }
}