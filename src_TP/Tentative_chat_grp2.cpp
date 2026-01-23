
#include "mbed.h"
#include "rtos.h"
#include "bme280.h"
#include <MQTTClientMbedOs.h>
#include <nsapi_dns.h>
#include <cstdio>
#include <cstring>

using namespace sixtron;



#define IO_USERNAME  "segroupe7"
#define IO_KEY       "aio_JDUC41UdUemIDC292LCTcczBFTyJ"

#define MOSQ_HOST "test.mosquitto.org"
#define MOSQ_PORT 1883

#define CHAT_TOPIC "seelec/chat/general"

#define BOARD_NAME "Groupe 7"


#define TOPIC_TEMP   IO_USERNAME "/feeds/temperature"
#define TOPIC_HUM    IO_USERNAME "/feeds/humidite"
#define TOPIC_PRES   IO_USERNAME "/feeds/pression"
#define TOPIC_LED    IO_USERNAME "/feeds/led"

#define TOPIC_CHAT_IN  IO_USERNAME "/feeds/chat_in"

DigitalOut led(LED1);
InterruptIn button(BUTTON1);
I2C i2c(I2C1_SDA, I2C1_SCL);
BME280 bme(&i2c, BME280::I2CAddress::Address1);

BufferedSerial pc(USBTX, USBRX, 115200);

MQTTClient *mosq_client;
TCPSocket mosq_socket;

NetworkInterface *network;
MQTTClient *client;
const char* hostname = "io.adafruit.com";
int port = 1883;

static char typed_msg[128];
static int typed_len = 0;
static volatile bool send_request = false;
static volatile bool msg_ready = false;
static Mutex msg_mutex;
Thread terminalThread;


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

void publish_text_adafruit(const char* topic, const char* text)
{
    MQTT::Message message;
    message.qos = MQTT::QOS1;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)text;
    message.payloadlen = strlen(text);

    client->publish(topic, message);
}

void mosq_send_chat(const char *text)
{
    char payload[180];
    snprintf(payload, sizeof(payload), "%s: %s", BOARD_NAME, text);

    MQTT::Message msg;
    msg.qos = MQTT::QOS0;
    msg.retained = false;
    msg.dup = false;
    msg.payload = (void*)payload;
    msg.payloadlen = strlen(payload);

    mosq_client->publish(CHAT_TOPIC, msg);

    pc.write("\r\n[SENT]\r\n> ", 12);
}

void mosq_messageArrived(MQTT::MessageData &md)
{
    MQTT::Message &m = md.message;

    char buffer[180];
    int len = m.payloadlen;
    if (len > 170) len = 170;

    memcpy(buffer, m.payload, len);
    buffer[len] = '\0';

    // Log received message to your own Adafruit account
    publish_text_adafruit(TOPIC_CHAT_IN, buffer);

    printf("> "); // reprint prompt

    printf("\n[RECV] %s\n> ", buffer);

    publish_text_adafruit(TOPIC_CHAT_IN, buffer);
    fflush(stdout);
}



void button_isr()
{
    msg_ready = true;
    send_request = true;
}

void terminal_task()
{
    printf("\n=== Type message, press ENTER to store, button to send ===\n> ");

    char c;
    while (true) {
        if (pc.read(&c, 1) == 1) {

            // ENTER -> store line
            if (c == '\r' || c == '\n') {
                msg_mutex.lock();
                typed_msg[typed_len] = '\0';
                msg_mutex.unlock();

                printf("\n[OK] Stored. Press button to send.\n> ");
                continue;
            }

            // Backspace
            if (c == 127 || c == '\b') {
                msg_mutex.lock();
                if (typed_len > 0) {
                    typed_len--;
                    typed_msg[typed_len] = '\0';
                    printf("\b \b");
                }
                msg_mutex.unlock();
                continue;
            }

            // Normal char
            msg_mutex.lock();
            if (typed_len < (int)sizeof(typed_msg) - 1) {
                typed_msg[typed_len++] = c;
                pc.write(&c, 1); // echo char
            }
            msg_mutex.unlock();
        }

        ThisThread::sleep_for(5ms);
    }
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

    // ===============================
    // Connect to Mosquitto
    // ===============================
    printf("Connecting to Mosquitto...\n");

    SocketAddress mosq_addr;
    if (network->gethostbyname(MOSQ_HOST, &mosq_addr) != 0) {
        printf("Mosquitto DNS failed\n");
        return -1;
    }
    mosq_addr.set_port(MOSQ_PORT);

    mosq_socket.open(network);
    if (mosq_socket.connect(mosq_addr) != 0) {
        printf("Mosquitto TCP connect failed\n");
        return -1;
    }

    mosq_client = new MQTTClient(&mosq_socket);

    MQTTPacket_connectData mosq_data = MQTTPacket_connectData_initializer;
    mosq_data.MQTTVersion = 4;
    mosq_data.keepAliveInterval = 30;
    mosq_data.clientID.cstring = (char*)"CHAT_NODE_" BOARD_NAME;

    if (mosq_client->connect(mosq_data) != 0) {
        printf("Mosquitto MQTT connect failed\n");
        return -1;
    }

    printf("Connected to Mosquitto!\n");

    // Subscribe to chat topic
    mosq_client->subscribe(CHAT_TOPIC, MQTT::QOS0, mosq_messageArrived);
    printf("Subscribed to: %s\n", CHAT_TOPIC);

    client->subscribe(TOPIC_LED, MQTT::QOS0, messageArrived);
    
    button.fall(&button_isr);
    terminalThread.start(terminal_task);

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
            mosq_client->yield(100); // Mosquitto

            if (send_request) {
                send_request = false;

                msg_mutex.lock();
                bool ok = (typed_len > 0);
                char copy[128];
                if (ok) {
                    strcpy(copy, typed_msg);
                    typed_len = 0;
                    typed_msg[0] = '\0';
                }
                msg_mutex.unlock();

                if (ok) {
                    mosq_send_chat(copy);
                } else {
                    printf("\n[WARN] No message typed\n> ");
                }
            }
        }
    }
}
