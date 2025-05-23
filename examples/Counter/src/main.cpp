#include <Arduino.h>
#include "LoraMesher.h"
#include <queue>
/**
 * @brief define LED
 */
#define BOARD_LED 4
#define LED_ON LOW
#define LED_OFF HIGH

LoraMesher &radio = LoraMesher::getInstance(); // Get LoRaMesher Instance

uint16_t NODE_ADDR = radio.getLocalAddress();

struct packetCtr
{
    uint16_t src_addr = NODE_ADDR;
    uint16_t hop_counter = 0;
};

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
    for (uint16_t index = 1; index <= flashes; index++)
    {
        digitalWrite(BOARD_LED, LED_ON);
        delay(delaymS);
        digitalWrite(BOARD_LED, LED_OFF);
        delay(delaymS);
    }
}

void setupLoraMesher()
{
    LoraMesher::LoraMesherConfig config;
    config.loraCs = 18;
    config.loraRst = 23;
    config.loraIrq = 26;
    config.loraIo1 = 33;
    config.module = LoraMesher::LoraModules::SX1276_MOD;
    SPIClass *spi = new SPIClass();
    spi->begin(5, 19, 27);
    config.spi = spi;
    radio.begin(config);
    radio.start();
    NODE_ADDR = radio.getLocalAddress();
    // createReceiveMessages();
    // radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
    Serial.println("Lora initialized");
}

void setup()
{
    Serial.begin(115200);
    pinMode(BOARD_LED, OUTPUT);
    led_Flash(8, 125);
    setupLoraMesher();
}

void loop()
{
    static uint32_t lastSend = 0, counter = 0;
    if (millis() - lastSend > 5000)
    {
        uint16_t dest = 8784;
        packetCtr packet;
        uint8_t packetSize = sizeof(packet);
        radio.createPacketAndSend<packetCtr>(dest, &packet, packetSize + 1);
        lastSend = millis();
    }
}

// dataPacket *helloPacket = new dataPacket;

// void led_Flash(uint16_t flashes, uint16_t delaymS)
// {
//     for (uint16_t index = 1; index <= flashes; index++)
//     {
//         digitalWrite(BOARD_LED, LED_ON);
//         delay(delaymS);
//         digitalWrite(BOARD_LED, LED_OFF);
//         delay(delaymS);
//     }
// }

// void printPacket(const dataPacket &data, uint16_t src, uint16_t dst)
// {
//     Serial.printf("Packet: SRC:%04X DST:%04X CNTR:%lu (from %04X, to %04X)\n", data.source, data.destination, data.counter, src, dst);
// }

// void printDataPacket(AppPacket<dataPacket> *packet)
// {
//     Serial.printf("Packet arrived from %04X with size %d\n", packet->src, packet->payloadSize);
//     dataPacket *dPacket = packet->payload;
//     size_t payloadLength = packet->getPayloadLength();
//     for (size_t i = 0; i < payloadLength; i++)
//     {
//         printPacket(dPacket[i], packet->src, packet->dst);
//     }
// }

// // This function now properly handles multihop (forwarding) logic!
// void processReceivedPackets(void *)
// {
//     for (;;)
//     {
//         ulTaskNotifyTake(pdPASS, portMAX_DELAY);
//         led_Flash(1, 100);

//         while (radio.getReceivedQueueSize() > 0)
//         {
//             AppPacket<dataPacket> *packet = radio.getNextAppPacket<dataPacket>();
//             if (packet)
//             {
//                 printDataPacket(packet);

//                 dataPacket *dPacket = packet->payload;

//                 // If I am the final destination, process the packet
//                 if (MY_ADDR == dPacket->destination)
//                 {
//                     Serial.println("*** Final destination received packet! ***");
//                     // Process the data as needed
//                 }
//                 // If not for me, forward if not already seen (or implement other duplicate suppression as needed)
//                 else
//                 {
//                     Serial.printf("Packet not for me (%04X), forwarding towards %04X\n", MY_ADDR, dPacket->destination);
//                     // Forward the same packet contents to the intended destination
//                     // You may want to implement duplicate filtering/rate limiting here
//                     radio.createPacketAndSend(dPacket->destination, dPacket, 1);
//                 }

//                 radio.deletePacket(packet);
//             }
//         }
//     }
// }

// TaskHandle_t receiveLoRaMessage_Handle = NULL;

// void createReceiveMessages()
// {
//     int res = xTaskCreate(
//         processReceivedPackets,
//         "Receive App Task",
//         4096,
//         (void *)1,
//         2,
//         &receiveLoRaMessage_Handle);
//     if (res != pdPASS)
//     {
//         Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
//     }
// }

// void setup()
// {
//     Serial.begin(115200);
//     Serial.println("initBoard");
//     pinMode(BOARD_LED, OUTPUT);
//     led_Flash(8, 125);
//     setupLoraMesher();
//     // Set this node's address (if required by LoRaMesher version)
//     // radio.setLocalAddress(MY_ADDR); // Uncomment if your LoRaMesher supports this
// }

// void loop()
// {
// #if (MY_ADDR == ADDR_2288) || (MY_ADDR == ADDR_2250)
//     static uint32_t counter = 0;
//     // Randomly delay between 10 and 25 seconds
//     uint32_t delayMs = 10000 + (esp_random() % 15000);

//     helloPacket->source = MY_ADDR;
//     helloPacket->destination = ADDR_C8D8;
//     helloPacket->counter = counter++;

//     // The LoRaMesher routing will either find a direct or multi-hop path
//     radio.createPacketAndSend(ADDR_C8D8, helloPacket, 1);

//     Serial.printf("Node %04X sent packet #%lu to %04X (will wait %lu ms)\n", MY_ADDR, helloPacket->counter, ADDR_C8D8, delayMs);
//     vTaskDelay(delayMs / portTICK_PERIOD_MS);
// #else
//     // C8D8 just receives and prints
//     vTaskDelay(20000 / portTICK_PERIOD_MS);
// #endif
// }