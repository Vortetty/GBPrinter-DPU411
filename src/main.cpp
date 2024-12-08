#include <Arduino.h>
#include <ArduinoJson.h>
#include <HalfStepper/HalfStepper.h>
#include <SerialPIO.h>
#include <bitset>
#include <deque>
#include <queue>
#include <string>

SerialPIO serialBufferIn(1, 2, 64 * 1024);  // 64KiB... yeah it's alot but that's more than enough to store any communication from the other controller, since i just wanna process it all at once

// Define a stepper and the pins it will use
// Stepper stepper(24, PIN_B5, PIN_B3, PIN_B6, PIN_B4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

#define ENDSTOP_DETECT 17
#define FEED_BUTTON 16

#define HEAD_STEPPER_PIN_1 28
#define HEAD_STEPPER_PIN_2 26
#define HEAD_STEPPER_PIN_3 27
#define HEAD_STEPPER_PIN_4 22

#define PAPER_STEPPER_PIN_1 21
#define PAPER_STEPPER_PIN_2 19
#define PAPER_STEPPER_PIN_3 20
#define PAPER_STEPPER_PIN_4 18

HalfStepper headStepper(20, HEAD_STEPPER_PIN_1, HEAD_STEPPER_PIN_2, HEAD_STEPPER_PIN_3, HEAD_STEPPER_PIN_4);
HalfStepper paperStepper(20, PAPER_STEPPER_PIN_1, PAPER_STEPPER_PIN_2, PAPER_STEPPER_PIN_3, PAPER_STEPPER_PIN_4);

#define HEAD_STEPPER_END_OFFSET -9
#define HEAD_STEPPER_LEFT_MARGIN 52   // margins should both be 30, gameboy printer images however are 160px wide. thus to keep the image centered a margin of 52 is used
#define HEAD_STEPPER_RIGHT_MARGIN 52  // the margin of 30 leaves 684 usable dots, so 684/160 = 4.275px, that leaves the closest valid pixel size at 4, so 640px wide
                                      //   this leaves 104 unused pixels, which are split between the two sides for centering reasons, so 52+30 gives the final 82 dot margin
#define HEAD_STEPPER_TRUE_MAX 744
#define HEAD_STEPPER_MAX (HEAD_STEPPER_TRUE_MAX - HEAD_STEPPER_RIGHT_MARGIN)
#define HEAD_STEPPER_DOTS (HEAD_STEPPER_MAX - HEAD_STEPPER_LEFT_MARGIN)
#define PAPER_STEPS_PER_PIXEL -7

#define MIN_PAPER_BURN_MICROS 1000
#define MAX_PAPER_BURN_MICROS 3500
#define BURNER_PIN_1 8  // bottom
#define BURNER_PIN_2 9
#define BURNER_PIN_3 10
#define BURNER_PIN_4 11
#define BURNER_PIN_5 12
#define BURNER_PIN_6 13
#define BURNER_PIN_7 14
#define BURNER_PIN_8 15
#define BURNER_PIN_9 7  // broken :cs

#define PRINT_FINISH_PIN 2

byte imageBuffer[128 * 112 * 2 / 8];

long long int headStepCounter = 0;

void headStepperMoveRel(int steps) {
    headStepper.step(steps);
    headStepCounter += steps;
}
inline void headStepperMoveTo(int steps) {
    headStepperMoveRel(steps - headStepCounter);
}

void runBurner(uint32_t pin, uint16_t micros) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(micros);
    digitalWrite(pin, LOW);
}

void run8Burners4Pixels(uint32_t px1, uint32_t px2, uint32_t px3, uint32_t px4) {
    digitalWrite(BURNER_PIN_1, HIGH);
    digitalWrite(BURNER_PIN_2, HIGH);
    digitalWrite(BURNER_PIN_3, HIGH);
    digitalWrite(BURNER_PIN_4, HIGH);
    digitalWrite(BURNER_PIN_5, HIGH);
    digitalWrite(BURNER_PIN_6, HIGH);
    digitalWrite(BURNER_PIN_7, HIGH);
    digitalWrite(BURNER_PIN_8, HIGH);
    delayMicroseconds(200);
    digitalWrite(BURNER_PIN_1, px1 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_2, px1 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_3, px2 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_4, px2 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_5, px3 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_6, px3 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_7, px4 == 0 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_8, px4 == 0 ? LOW : HIGH);
    delayMicroseconds(450);
    digitalWrite(BURNER_PIN_1, px1 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_2, px1 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_3, px2 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_4, px2 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_5, px3 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_6, px3 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_7, px4 <= 1 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_8, px4 <= 1 ? LOW : HIGH);
    delayMicroseconds(450);
    digitalWrite(BURNER_PIN_1, px1 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_2, px1 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_3, px2 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_4, px2 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_5, px3 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_6, px3 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_7, px4 <= 2 ? LOW : HIGH);
    digitalWrite(BURNER_PIN_8, px4 <= 2 ? LOW : HIGH);
    delayMicroseconds(450);
    digitalWrite(BURNER_PIN_1, LOW);
    digitalWrite(BURNER_PIN_2, LOW);
    digitalWrite(BURNER_PIN_3, LOW);
    digitalWrite(BURNER_PIN_4, LOW);
    digitalWrite(BURNER_PIN_5, LOW);
    digitalWrite(BURNER_PIN_6, LOW);
    digitalWrite(BURNER_PIN_7, LOW);
    digitalWrite(BURNER_PIN_8, LOW);
}

void runAllBurners(uint16_t micros) {
    digitalWrite(BURNER_PIN_1, HIGH);
    digitalWrite(BURNER_PIN_2, HIGH);
    digitalWrite(BURNER_PIN_3, HIGH);
    digitalWrite(BURNER_PIN_4, HIGH);
    digitalWrite(BURNER_PIN_5, HIGH);
    digitalWrite(BURNER_PIN_6, HIGH);
    digitalWrite(BURNER_PIN_7, HIGH);
    digitalWrite(BURNER_PIN_8, HIGH);
    digitalWrite(BURNER_PIN_9, HIGH);
    delayMicroseconds(micros);
    digitalWrite(BURNER_PIN_1, LOW);
    digitalWrite(BURNER_PIN_2, LOW);
    digitalWrite(BURNER_PIN_3, LOW);
    digitalWrite(BURNER_PIN_4, LOW);
    digitalWrite(BURNER_PIN_5, LOW);
    digitalWrite(BURNER_PIN_6, LOW);
    digitalWrite(BURNER_PIN_7, LOW);
    digitalWrite(BURNER_PIN_8, LOW);
    digitalWrite(BURNER_PIN_9, LOW);
}

void setup() {
    // Init endstop
    pinMode(ENDSTOP_DETECT, INPUT_PULLUP);

    // Init feed
    pinMode(FEED_BUTTON, INPUT_PULLUP);

    // Init head stepper
    headStepper.setSpeed(700);                  // This stepper can move quick so we'll make it move quick, 700 seems to be the max unless i provide way more power
    if (digitalRead(ENDSTOP_DETECT) == HIGH) {  // If near endstop
        headStepperMoveRel(25);                 // then move right so we can accurately home the head
    }
    while (digitalRead(ENDSTOP_DETECT) == LOW) {  // Until we hit the endstop
        headStepperMoveRel(-1);                   // Move towards endstop
        delayMicroseconds(100);                   // 0.1ms delay to ensure we don't over/undershoot
    }
    headStepperMoveRel(HEAD_STEPPER_END_OFFSET);  // Move so we're actually at the end
    headStepper.SetPosition(0);                   // Set this as the home position
    headStepCounter = 0;

    // Init paper stepper
    paperStepper.setSpeed(700);

    // Burner Pins
    pinMode(BURNER_PIN_1, OUTPUT);
    pinMode(BURNER_PIN_2, OUTPUT);
    pinMode(BURNER_PIN_3, OUTPUT);
    pinMode(BURNER_PIN_4, OUTPUT);
    pinMode(BURNER_PIN_5, OUTPUT);
    pinMode(BURNER_PIN_6, OUTPUT);
    pinMode(BURNER_PIN_7, OUTPUT);
    pinMode(BURNER_PIN_8, OUTPUT);
    pinMode(BURNER_PIN_9, OUTPUT);

    // Finished printing pin
    pinMode(PRINT_FINISH_PIN, OUTPUT);
    digitalWrite(PRINT_FINISH_PIN, 0);

    // USB Serial, flow control ignore makes it work on linux
    // Bug had been reported (https://github.com/earlephilhower/arduino-pico/issues/2023) but noone seems to have been able to fix it so who knows
    // Sending serial data in the setup loop seems to softlock the board, so that's fun
    Serial.begin(115200);
    Serial.ignoreFlowControl(true);
    // while (!Serial)
    //     ;

    // Hardware UART 1, on pins 0/1, larger fifo just incase there's a short hang while processing data into duffers
    // Serial between the UNO and the pico, worth noting the uno's serialusb is echoed to both usb and the pico
    // So if the UNO has issues, one could disable the serial filter in the loop temporarily to debug it.
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.setFIFOSize(256);
    Serial1.begin(115200);
    while (!Serial1)
        ;
}

// https://gist.github.com/xsleonard/7341172?permalink_comment_id=4575673#gistcomment-4575673
// black magic, thanks github
int HexStringToBytes(const char *hexStr,
                     unsigned char *output,
                     unsigned int *outputLen) {
    size_t len = strlen(hexStr);
    if (len % 2 != 0) {
        Serial.println("hexparseerror");
        return -1;
    }
    size_t finalLen = len / 2;
    *outputLen = finalLen;
    for (size_t inIdx = 0, outIdx = 0; outIdx < finalLen; inIdx += 2, outIdx++) {
        if ((hexStr[inIdx] - 48) <= 9 && (hexStr[inIdx + 1] - 48) <= 9) {
            goto convert;
        } else {
            if (((hexStr[inIdx] - 65) <= 5 && (hexStr[inIdx + 1] - 65) <= 5) || ((hexStr[inIdx] - 97) <= 5 && (hexStr[inIdx + 1] - 97) <= 5)) {
                goto convert;
            } else {
                *outputLen = 0;
                return -1;
            }
        }
    convert:
        output[outIdx] =
            (hexStr[inIdx] % 32 + 9) % 25 * 16 + (hexStr[inIdx + 1] % 32 + 9) % 25;
    }
    output[finalLen] = '\0';
    return 0;
}

class imageTile {
public:
    imageTile() {}
    imageTile(byte *in) {
        memcpy(data, in, 16);
    }

    byte data[16];
};

std::bitset<160 * 200 * 2> imageBits;  // GB Printer has an 8KiB memory to store images
                                       // each image is *up to* 160*200px, at 2bpp
                                       // so a 64Kb or 7.8125KiB buffer is needed

std::queue<std::string> lines;  // Store the data sent from the UNO, only data commands, init commands, and print commands should end up in here
char readBuffer[128] = {0};     // Buffer for each line as it comes in, the longest line it should send is generally 75 bytes (print command) so this just ensures that we have space for extra! better safe than sorry when i have memory to spare
JsonDocument doc;               // Document for decoding json lines
std::deque<imageTile> tiles;

void loop() {
    while (true) {  // Run until break condition
        while (!Serial1.available()) {
            if (digitalRead(FEED_BUTTON) == LOW) {
                paperStepper.step(-1);
            }
        }                                                             // Wait for serial to be available, then continue
        uint8_t len = Serial1.readBytesUntil('\n', readBuffer, 128);  // Read in a line from serial

        if (readBuffer[0] == '&' || readBuffer[0] == '%') {  // Check command type, will either be json or data, any comments or blank lines are ignored
            lines.push(std::string(readBuffer));             // Store command/data in buffer
            Serial.println(readBuffer);                      // Echo to USBSerial
            if (readBuffer[len - 2] == '$') {                // If command ends with a $ sign, that means it's a print command
                                                             // Was easier to implement it this way than to parse json on the data read loop
                memset(readBuffer, 0, 128);                  // Clear read buffer
                break;                                       // Escape loop; move to printing
            }
        } else {
            // nothing since i don't care about empty lines and comments
        }

        memset(readBuffer, 0, 128);  // Clear read buffer
    }

    // Memory debug, just wanna make sure i don't use too much, which should never happen but this can help debug if it does.
    // I removed inquiry commands from the output specifically to prevent issues with memory, since i don't care about the fake printer's status anyway
    Serial.printf("%.2f/%.2fKiB used (%.2fKiB free)\n", rp2040.getUsedHeap() / 1024.0f, rp2040.getTotalHeap() / 1024.0f, rp2040.getFreeHeap() / 1024.0f);
    Serial.printf("%i lines stored\n", lines.size());

    // - Begin decoding, just gonna read through each command
    // - This will take a second or two
    // - Possible lines:
    // -                                           &00000000000000000000000000000000 | Hex data
    // -                                                         %{"command":"INIT"} | Clear all buffers and prepare to recieve data
    // -                                          %{"command":"D","end":0,"len":640} | Data sent, length will be a multiple of 16 bytes (1 gb tile)
    // -                                            %{"command":"D","end":1,"len":0} | Denotes end of data, prepare to print
    // - %{"command":"P","shts":1,"mrgnupr":1,"mrgnlwr":3,"palette":228,"dnsty":73}$ | Print
    // -                                                                                    shts: Copies to print
    // -                                                                                 mrgnupr: Blank pixels above
    // -                                                                                 mrgnlwr: Blank pixels below
    // -                                                                                 palette: seems to be a 4 crumb value, most common is 11-10-01-00, appears to tell you which color is darkest(left) and brightest(right)
    // -                                                                                   dnsty: 7-bit value for exposure time, 0 is -25%, 127 is +25%, 64 is the normal darkness (source: https://gbdev.io/pandocs/Gameboy_Printer.html)
    bool gotDataEndPacket = false;
    bool gotPrintCommand = false;
    uint8_t copies = 0,
            margin_top = 0,
            margin_bottom = 0,
            palette = 0,
            exposure = 0;
    while (lines.size() > 0) {
        std::string line = lines.front();
        lines.pop();

        if (line[0] == '%') {
            deserializeJson(doc, line.c_str() + 1);
            if (doc["command"].as<std::string>() == "INIT") {
                tiles.clear();
                Serial.println("Init command, buffers and state cleared");
            } else if (doc["command"].as<std::string>() == "D") {
                if (doc["end"] == 1) {
                    Serial.printf("Data end, preparing to print, resolution 160x%i\n", tiles.size() / 20 * 8);
                    gotDataEndPacket = true;
                } else {
                    Serial.printf("Reading %i more bytes of data (%i tiles)\n", doc["len"].as<int>(), doc["len"].as<int>() / 16);
                }
            } else if (doc["command"].as<std::string>() == "P") {
                Serial.printf("Printing image\n  Upper Margin: %i\n  Lower Margin: %i\n        Copies: %i\n      Palettes: %i\n      Exposure: %i\n", doc["mrgnupr"].as<int>(), doc["mrgnlwr"].as<int>(), doc["shts"].as<int>(), doc["palette"].as<int>(), doc["dnsty"].as<int>());
                copies = doc["shts"].as<int>();
                margin_top = doc["mrgnupr"].as<int>();
                margin_bottom = doc["mrgnlwr"].as<int>();
                palette = doc["palette"].as<int>();
                exposure = 1 + ((doc["dnsty"].as<double>() - 64) / 63.0f * 0.25);  // tested with this equation in google: 1 + ((0-64)/63*0.25)
                                                                                   // it could be better but it's off by .004 (.4%) which will not matter hardly at all
            }
        } else {
            imageTile tile;
            unsigned int outLen = 0;
            tiles.push_back(tile);
            line[line.length() - 1] = 0;
            HexStringToBytes(line.c_str() + 1, tiles.back().data, &outLen);
        }

        doc.clear();
    }

    int im_x = 160, im_y = tiles.size() / 20 * 8;
    int tile_rows = tiles.size() / 20;
    Serial.printf("Processing %i tiles into array (%ix%i tiles)\n", tiles.size(), 20, tiles.size() / 20);
    Serial.printf("Array of %.3fKiB (%ix%ipx)\n", im_x * im_y / 1024.0, im_x, im_y);

    byte image[im_x * im_y] = {0};

    // Serial.println("--- START PGM ---");
    // Serial.println("P2");
    // Serial.printf("%i %i\n", im_x, im_y);
    // Serial.println("3");

    for (int tn = 0; tn < tiles.size(); tn++) {
        int tile_x = tn % 20;
        int tile_y = tn / 20;
        int x_offset = tile_x * 8;
        int y_offset = tile_y * 8;

        for (int i = 0; i < 16; i += 2) {
            byte low = tiles[tn].data[i];
            byte high = tiles[tn].data[i + 1];

            int cur_y = y_offset + (i / 2);

            for (int j = 0; j < 8; j++) {
                image[cur_y * im_x + x_offset + j] = (byte)(((low >> (7 - j)) & 0b1) | (((high >> (7 - j)) & 0b1) << 1));
            }
        }
    }

    headStepperMoveTo(10);
    paperStepper.step(PAPER_STEPS_PER_PIXEL * 8 * margin_top);
    headStepperMoveTo(HEAD_STEPPER_LEFT_MARGIN);
    for (int y = 0; y < im_y; y += 4) {
        Serial.printf("Lines %i-%i...", y, y+3);
        for (int x = 0; x < im_x; x++) {
            for (int i = 0; i < 2; i++) {
                run8Burners4Pixels(image[(y + 3) * im_x + x], image[(y + 2) * im_x + x], image[(y + 1) * im_x + x], image[y * im_x + x]);
                delayMicroseconds(1000);  // Let head cool for next pixel
                headStepperMoveRel(2);
            }
        }
        headStepperMoveTo(10);
        paperStepper.step(PAPER_STEPS_PER_PIXEL * 8);
        headStepperMoveTo(HEAD_STEPPER_LEFT_MARGIN);
        Serial.printf(" Complete! (%.3f%%)\n", (float)y/im_y*100);
    }
    headStepperMoveTo(0);
    paperStepper.step(PAPER_STEPS_PER_PIXEL * 8 * margin_bottom);
    // Serial.println("\n--- END PGM ---");

    Serial.write("Send print finish notif\n");
    pinMode(PRINT_FINISH_PIN, HIGH);
    delay(1);  // So that the nano can detect
    pinMode(PRINT_FINISH_PIN, LOW);
    Serial.write("Send print finish notif done\n");
}