#include "rf-fans.h"


#define BASE_TOPIC RHINE_BASE_TOPIC

#define CMND_BASE_TOPIC CMND_TOPIC BASE_TOPIC
#define STAT_BASE_TOPIC STAT_TOPIC BASE_TOPIC

#define SUBSCRIBE_TOPIC_CMND CMND_BASE_TOPIC "/#"

#define SUBSCRIBE_TOPIC_STAT_SETUP STAT_BASE_TOPIC "/#"

#ifndef RHINE_TX_FREQ
#define TX_FREQ 303.875 // Rhine CHQ7262T6 https://fcc.report/FCC-ID/CHQ7262T6/6261954
#else
#define TX_FREQ RHINE_TX_FREQ
#endif

// RC-switch settings
#define RF_PROTOCOL 15
#define RF_REPEATS  7

#define DIP_COUNT 2
// Keep track of states for all dip settings
static fan fans[DIP_COUNT];

//  370,  333, 285, 250, 200
static int colorTempList[5] = { 370, 333, 285, 250, 200 };

static unsigned long lastvalue;
static unsigned long lasttime;

static void postStateUpdate(int id) {
    char outNumber[100];
    sprintf(outTopic, "%s/%s/direction", STAT_BASE_TOPIC, idStrings[id]);
    client.publish(outTopic, fans[id].directionState ? "REVERSE" : "FORWARD", true);
    sprintf(outTopic, "%s/%s/fan", STAT_BASE_TOPIC, idStrings[id]);
    client.publish(outTopic, fans[id].fanState ? "ON" : "OFF", true);
    sprintf(outTopic, "%s/%s/speed", STAT_BASE_TOPIC, idStrings[id]);
#ifndef RHINE_6SPEED
    client.publish(outTopic, fanStateTable[fans[id].fanSpeed], true);
#else
    client.publish(outTopic, fanFullStateTable[fans[id].fanSpeed], true);
#endif
    sprintf(outTopic, "%s/%s/light", STAT_BASE_TOPIC, idStrings[id]);
    client.publish(outTopic, fans[id].lightState ? "ON" : "OFF", true);

    sprintf(outTopic, "%s/%s/color_temp", STAT_BASE_TOPIC, idStrings[id]);
    sprintf(outNumber, "%i", fans[id].colorTemp);
    client.publish(outTopic, outNumber, true);

    sprintf(outTopic, "%s/%s/brightness", STAT_BASE_TOPIC, idStrings[id]);
    sprintf(outNumber, "%i", fans[id].brightness);
    client.publish(outTopic, outNumber, true);

    sprintf(outTopic, "%s/%s/percent", STAT_BASE_TOPIC, idStrings[id]);
    *outPercent = '\0';
    if (fans[id].fanState) {
        switch (fans[id].fanSpeed) {
            case FAN_VI:
                sprintf(outPercent, "%d", FAN_PCT_VI);
                break;
            case FAN_V:
                sprintf(outPercent, "%d", FAN_PCT_V);
                break;
            case FAN_IV:
                sprintf(outPercent, "%d", FAN_PCT_IV);
                break;
            case FAN_III:
                sprintf(outPercent, "%d", FAN_PCT_III);
                break;
            case FAN_II:
                sprintf(outPercent, "%d", FAN_PCT_II);
                break;
            case FAN_I:
                sprintf(outPercent, "%d", FAN_PCT_I);
                break;
        }
    } else {
        sprintf(outPercent, "%d", FAN_PCT_OFF);
    }
    client.publish(outTopic, outPercent, true);
}

static void transmitState(int fanId, int code) {
    mySwitch.disableReceive();         // Receiver off
    ELECHOUSE_cc1101.setMHZ(TX_FREQ);
    ELECHOUSE_cc1101.SetTx();           // set Transmit on
    mySwitch.enableTransmit(TX_PIN);   // Transmit on
    mySwitch.setRepeatTransmit(RF_REPEATS); // transmission repetitions.
    mySwitch.setProtocol(RF_PROTOCOL);        // send Received Protocol

    // Build out RF code
    //   Code follows the 24 bit pattern
    //   0000 1110 /// Fan direction summer
    //   0000 1111 /// Fan direction winter
    //   0000 1010 /// Fan on/off
    //   0000 0110 /// Fan Speed - I
    //   0000 0101 /// Fan Speed - II
    //   0000 0100 /// Fan Speed - III
    //   0000 0011 /// Fan Speed - IV
    //   0000 0010 /// Fan Speed - V
    //   0000 0001 /// Fan Speed - VI
    //
    //   0001 1011 /// Light on/off
    //   0001 1100 /// Light Brightness Higher
    //   0001 1101 /// Light Brightness Lower
    //
    //   0010 0001 /// Timer 2 hour
    //   0010 0010 /// Timer 4 hour
    //   0010 0011 /// Timer 8 hour
    //   0011 0000 /// Fan Breeze mode
    //   0100 1011 /// Light Color Temp
    //
    //   1001 1010 /// Power

    int rfCode = (fanId & 0x0f) << 8 | (code & 0xff);
    if (fanId != 0)
        rfCode = 0x4caf00 | rfCode;

    mySwitch.send(rfCode, 24);      // send 24 bit code
    mySwitch.disableTransmit();   // set Transmit off
    ELECHOUSE_cc1101.setMHZ(RX_FREQ);
    ELECHOUSE_cc1101.SetRx();      // set Receive on
    mySwitch.enableReceive(RX_PIN);   // Receiver on
#ifdef DEBUG
    Serial.print("Sent command hamptonbay4: ");
    Serial.print(fanId);
    Serial.print(" ");
    for(int b=24; b>0; b--) {
      Serial.print(bitRead(rfCode,b-1));
    }
    Serial.println("");
#endif
    postStateUpdate(fanId);
}

static int findClosest(int target, int &closestIndex) {
    int closest = colorTempList[0];
    closestIndex = 0; // Index of the closest number
    int diff = abs(closest - target);

    for (int i = 1; i < 5; i++) {
        int newDiff = abs(colorTempList[i] - target);
        if (newDiff < diff) {
            diff = newDiff;
            closest = colorTempList[i];
            closestIndex = i;
        }
    }

    return closest;
}

static int findRotationalDistance(int current, int closestIndex) {
    int currentIndex = -1;

    // Find the indexes
    for (int i = 0; i < 5; i++) {
        if (colorTempList[i] == current) currentIndex = i;
    }

    int distance = 0;
    while (currentIndex != closestIndex) {
        currentIndex = (currentIndex + 1) % 5;
        distance++;
    }
    return distance;
}

static void processCommandTopics(char *topic, char *payloadChar, unsigned int length) {
    if (strncmp(topic, CMND_BASE_TOPIC, sizeof(CMND_BASE_TOPIC) - 1) == 0) {
        // Get ID after the base topic + a slash
        char id[5];
        int percent;
        memcpy(id, &topic[sizeof(CMND_BASE_TOPIC)], 4);
        id[4] = '\0';
        if (strspn(id, idchars)) {
            uint8_t idint = strtol(id, (char **) NULL, 2);
            char *attr;
            // Split by slash after ID in topic to get attribute and action
            attr = strtok(topic + sizeof(CMND_BASE_TOPIC) - 1 + 5, "/");

            if (strcmp(attr, "percent") == 0) {
                percent = atoi(payloadChar);
                if (percent > FAN_PCT_OVER) {
                    fans[idint].fanState = true;
                    if (percent > (FAN_PCT_V + FAN_PCT_OVER)) {
                        fans[idint].fanSpeed = FAN_VI;
                        transmitState(idint, 0x01);
                    } else if (percent > (FAN_PCT_IV + FAN_PCT_OVER)) {
                        fans[idint].fanSpeed = FAN_V;
                        transmitState(idint, 0x02);
                    } else if (percent > (FAN_PCT_III + FAN_PCT_OVER)) {
                        fans[idint].fanSpeed = FAN_IV;
                        transmitState(idint, 0x03);
                    } else if (percent > (FAN_PCT_II + FAN_PCT_OVER)) {
                        fans[idint].fanSpeed = FAN_III;
                        transmitState(idint, 0x04);
                    } else if (percent > (FAN_PCT_I + FAN_PCT_OVER)) {
                        fans[idint].fanSpeed = FAN_II;
                        transmitState(idint, 0x05);
                    } else {
                        fans[idint].fanSpeed = FAN_I;
                        transmitState(idint, 0x06);
                    }
                } else {
                    fans[idint].fanState = false;
                    transmitState(idint, 0x0a);
                }
            } else if (strcmp(attr, "fan") == 0) {
                if (strcmp(payloadChar, "toggle") == 0) {
                    if (fans[idint].fanState)
                        strcpy(payloadChar, "off");
                    else
                        strcpy(payloadChar, "on");
                }
                if (strcmp(payloadChar, "on") == 0) {
                    fans[idint].fanState = true;
                    switch (fans[idint].fanSpeed) {
                        case FAN_BREEZE:
                            transmitState(idint, 0x30);
                            break;
                        case FAN_VI:
                            transmitState(idint, 0x01);
                            break;
                        case FAN_V:
                            transmitState(idint, 0x02);
                            break;
                        case FAN_IV:
                            transmitState(idint, 0x03);
                            break;
                        case FAN_III:
                            transmitState(idint, 0x04);
                            break;
                        case FAN_II:
                            transmitState(idint, 0x05);
                            break;
                        case FAN_I:
                            transmitState(idint, 0x06);
                            break;
                    }
                } else {
                    fans[idint].fanState = false;
                    transmitState(idint, 0x0a);
                }
            } else if (strcmp(attr, "speed") == 0) {
                if (strcmp(payloadChar, "+") == 0) {
                    fans[idint].fanState = true;
                    switch (fans[idint].fanSpeed) {
                        case FAN_I:
                            fans[idint].fanSpeed = FAN_II;
                            break;
                        case FAN_II:
                            fans[idint].fanSpeed = FAN_III;
                            break;
                        case FAN_III:
                            fans[idint].fanSpeed = FAN_IV;
                            break;
                        case FAN_IV:
                            fans[idint].fanSpeed = FAN_V;
                            break;
                        case FAN_V:
                            fans[idint].fanSpeed = FAN_VI;
                            break;
                        case FAN_VI:
                            fans[idint].fanSpeed = FAN_VI;
                            break;
                        default:
                            if (fans[idint].fanSpeed < FAN_VI)
                                fans[idint].fanSpeed = FAN_VI;
                            if (fans[idint].fanSpeed > FAN_I)
                                fans[idint].fanSpeed = FAN_I;
                            break;
                    }
                } else if (strcmp(payloadChar, "-") == 0) {
                    fans[idint].fanState = true;
                    switch (fans[idint].fanSpeed) {
                        case FAN_I:
                            fans[idint].fanSpeed = FAN_I;
                            break;
                        case FAN_II:
                            fans[idint].fanSpeed = FAN_I;
                            break;
                        case FAN_III:
                            fans[idint].fanSpeed = FAN_II;
                            break;
                        case FAN_IV:
                            fans[idint].fanSpeed = FAN_III;
                            break;
                        case FAN_V:
                            fans[idint].fanSpeed = FAN_IV;
                            break;
                        case FAN_VI:
                            fans[idint].fanSpeed = FAN_V;
                            break;
                        default:
                            if (fans[idint].fanSpeed < FAN_VI)
                                fans[idint].fanSpeed = FAN_VI;
                            if (fans[idint].fanSpeed > FAN_I)
                                fans[idint].fanSpeed = FAN_I;
                            break;
                    }
                } else if (strcmp(payloadChar, "high") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_HI;
                    transmitState(idint, 0x01);
                } else if (strcmp(payloadChar, "medium") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_MED;
                    transmitState(idint, 0x04);
                } else if (strcmp(payloadChar, "low") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_LOW;
                    transmitState(idint, 0x06);
                } else if (strcmp(payloadChar, "i") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_I;
                    transmitState(idint, 0x06);
                } else if (strcmp(payloadChar, "ii") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_II;
                    transmitState(idint, 0x05);
                } else if (strcmp(payloadChar, "iii") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_III;
                    transmitState(idint, 0x04);
                } else if (strcmp(payloadChar, "iv") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_IV;
                    transmitState(idint, 0x03);
                } else if (strcmp(payloadChar, "v") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_V;
                    transmitState(idint, 0x02);
                } else if (strcmp(payloadChar, "vi") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_VI;
                    transmitState(idint, 0x01);
                } else if (strcmp(payloadChar, "breeze") == 0) {
                    fans[idint].fanState = true;
                    fans[idint].fanSpeed = FAN_BREEZE;
                    transmitState(idint, 0x30);
                } else {
                    fans[idint].fanState = false;
                    transmitState(idint, 0x0a);
                }
            } else if (strcmp(attr, "light") == 0) {
                if (strcmp(payloadChar, "toggle") == 0) {
                    if (fans[idint].lightState)
                        strcpy(payloadChar, "off");
                    else
                        strcpy(payloadChar, "on");
                }
                if (strcmp(payloadChar, "on") == 0 && !fans[idint].lightState) {
                    fans[idint].lightState = true;
                    transmitState(idint, 0x1b);
                } else if (strcmp(payloadChar, "off") == 0 && fans[idint].lightState) {
                    fans[idint].lightState = false;
                    transmitState(idint, 0x1b);
                }
            } else if (strcmp(attr, "direction") == 0) {
                if (strcmp(payloadChar, "toggle") == 0) {
                    if (fans[idint].directionState)
                        strcpy(payloadChar, "forward");
                    else
                        strcpy(payloadChar, "reverse");
                }
                if (strcmp(payloadChar, "reverse") == 0 && !fans[idint].directionState) {
                    fans[idint].directionState = true;
                    transmitState(idint, 0x0f);
                } else if (strcmp(payloadChar, "forward") == 0 && fans[idint].directionState) {
                    fans[idint].directionState = false;
                    transmitState(idint, 0x0e);
                }
            } else if (strcmp(attr, "brightness") == 0) {
                int requestedBrightness = atoi(payloadChar);
                int diff = (requestedBrightness - fans[idint].brightness) / 10;
                int diffSign = diff < 0 ? -1 : 1;
                int runCount = abs(diff)+1;
                int command = diff < 0 ? 0x1d : 0x1c;
                int newBrightness = fans[idint].brightness;

                for (int i = 0; i < runCount; ++i) {
                    newBrightness = fans[idint].brightness + (10 * diffSign);
                    if(newBrightness < 0) newBrightness = 0;
                    if(newBrightness > 255) newBrightness = 255;

                    fans[idint].brightness = newBrightness;
                    transmitState(idint, command);
                    delay(50);
                }
            } else if (strcmp(attr, "color_temp") == 0) {
                int requestedColorTemp = atoi(payloadChar);

                int closestIndex = 0, currentIndex = 0;
                findClosest(requestedColorTemp, closestIndex);
                int distance = findRotationalDistance(fans[idint].colorTemp, closestIndex);

                // 2700, 3000, 3500, 4000, 5000
                //  370,  333, 285, 250, 200
                for (int i = 0; i < 5; i++) {
                    if (colorTempList[i] == fans[idint].colorTemp) currentIndex = i;
                }
                for (int i = 0; i < distance; ++i) {
                    currentIndex = (currentIndex + 1) % 5;
                    fans[idint].colorTemp = colorTempList[currentIndex];
                    transmitState(idint, 0x4b);
                    delay(1000); // This fan only processes color changes once a second
                }
            }
        } else {
            // Invalid ID
            return;
        }
    }
}

static void processStatTopics(char *topic, char *payloadChar, unsigned int length) {
    if (strncmp(topic, STAT_BASE_TOPIC, sizeof(STAT_BASE_TOPIC) - 1) == 0) {
        // Get ID after the base topic + a slash
        char id[5];
        memcpy(id, &topic[sizeof(STAT_BASE_TOPIC)], 4);
        id[4] = '\0';
        if (strspn(id, idchars)) {
            uint8_t idint = strtol(id, (char **) NULL, 2);
            char *attr;
            // Split by slash after ID in topic to get attribute and action
            attr = strtok(topic + sizeof(STAT_BASE_TOPIC) - 1 + 5, "/");

            if (strcmp(attr, "fan") == 0) {
                if (strcmp(payloadChar, "on") == 0) {
                    fans[idint].fanState = true;
                } else {
                    fans[idint].fanState = false;
                }
            } else if (strcmp(attr, "speed") == 0) {
                if (strcmp(payloadChar, "high") == 0) {
                    fans[idint].fanSpeed = FAN_HI;
                } else if (strcmp(payloadChar, "medium") == 0) {
                    fans[idint].fanSpeed = FAN_MED;
                } else if (strcmp(payloadChar, "low") == 0) {
                    fans[idint].fanSpeed = FAN_LOW;
                } else if (strcmp(payloadChar, "i") == 0) {
                    fans[idint].fanSpeed = FAN_I;
                } else if (strcmp(payloadChar, "ii") == 0) {
                    fans[idint].fanSpeed = FAN_II;
                } else if (strcmp(payloadChar, "iii") == 0) {
                    fans[idint].fanSpeed = FAN_III;
                } else if (strcmp(payloadChar, "iv") == 0) {
                    fans[idint].fanSpeed = FAN_IV;
                } else if (strcmp(payloadChar, "v") == 0) {
                    fans[idint].fanSpeed = FAN_V;
                } else if (strcmp(payloadChar, "vi") == 0) {
                    fans[idint].fanSpeed = FAN_VI;
                } else if (strcmp(payloadChar, "breeze") == 0) {
                    fans[idint].fanSpeed = FAN_BREEZE;
                }
            } else if (strcmp(attr, "light") == 0) {
                if (strcmp(payloadChar, "on") == 0) {
                    fans[idint].lightState = true;
                } else {
                    fans[idint].lightState = false;
                }
            } else if (strcmp(attr, "direction") == 0) {
                if (strcmp(payloadChar, "reverse") == 0) {
                    fans[idint].directionState = true;
                } else {
                    fans[idint].directionState = false;
                }
            } else if (strcmp(attr, "brightness") == 0) {
                fans[idint].brightness = atoi(payloadChar);
            } else if (strcmp(attr, "color_temp") == 0) {
                fans[idint].colorTemp = atoi(payloadChar);
            }
        } else {
            // Invalid ID
            return;
        }
    }
}

void rhineMQTT(char *topic, char *payloadChar, unsigned int length) {
    processCommandTopics(topic, payloadChar, length);
    processStatTopics(topic, payloadChar, length);
}

void rhineRF(unsigned long value, int prot, int bits) {
#ifdef DEBUG
    Serial.print("Recv command hamptonbay4: ");
    Serial.print(prot);
    Serial.print(" - ");
    Serial.print(value);
    Serial.print(" - ");
    Serial.print(bits);
    Serial.print("  :  ");
    for(int b=24; b>0; b--) {
      Serial.print(bitRead(value,b-1));
    }
    Serial.println();
#endif

    if (prot == 6 && bits == 24 && ((value & 0xfff000) == 0x4ca000 || (value & 0xfff000) == 0)) {
        unsigned long t = millis();
        if (value == lastvalue) {
            if (t - lasttime < NO_RF_REPEAT_TIME)
                return;
            lasttime = t;
        }
        lastvalue = value;
        lasttime = t;
        Serial.print("--- processing command - ");
        Serial.println(value);

        int dipId = (value >> 7) & 0x0f;
        int brightness = 0;
        int currentIndex = 0;
        // Got a correct id in the correct protocol
        if (dipId < DIP_COUNT) {
            // Convert to dip id
            if ((value & 0x80) == 0x80)
                fans[dipId].powerState = false;
            else
                fans[dipId].powerState = true;
            switch (value & 0x3f) {
                case 0x0e: // Direction
                    fans[dipId].directionState = true;
                    break;
                case 0x0f: // Direction
                    fans[dipId].directionState = false;
                    break;
                case 0x1b: // Light
                    fans[dipId].lightState = !(fans[dipId].lightState);
                    break;
                case 0x06: // Fan I
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_I;
                    break;
                case 0x05: // Fan II
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_II;
                    break;
                case 0x04: // Fan III
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_III;
                    break;
                case 0x03: // Fan IV
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_IV;
                    break;
                case 0x02: // Fan V
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_V;
                    break;
                case 0x01: // Fan VI
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_VI;
                    break;
                case 0x30: // Fan Breeze mode
                    fans[dipId].fanState = true;
                    fans[dipId].fanSpeed = FAN_BREEZE;
                    break;
                case 0x0a: // Fan Off
                    fans[dipId].fanState = false;
                    break;
                case 0x1c: // Increase Brightness
                    brightness = fans[dipId].brightness++;
                    if (brightness <= 255)
                        fans[dipId].brightness = brightness;
                    break;
                case 0x1d: // Decrease Brightness
                    brightness = fans[dipId].brightness--;
                    if (brightness >= 0)
                        fans[dipId].brightness = brightness;
                    break;
                case 0x0b: // color temp
                    for (int i = 0; i < 5; i++) {
                        if (colorTempList[i] == fans[dipId].colorTemp) currentIndex = i;
                    }
                    currentIndex = (currentIndex + 1) % 5;
                    fans[dipId].colorTemp = colorTempList[currentIndex];
                    break;
                default:
                    break;
            }
            postStateUpdate(dipId);
        }
    }
}

void rhineMQTTSub(boolean setup) {
    client.subscribe(SUBSCRIBE_TOPIC_CMND);

    if (setup) client.subscribe(SUBSCRIBE_TOPIC_STAT_SETUP);
}

void rhineSetup() {
    lasttime = 0;
    lastvalue = 0;
    // initialize fan struct
    for (int i = 0; i < DIP_COUNT; i++) {
        fans[i].powerState = false;
        fans[i].fade = false;
        fans[i].directionState = false;
        fans[i].lightState = false;
        fans[i].fanState = false;
        fans[i].fanSpeed = FAN_LOW;
        fans[i].colorTemp = 0;
        fans[i].brightness = 255;
    }
}

void rhineSetupEnd() {
    client.unsubscribe(SUBSCRIBE_TOPIC_STAT_SETUP);
}
