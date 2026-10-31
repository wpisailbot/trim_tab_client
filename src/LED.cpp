#include "Constants.h"

CRGB leds[NUM_LEDS];

static void setSectionSolid(int sectionIndex, const CRGB &color) {
  const int mid = NUM_LEDS / 2;
  if (sectionIndex < 0 || sectionIndex > 5) {
    return;
  }

  const int totalHalf = (NUM_LEDS - 1) / 2;
  const int baseHalves[5] = {13, 13, 13, 14, 14};
  int sumBase = 0;
  for (int i = 0; i < 5; i++) {
    sumBase += baseHalves[i];
  }
  const int remainingHalf = max(0, totalHalf - sumBase);
  const int halfHeights[6] = {13, 13, 13, 14, 14, remainingHalf};

  if (halfHeights[sectionIndex] == 0) {
    return;
  }
  int offset = 0;
  for (int i = 0; i < sectionIndex; i++) {
    offset += halfHeights[i];
  }

  if (sectionIndex == 0) {
    const int leftStart = max(0, mid - halfHeights[0]);
    const int leftEnd = max(0, mid - 1);
    const int rightStart = min(NUM_LEDS - 1, mid + 1);
    const int rightEnd = min(NUM_LEDS - 1, mid + halfHeights[0]);
    for (int i = leftStart; i <= leftEnd; i++) {
      leds[i] = color;
    }
    for (int i = rightStart; i <= rightEnd; i++) {
      leds[i] = color;
    }
    leds[mid] = color;
    return;
  }

  const int leftStart = max(0, mid - (offset + halfHeights[sectionIndex]));
  const int leftEnd = max(0, mid - (offset + 1));
  const int rightStart = min(NUM_LEDS - 1, mid + (offset + 1));
  const int rightEnd = min(NUM_LEDS - 1, mid + (offset + halfHeights[sectionIndex]));

  for (int i = leftStart; i <= leftEnd; i++) {
    leds[i] = color;
  }
  for (int i = rightStart; i <= rightEnd; i++) {
    leds[i] = color;
  }
}

bool lightLED(bool reach_buoy,bool move_flag,bool found_buoy,bool trim_auto,bool rudder_auto,bool connection_status) {
  static int light_i = 0;
  const CRGB defaultColors[6] = {CRGB::Red,    CRGB::Green,  CRGB::Blue,
                                 CRGB::Yellow, CRGB::Purple, CRGB::Orange};
                          
  // Start with everything off so section colors are obvious.
  fill_solid(leds, NUM_LEDS, CRGB::Black);            
  if(reach_buoy){
    setSectionSolid(0, CRGB::Green);
    setSectionSolid(1, CRGB::Green);
    setSectionSolid(2, CRGB::Green);
    setSectionSolid(3, CRGB::Green);
    setSectionSolid(4, CRGB::Green);
  } else {
    
    if (move_flag) { //eventually do one that goes from green ->yellow -> red for hull battery
      setSectionSolid(0, CRGB::Red);
    } else {
      setSectionSolid(0, CRGB::Black);
    }
    if (found_buoy) {
      setSectionSolid(1, CRGB::Blue);
    } else {
      setSectionSolid(1, CRGB::Black);
    }
    if (trim_auto){
      setSectionSolid(2, CRGB::Green);
    } else {
      setSectionSolid(2, CRGB::Black);
    }
    if (rudder_auto){
      setSectionSolid(3, CRGB::Yellow);
    } else {
      setSectionSolid(3, CRGB::Black);
    }
    if (light_i>10){
      setSectionSolid(4, CRGB::Purple);
      if(light_i > 20) {
        light_i = 0;
      }
    } else {
      setSectionSolid(4, CRGB::Black);
    }
    if (connection_status){
      setSectionSolid(5, CRGB::Orange);
    } else {
      setSectionSolid(5, CRGB::Black);
    }
  }
  FastLED.show();
  light_i++;
  return true;
}

void LED_setup(){
  Serial.println("Setting up LEDs");
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
}