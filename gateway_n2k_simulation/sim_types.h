#pragma once

// AIS target descriptor used by the simulator.
// Defined here so the Arduino preprocessor's auto-generated function prototypes
// can resolve SimAisTarget before any function body is compiled.
struct SimAisTarget {
  uint32_t mmsi;
  char     name[21];
  double   lat0;
  double   lon0;
  double   sogKt;
  double   cogDeg;
  double   headingDeg;
  uint8_t  shipType;
  bool     classA;
};