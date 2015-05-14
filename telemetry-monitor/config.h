/**
 * @file config.h
 * Contains definitions of CAN messages and the scalar values for each message.
 *
 * @author Andrew Mass
 * @date Created: 2014-04-27
 * @date Modified: 2014-05-06
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <QString>

using std::map;
using std::pair;
using std::make_pair;

#define NUM_BARS 2
#define NUM_METRICS 4

/*
 * Structure that represents a CAN message. Contains information about the
 * ID of the message (our ID, not the official CAN bus ID), the title of the
 * message, a string representing the units, and a scalar for the message.
 */
struct message {
  unsigned char id;
  QString title;
  QString units;
  double scalar;
  int minBar;
  int maxBar;
  int danger_low;
  int danger_high;

  message();

  message(unsigned char i, const QString & t, const QString & u, double s,
      int miB, int maB, int dl, int dh) {
    id = i;
    title = t;
    units = u;
    scalar = s;
    minBar = miB;
    maxBar = maB;
    danger_low = dl;
    danger_high = dh;
  }
};

/*
 * Initializes a map from message IDs to message structs that will have all
 * the configuation for different CAN messages that will be sent over the
 * telemetry link.
 */
inline map<unsigned char, message> init_map() {
  map<unsigned char, message> can_map;
  can_map.insert(make_pair(0x22, message(0x22, "Engine Temp", "C", 0.1, 0, 130,0, 115)));
  can_map.insert(make_pair(0x11, message(0x11, "Oil Temp", "C", 0.1, 0, 180,0, 140)));
  can_map.insert(make_pair(0x44, message(0x44, "Oil Pressure", "psi", 0.1, 14.5, 110, 15, 105)));
  can_map.insert(make_pair(0x66, message(0x66, "RPM", "rpm", 1.0, 0, 15000, -1, 13600)));
  can_map.insert(make_pair(0x33, message(0x33, "Voltage", "V", 0.01, 9, 16, 9, 15)));
  can_map.insert(make_pair(0x55, message(0x55, "Speed", "MPH", 0.1, 0, 100, -1, 85)));
  can_map.insert(make_pair(0x77, message(0x77, "Lambda", "La", .001, 0, 10, .1, 7)));
  return can_map;
}

/*
 * Gives the id of the CAN message to display at each location. If more
 * metrics or bars are added, the id of the CAN message to display at each
 * location will have to be added here.
 *
 * Order is as follows:
 * left0, left1, left2, left3, right0, right1, right2, right3
 *
 * Example:
 * If the "Oil Temp" message was supposed to be displayed at the second metric
 * location on the right side, metric_indices[5] = 0x02.
 */
static const unsigned char metric_ids[NUM_METRICS * 2] = {0x11, 0x22,
  0x33, 0x44, 0x55, 0x66, 0x77, 0x22};

/*
 * Similiar to above, but for the bars on the bottom of the screen.
 *
 * Order is as follows:
 * bar0, bar1
 */
static const unsigned char bar_ids[NUM_BARS] = {0x11, 0x22};

#endif // CONFIG_H
