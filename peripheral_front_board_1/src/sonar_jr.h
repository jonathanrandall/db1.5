#ifndef SONAR_JR_
#define SONAR_JR_

#include <Arduino.h>

#define MAX_DISTANCE 400
#define MIN_DISTANCE 2

class sonar_jr
{
  //  private:
  //    const float soundcm = 0.0343; //cm/mirosecond
public:
  int trig_pin;
  int echo_pin;
  //    float distance;
  float get_distance();
  float get_distance1();
  sonar_jr(int trig, int ech);
};

sonar_jr::sonar_jr(int trig, int ech) : trig_pin(trig), echo_pin(ech)
{
  pinMode(trig_pin, OUTPUT);         // Sets the trigPin as an Output
  pinMode(echo_pin, INPUT_PULLDOWN); // Sets the echoPin as an Input
}

float sonar_jr::get_distance1()
{
  float d1, d2, d3, distance;
  float d_max;
  float d_min;
  float ds[3];
  d1 = get_distance();
  d_max = d1;
  d_min = d1;
  delay(20);
  d2 = get_distance();
  if (d2 < d_min)
    d_min = d2;
  else
    d_max = d2;
  delay(20);
  d3 = get_distance();
  if (d3 > d_max)
    d_max = d3;
  if (d3 < d_min)
    d_min = d3;
  if (d3 > d_min && d3 < d_max)
    return d3;
  if (d2 > d_min && d2 < d_max)
    return d2;
  if (d1 > d_min && d1 < d_max)
    return d1;
  // if(distance>MIN_DISTANCE && distance < MAX_DISTANCE) return distance;
  return (d_max+d_min)/2;
}

float sonar_jr::get_distance()
{
  long duration;
  float distance;

  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo_pin, HIGH, 30000);
  //  Serial.println(duration);
  // Calculate the distance
  distance = ((float)duration) * 0.0343 / 2;
  if(distance>MIN_DISTANCE && distance < MAX_DISTANCE) return distance;
  return 0;
}

void start_sonar_jr(sonar_jr sjr)
{
  pinMode(sjr.trig_pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(sjr.echo_pin, INPUT);  // Sets the echoPin as an Input
}

#endif
