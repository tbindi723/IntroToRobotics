// Generates a velocity profile (with checkpoints t1, t2, tf and max speed vel)
// based on max velocity, desired distance, and fraction of ramp-up/ramp-down time
struct velProfile genVelProfile(float vMax, float distance, float rampFraction) {
  struct velProfile p;
  p.vel = vMax;
  p.tf = distance / ((1-rampFraction)*p.vel);
  p.t1 = rampFraction*p.tf;
  p.t2 = (1-rampFraction)*p.tf;
  return p;
}