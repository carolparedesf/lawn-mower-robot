#ifndef Encoder_h_
#define Encoder_h_

#include "Arduino.h"
#include <ESP32Encoder.h>
class Encoder
{
	private:
		int counts_per_rev_ = -1;
		ESP32Encoder encoder_;
		int64_t prev_ticks_ = 0;
	public:
		Encoder(int pinA, int pinB, int counts_per_rev, bool invert = false)
		{
			if (invert)
			{
				int tmp = pinA;
				pinA = pinB;
				pinB = tmp;
			}
			counts_per_rev_ = counts_per_rev;
			ESP32Encoder::useInternalWeakPullResistors = puType::UP;
			//encoder_.attachHalfQuad(pinA, pinB);
			encoder_.attachSingleEdge(pinA, pinB);
			//encoder_.attachFullQuad(pinA, pinB);
			prev_ticks_ = encoder_.getCount();
		}
float getRadPerSec(float dt)
{
	if (counts_per_rev_< 0) return 0.0f;
	
	int64_t ticks = encoder_.getCount();
	int64_t delta_ticks = ticks - prev_ticks_;
	prev_ticks_ = ticks;
	return (2.0f * PI * delta_ticks)/(counts_per_rev_ * dt);
}
inline int32_t read()
{
	if (counts_per_rev_< 0) return 0;
	return encoder_.getCount();
}
inline void write(int32_t p)
{
	if (counts_per_rev_ < 0) return;
	encoder_.setCount(p);
}
};


#endif
