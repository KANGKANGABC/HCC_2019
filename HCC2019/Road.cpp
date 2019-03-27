#include "Road.h"



Road::Road()
{

}


Road::~Road()
{
	delete[] this->lane;
}

void Road::CreateLane()
{
	// ————————
	//     3
	//  <— — — —
	//     2
	//——————————
	//     0
	//  — — — —>
	//     1
	//——————————
	this->lane = new Lane[(1 + this->isDuplex) * this->channel];
	for (int i = 0; i < this->channel; ++i)
	{
		lane[i].dir = FORWARD;//正向
		lane[i].idLane = i;//id按照升序，先正向的原则（升序符合调度的顺序）
	}
	if (this->isDuplex)
	{
		for (int i = this->channel; i < 2 * this->channel; ++i)
		{
			lane[i].dir = BACKWARD;//逆向
			lane[i].idLane = i;//id按照升序，先正向的原则（升序符合调度的顺序）
		}
	}
}
