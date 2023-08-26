#include "PowerLossMonitor.h"



PowerLossMonitor::PowerLossMonitor()
{
}


PowerLossMonitor::~PowerLossMonitor()
{
}



void PowerLossMonitor::SetCountLimit(int countLimit)
{
	countLimitThreshold = countLimit;
}


void PowerLossMonitor::SetDelayTimeout(int seconds)
{
	// Called at 10hz
	delayMaxCount = seconds * 10;
}


void PowerLossMonitor::Reset(int baseCount)
{
	powerLossBaseCount = baseCount;
	delayCount = 0;
	inDescent = false;
}


bool PowerLossMonitor::DescentRequired(int powerLossCount)
{
	bool result = false;
	
	// Check if we are currently in a descent
	if (inDescent == true) 
	{
		delayCount++;
		if (delayCount >= delayMaxCount) 
		{
			// Reset power loss monitor on timeout
			Reset(powerLossCount);			
		}
		result = false;
	}
	else
	{
		if ((powerLossCount - powerLossBaseCount) >= countLimitThreshold)
		{
			inDescent = true;
			result = true;
		}
	}
	
	return result;
}



		
