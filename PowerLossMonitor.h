#ifndef POWERLOSSMONITOR_H
#define POWERLOSSMONITOR_H

class PowerLossMonitor
{
	public:
		PowerLossMonitor();
		~PowerLossMonitor();
	
		void SetCountLimit(int countLimit);
		void SetDelayTimeout(int seconds);
		void Reset(int baseCount);
		bool DescentRequired(int powerLossCount);
	
		
		

	protected:
		int countLimitThreshold;
		int powerLossBaseCount;
		int delayMaxCount;
		int delayCount;
		
		bool inDescent;
		
};

#endif //POWERLOSSMONITOR_H

