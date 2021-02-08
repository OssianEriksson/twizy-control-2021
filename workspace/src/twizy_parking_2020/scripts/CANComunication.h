
#include <canlib.h>
#include "canstat.h"

class CANComunication
{
	private:
	char angle_1;
	char angle_2;
	char throttle;
	char direction;
	canHandle hnd;
	canStatus stat;
	public:
	int speed;
	int steer;
	int StartCan();
	int SendMessage();
	int StopCan();

};
