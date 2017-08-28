#ifndef ROVER_COMMANDER_H_
#define ROVER_COMMANDER_H_

#include <sys/stat.h>


class RoverCommander{

public:

    RoverCommander();


private:

    static constexpr uint8_t COMMANDER_MAX_GPS_NOISE = 60;		/**< Maximum percentage signal to noise ratio allowed for GPS reception */


};



#endif
