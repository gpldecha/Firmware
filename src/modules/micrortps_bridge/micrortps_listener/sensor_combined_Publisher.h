// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file sensor_combined_Publisher.h
 * This header file contains the declaration of the publisher functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#ifndef _SENSOR_COMBINED__PUBLISHER_H_
#define _SENSOR_COMBINED__PUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>

#include "sensor_combined_PubSubTypes.h"

using namespace eprosima::fastrtps;

class sensor_combined_Publisher 
{
public:
	sensor_combined_Publisher();
	virtual ~sensor_combined_Publisher();
	bool init();
	void run();
private:
	Participant *mp_participant;
	Publisher *mp_publisher;
	
	class PubListener : public PublisherListener
	{
	public:
		PubListener() : n_matched(0){};
		~PubListener(){};
		void onPublicationMatched(Publisher* pub,MatchingInfo& info);
		int n_matched;
	} m_listener;
	sensor_combined_PubSubType myType;
};

#endif // _SENSOR_COMBINED__PUBLISHER_H_