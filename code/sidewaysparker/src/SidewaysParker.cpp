/**
 * sidewaysparker - Sample application for realizing a sideways parking car.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <cstdio>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "SidewaysParker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;




        SidewaysParker::SidewaysParker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SidewaysParker") {
        }

        SidewaysParker::~SidewaysParker() {}

        void SidewaysParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void SidewaysParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {
           // const double ULTRASONIC_FRONT_CENTER = 3;
          // const double ULTRASONIC_FRONT_RIGHT = 4;
            const double INFRARED_FRONT_RIGHT = 0;
            const double INFRARED_REAR_RIGHT = 2;
            const double INFRARED_REAR = 1;
            const double ODOMETER = 5;

            bool isObstacle = true;
            bool isParking = false;
            bool isBackObstacle = false;
            //bool isTurnLeft = false; // use for parking

            int irFrontSide;
            //int irBackSide;
            int irBack;
            int odometer;
            int oldOdometer = 0;


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                irFrontSide = (int)sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                //irBackSide = (int)sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                irBack = (int)sbd.getValueForKey_MapOfDistances(INFRARED_REAR);
                odometer = (int)sbd.getValueForKey_MapOfDistances(ODOMETER);

                // Create vehicle control data.
                VehicleControl vcparker;

                // when the car is detecting obstacle how it should behave
                if(isObstacle){
                	// when the car don't detect obstacle anymore change the "isObstacle" to false
                	if(irFrontSide > 1){
                		isObstacle = false;
                	}
                	// trigger car to move
                	if(irBack < 1){
                		vcparker.setSpeed(1);
                		vcparker.setSteeringWheelAngle(2);
                	}else{
                	vcparker.setSteeringWheelAngle(2);
                	vcparker.setSpeed(2);}
                }
                // when the car don't dectect obstacle it in the stage where checking for parking space is free or not
                else if(!isObstacle){
                	oldOdometer = startMeasuringDistance(odometer,oldOdometer);
                	// tell the car that the gap is enough for parking so tell it to park
                	// Gap enough on 6
                	if(getMeasuringDistance() >= 6 && !isParking){
                		isParking = true;
                		resetMeasuringDistance();
                	}
                	// there is an obstacle and parking gap is not enough for you
                	else if(irFrontSide < 2 && !isParking){
                		isObstacle = true;
                		resetMeasuringDistance();
                	}
                	// in the parking stage  /// Angle right = 3 -> 120, straight = 2 -> 82, left = 1 -> 30
                	// Speed idle = 1, forward = 2, backward = 3
                	else if(isParking){
                		/// Have to fix naming of MeasuringGap :: it missunderstanding here
                		// straight forward 2 :: to get car parallel to car beside
                		if(getMeasuringDistance() < 3 && !isBackObstacle){
                			vcparker.setSteeringWheelAngle(2);
                			vcparker.setSpeed(2);
                		}
                		// right backward :: 3 -- 7
                		else if(getMeasuringDistance() >= 3 && getMeasuringDistance() < 8 && !isBackObstacle) {
							vcparker.setSteeringWheelAngle(3);
							vcparker.setSpeed(3);
                		}
				// Straight backward :: 8
                		else if(getMeasuringDistance() >= 8 && getMeasuringDistance() < 9 && !isBackObstacle) {
							vcparker.setSteeringWheelAngle(2);
							vcparker.setSpeed(3);
                		}
                		// left backward :: 9 and irback == 1-7
                		else if(getMeasuringDistance() >= 9 && irBack > 0 && !isBackObstacle) {
                			vcparker.setSteeringWheelAngle(1);
                			vcparker.setSpeed(3);
                		}
                		// stop :: irBack == 0
                		else if(irBack < 1) {
                			vcparker.setSteeringWheelAngle(2);
                			vcparker.setSpeed(1);
                			isBackObstacle = true;
                			resetMeasuringDistance();
                		}
                		// move forward 1 step
                		else if(getMeasuringDistance() < 1 && isBackObstacle) {
                		  	vcparker.setSteeringWheelAngle(2);
                		  	vcparker.setSpeed(2);
                		}
				// stop
                		else if(getMeasuringDistance() >= 1 && isBackObstacle) {
                			vcparker.setSteeringWheelAngle(2);
                			vcparker.setSpeed(1);
                		}


                	}
                	// continue moving forward
                	else{
                		vcparker.setSteeringWheelAngle(2);
                		vcparker.setSpeed(2);
                	}
                }

                //cout << "ultraFront: " <<sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER)<<endl;
                //cout << "ultraRight: " <<sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT)<<endl;
                cout << "infraRedFrontRight: " <<sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT)<<endl;
                cout << "infraRedRearRight: " <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT)<<endl;
                cout << "infraRedRear: " <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR)<<endl;
                cout << "Distance: " <<getMeasuringDistance()<<endl;
                cout << "odometer: " <<sbd.getValueForKey_MapOfDistances(ODOMETER)<<endl;

                // Create container for finally sending the data.
                Container c(vcparker);
                // Send container.
                getConference().send(c);

            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature}
