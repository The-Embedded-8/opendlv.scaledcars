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

        //----------------------------------------------------------------------------------//


        //----------------------------------------------------------------------------------//


        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {
           // const double ULTRASONIC_FRONT_CENTER = 3;
          // const double ULTRASONIC_FRONT_RIGHT = 4;
            const double INFRARED_FRONT_RIGHT = 0;
      //      const double INFRARED_REAR_RIGHT = 2;
        //    const double INFRARED_REAR = 1;
            const double ODOMETER = 5;

            bool isObstacle = true;
            bool isParking = false;

            int irFrontSide;
          //  int irBackSide;
           // int irBack;
            int odometer;
            int oldOdometer = 0;


            //double distanceOld = 0;
            //double absPathStart = 0;
            //double absPathEnd = 0;

            //int stageMoving = 0;
            //int stageMeasuring = 0;


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                irFrontSide = (int)sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
   //             irBackSide = (int)sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
     //           irBack = (int)sbd.getValueForKey_MapOfDistances(INFRARED_REAR);
                odometer = (int)sbd.getValueForKey_MapOfDistances(ODOMETER);

                // Create vehicle control data.
                VehicleControl vcparker;

                // when the car is detecting obstacle how it should behave
                if(isObstacle){
                	// when the car don't detect obstacle anymore change the "isObstacle" to false
                	if(irFrontSide > 1){
                		isObstacle = false;
                	}
                	vcparker.setSteeringWheelAngle(80);
                	vcparker.setSpeed(2);
                }
                // when the car don't dectect obstacle it in the stage where checking for parking space is free or not
                else if(!isObstacle){
                	oldOdometer = startMeasuringGap(odometer,oldOdometer);
                	// tell the car that the gap is enough for parking so tell it to park
                	if(getMeasuringGap() >= 9 && !isParking){
                		isParking = true;
                		stopMeasuringGap();
                	}
                	// there is an obstacle and parking gap is not enough for you
                	else if(irFrontSide < 2 && !isParking){
                		isObstacle = true;
                		stopMeasuringGap();
                		vcparker.setSteeringWheelAngle(0);
                		vcparker.setSpeed(2);
                	}
                	// in the parking stage  /// Angle right= 120, normal = 80, left = 30
                	else if(isParking){
                		if(getMeasuringGap() < 6){
                			vcparker.setSteeringWheelAngle(120);
                			vcparker.setSpeed(2);
                		}
                		else if(getMeasuringGap() >= 6  && getMeasuringGap() < 9) {
                			vcparker.setSteeringWheelAngle(30);
                			vcparker.setSpeed(2);
                		}
                		else if(getMeasuringGap() > 8) {
                		    stopMeasuringGap();
                		}
                	}
                	// continue moving forward
                	else{
                		vcparker.setSteeringWheelAngle(80);
                		vcparker.setSpeed(2);
                	}
                }

	       // cout << "StageMachine" <<stageMoving << endl;
                // Moving state machine.
                //if (stageMoving == 0) {
                    // Go forward.
                   // vcparker.setSpeed(2);
                   // vcparker.setSteeringWheelAngle(0);
                //}
//                if ((stageMoving > 0) && (stageMoving < 40)) {
//                    // Move slightly forward.
//                    vcparker.setSpeed(.4);
//                    vcparker.setSteeringWheelAngle(0);
//                    stageMoving++;
//                }
//                if ((stageMoving >= 40) && (stageMoving < 45)) {
//                    // Stop.
//                    vcparker.setSpeed(0);
//                    vcparker.setSteeringWheelAngle(0);
//                    stageMoving++;
//                }
//                if ((stageMoving >= 45) && (stageMoving < 85)) {
//                    // Backwards, steering wheel to the right.
//                    vcparker.setSpeed(-1.6);
//                    vcparker.setSteeringWheelAngle(25);
//                    stageMoving++;
//                }
//                if ((stageMoving >= 85) && (stageMoving < 220))  {
//                    // Backwards, steering wheel to the left.
//		    cout << "Infrared" <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR) << endl;
//                    vcparker.setSpeed(-.175);
//                    vcparker.setSteeringWheelAngle(-25);
//                    stageMoving++;
//                }
//                if (stageMoving >= 220 && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR)<0)) {
//		    cout << "Infrared" <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR) << endl;
//                    vcparker.setSpeed(-.175);
//                    vcparker.setSteeringWheelAngle(0);
//                    stageMoving++;
//		    }
//	         if  (stageMoving >= 220 && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR)>=0)){
//                    // Stop.
//			cout << "Infrared stop" <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR) << endl;
//                    vcparker.setSpeed(0);
//                    vcparker.setSteeringWheelAngle(0);
//			}
//
//                // Measuring state machine.
//                switch (stageMeasuring) {
//                    case 0:
//                        {
//                            // Initialize measurement.
//                            distanceOld = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
//                            stageMeasuring++;
//                        }
//                    break;
//                    case 1:
//                        {
//                            // Checking for sequence +, -.
//                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0)) {
//                                // Found sequence +, -.
//                                stageMeasuring = 2;
//                                absPathStart = vd.getAbsTraveledPath();
//                            }
//                            distanceOld = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
//                        }
//                    break;
//                    case 2:
//                        {
//                            // Checking for sequence -, +.
//                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0)) {
//                                // Found sequence -, +.
//                                stageMeasuring = 1;
//                                absPathEnd = vd.getAbsTraveledPath();
//
//                                const double GAP_SIZE = (absPathEnd - absPathStart);
//
//                                cerr << "Size = " << GAP_SIZE << endl;
//
//                                if ((stageMoving < 1) && (GAP_SIZE > 7)) {
//                                    stageMoving = 1;
//                                }
//                            }
//                            distanceOld = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
//                        }
//                    break;
//                }
                //cout << "ultraFront: " <<sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER)<<endl;
                //cout << "ultraRight: " <<sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT)<<endl;
                cout << "infraRedFrontRight: " <<sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT)<<endl;
     //           cout << "infraRedRearRight: " <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT)<<endl;
     //           cout << "infraRedRear: " <<sbd.getValueForKey_MapOfDistances(INFRARED_REAR)<<endl;
                cout << "Distance: " <<getMeasuringGap()<<endl;
                cout << "odometer: " <<sbd.getValueForKey_MapOfDistances(ODOMETER)<<endl;

               /*if(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) >1 && sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) <5){

			

                	vc.setSpeed(2);

                	vc.setSteeringWheelAngle(-25);
                }
			else vc.setSteeringWheelAngle(25);
*/
                // Create container for finally sending the data.
                Container c(vcparker);
                // Send container.
                getConference().send(c);

            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature}
