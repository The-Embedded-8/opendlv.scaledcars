/**
 * lanefollower - Sample application for following lane markings.
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

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

#include "LaneFollower.h"




namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;

        LaneFollower::LaneFollower(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
            m_hasAttachedToSharedImageMemory(false),
            m_sharedImageMemory(),
            m_image(NULL),
            m_debug(false),
            m_font(),
            m_previousTime(),
            m_eSum(0),
            m_eOld(0),
            m_vehicleControl() {}

        LaneFollower::~LaneFollower() {}

        void LaneFollower::setUp() {
            // This method will be call automatically _before_ running body().
            if (m_debug) {
                // Create an OpenCV-window.
                cvNamedWindow("WindowShowImage", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("WindowShowImage", 300, 100);
            }
        }

        void LaneFollower::tearDown() {
            // This method will be call automatically _after_ return from body().
            if (m_image != NULL) {
                cvReleaseImage(&m_image);
            }

            if (m_debug) {
                cvDestroyWindow("WindowShowImage");
            }
        }

        bool LaneFollower::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage> ();

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory
                            = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
                                    si.getName());
                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);
                    const uint32_t numberOfChannels = 3;
                    // For example, simply show the image.
                    if (m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, numberOfChannels);
                    }

                    // Copying the image data is very expensive...
                    if (m_image != NULL) {
                        memcpy(m_image->imageData,
                               m_sharedImageMemory->getSharedMemory(),
                               si.getWidth() * si.getHeight() * numberOfChannels);
                    }

                    // Mirror the image.
                    //cvFlip(m_image, 0, -1);

                    retVal = true;
                }
            }
            return retVal;
        }

        void LaneFollower::processImage() {
            IplImage *gray = cvCreateImage(cvGetSize(m_image),IPL_DEPTH_8U,1);
            cvCvtColor(m_image,gray,CV_BGR2GRAY);
            cvSmooth(gray,gray, CV_BLUR, 3,3);
            cvCanny(gray,gray, 50,200,3);
            cvMerge(gray,gray,gray, NULL, m_image);

            static bool useRightLaneMarking = true;
            double e = 0;

            const int32_t CONTROL_SCANLINE = 462; // calibrated length to right: 280px
            const int32_t distance = 280;

            TimeStamp beforeImageProcessing;
            for(int32_t y = m_image->height - 8; y > m_image->height * .6; y -= 10) {
                // Search from middle to the left:
                CvScalar pixelLeft;
                CvPoint left;
                left.y = y;
                left.x = -1;
                for(int x = m_image->width/2; x > 0; x--) {
                    pixelLeft = cvGet2D(m_image, y, x);
                    if (pixelLeft.val[0] >= 200) {
                        left.x = x;
                        break;
                    }
                }

                // Search from middle to the right:
                CvScalar pixelRight;
                CvPoint right;
                right.y = y;
                right.x = -1;
                for(int x = m_image->width/2; x < m_image->width; x++) {
                    pixelRight = cvGet2D(m_image, y, x);
                    if (pixelRight.val[0] >= 200) {
                        right.x = x;
                        break;
                    }
                }

                if (m_debug) {
                    if (left.x > 0) {
                        CvScalar green = CV_RGB(0, 255, 0);
                        cvLine(m_image, cvPoint(m_image->width/2, y), left, green, 1, 8);

                        stringstream sstr;
                        sstr << (m_image->width/2 - left.x);
                        cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 - 100, y - 2), &m_font, green);
                    }
                    if (right.x > 0) {
                        CvScalar red = CV_RGB(255, 0, 0);
                        cvLine(m_image, cvPoint(m_image->width/2, y), right, red, 1, 8);

                        stringstream sstr;
                        sstr << (right.x - m_image->width/2);
                        cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 + 100, y - 2), &m_font, red);
                    }
                }

                if (y == CONTROL_SCANLINE) {
                    // Calculate the deviation error.
                    if (right.x > 0) {
                        if (!useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }

                        e = ((right.x - m_image->width/2.0) - distance)/distance;

                        useRightLaneMarking = true;
                    }
                    else if (left.x > 0) {
                        if (useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                        
                        e = (distance - (m_image->width/2.0 - left.x))/distance;

                        useRightLaneMarking = false;
                    }
                    else {
                        // If no measurements are available, reset PID controller.
                        m_eSum = 0;
                        m_eOld = 0;
                    }
                }
            }

            TimeStamp afterImageProcessing;

            // Show resulting features.
            if (m_debug) {
                if (m_image != NULL) {
                    cvShowImage("WindowShowImage", m_image);
                    cvWaitKey(10);
                }
            }

            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;

            if (fabs(e) < 1e-2) {
                m_eSum = 0;
            }
            else {
                m_eSum += e;
            }

            // The following values have been determined by Twiddle algorithm.
            const double Kp = 25.5;

            const double Ki = 8.5;

            const double Kd = 1;
                

            const double p = Kp * e;
            const double i = Ki * timeStep * m_eSum;
            const double d = Kd * (e - m_eOld)/timeStep;
            m_eOld = e;

            const double y = p + i + d;
            
            double desiredSteering = 0;
            if (fabs(e) > 1e-2) {
                desiredSteering = y;

                if (desiredSteering > 80.0) {
                    desiredSteering = 80.0;
                }
                if (desiredSteering < -80.0) {
                    desiredSteering = -80.0;
                }
            }


            // Go forward.
            m_vehicleControl.setSpeed(2);
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);
        }

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t> ("lanefollower.debug") == 1;

            // Initialize fonts.
            const double hscale = 0.4;
            const double vscale = 0.3;
            const double shear = 0.2;
            const int thickness = 1;
            const int lineType = 6;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);

            // Sensor IDs.
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
            const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;
            // Overtaking parameters.
            const double OVERTAKING_DISTANCE = 3;
            const double FREE_LANE_DISTANCE = 5;
            const double FRONT_IR_TO_OBJECT = 1;
            const double FRONT_IR_SEARCH_END_OF_OBJECT = 3;
            const double REAR_IR_CONT_LANE = 1;
            const double VEHICLE_SPEED = 1;

            // Overall state machines for moving and measuring.
            enum StateMachineMoving {
            IDLE, FORWARD, SWITCH_TO_LEFT, LEFT_LANE_CORRECT, 
            CONTINUE_ON_LEFT_LANE, SWITCH_TO_RIGHT, RIGHT_LANE_CORRECT };

            enum StateMachineMeasuring {
            DISABLE, FINDING_OBJECT, HAVE_FRONT_IR, SEARCH_END_OF_OBJECT };
            // Init state machines.
            StateMachineMoving stageMoving = IDLE;
            StateMachineMeasuring stageMeasuring = FINDING_OBJECT;

            // State counter for dynamically moving back to right lane.
            int32_t turnRightCounter = 0;
            int32_t turnLeftCounter = 0;

        

            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                bool has_next_frame = true;

                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());

                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (true == has_next_frame) {
                    processImage();
                }


                // Overtaking part.
                {
                    // 1. Get most recent vehicle data:
                    Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                    VehicleData vd = containerVehicleData.getData<VehicleData> ();

                    // 2. Get most recent sensor board data:
                    Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                    SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                    // Moving state machine.

                    if(stageMoving == IDLE) {
                        m_vehicleControl.setSpeed(0);
                        m_vehicleControl.setSteeringWheelAngle(0);
                    }
                     else if (stageMoving == FORWARD) {
                         m_vehicleControl.setSpeed(0);
                         m_vehicleControl.setSteeringWheelAngle(0);
                    }
                    else if (stageMoving == SWITCH_TO_LEFT) {
                        // Move to the left lane: Turn left part until the front IR has closed up on the object.
                        m_vehicleControl.setSpeed(VEHICLE_SPEED);
                        m_vehicleControl.setSteeringWheelAngle(-30);

                        stageMeasuring = HAVE_FRONT_IR;
                        turnRightCounter++;
                    }
                    else if (stageMoving == LEFT_LANE_CORRECT) {
                        // Move to the left lane: Turn right part until the rear IR has closed up on the object.
                        m_vehicleControl.setSpeed(VEHICLE_SPEED);
                        m_vehicleControl.setSteeringWheelAngle(40);
                       
                        turnLeftCounter++;
                        // State machine measuring: Stay in this state until the front IR has determined that the car has reached the end of the object.
                        stageMeasuring = SEARCH_END_OF_OBJECT;
                        // State machine movement: Stay in this state until the rear IR has determined the car has completed both turns on the left lane.
                        if(sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) <= REAR_IR_CONT_LANE) {
                        stageMoving = CONTINUE_ON_LEFT_LANE;
                        }
                    }
                    else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                        // Move to the left lane: Passing stage.
                        m_vehicleControl.setSpeed(VEHICLE_SPEED);
                        m_vehicleControl.setSteeringWheelAngle(0);
                    }
                    else if (stageMoving == SWITCH_TO_RIGHT) {
                        // Move to the right lane: Turn right until the turnCounter decrements to meet the condition.
                        m_vehicleControl.setSpeed(VEHICLE_SPEED);
                        m_vehicleControl.setSteeringWheelAngle(20);
                      
                        turnRightCounter--;
                        if (turnRightCounter <= 30) {
                            stageMoving = RIGHT_LANE_CORRECT;
                        }
                    }
                    else if (stageMoving == RIGHT_LANE_CORRECT) {
                        // Adjust on the right lane by turning left until the turnCounter decrements to meet
                        // the condition then proceed to reset state machines, turnCounters and LaneFollower PID.
                       m_vehicleControl.setSpeed(VEHICLE_SPEED);
                       m_vehicleControl.setSteeringWheelAngle(-30);

                        turnLeftCounter--;
                        if (turnLeftCounter < turnLeftCounter / 4) {
                            // Start over.
                            stageMoving = FORWARD;
                            stageMeasuring = FINDING_OBJECT;

                            turnLeftCounter = 0;
                            turnRightCounter = 0;

                            // Reset PID controller.
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                    }
                     if (stageMeasuring == FINDING_OBJECT) {
                        double US = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                        // Start turning left once the front US has detected an object.
                        if (US <= OVERTAKING_DISTANCE && US > 0) {
                           stageMoving = SWITCH_TO_LEFT;
                            // Disable measuring until requested from moving state machine again.
                            stageMeasuring = DISABLE;
                        }
                        else {
                            stageMeasuring = FINDING_OBJECT;
                        }
                    }
                    else if (stageMeasuring == HAVE_FRONT_IR) {
                        // Remain in this stage until the front IR has closed up on the object.
                        if ((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) <= FRONT_IR_TO_OBJECT)) {
                            // Turn to right.
                            stageMoving = LEFT_LANE_CORRECT;
                        }
                   
                    }
                    else if (stageMeasuring == SEARCH_END_OF_OBJECT) {
                        // Find end of object.
                        // Start turning  to the right once the front IR has reached the correct distance and the right US has determined the right lane is free.
                        if (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) >= FRONT_IR_SEARCH_END_OF_OBJECT &&
                            sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) >= FREE_LANE_DISTANCE) {
                            // Move to right lane again.
                            stageMoving = SWITCH_TO_RIGHT; 
                            // Disable measuring until requested from moving state machine again.
                            stageMeasuring = DISABLE;
                        }
                    }
                }

                // Create container for finally sending the set values for the control algorithm.
                Container c2(m_vehicleControl);
                // Send container.
                getConference().send(c2);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} 