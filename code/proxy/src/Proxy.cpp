/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
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
#include <chrono>

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>
#include <string>
#include <memory>

#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <time.h>
#include <opendavinci/odcore/base/Thread.h>
#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "automotivedata/generated/automotive/VehicleControl.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include <automotivedata/generated/automotive/miniature/SensorBoardData.h>
#include <bitset>

#include "OpenCVCamera.h"

#ifdef HAVE_UEYE
    #include "uEyeCamera.h"
#endif

#include "Proxy.h"  

using namespace odcore;
using namespace odcore::wrapper;
using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odtools::recorder;

namespace automotive {
    namespace miniature {

        // Map to map the sensor reads with the sensor
        static map<uint32_t, double> map;

        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera()
        {}

        Proxy::~Proxy() {
        }

        void Proxy::setUp() {
            // This method will be call automatically _before_ running body().
            if (getFrequency() < 20) {
                cerr << endl << endl << "Proxy: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
            }
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Create built-in recorder.
            const bool useRecorder = kv.getValue<uint32_t>("proxy.useRecorder") == 1;
            if (useRecorder) {
                // URL for storing containers.
                stringstream recordingURL;
                recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS_noBlankNoColons() << ".rec";
                // Size of memory segments.
                const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.memorySegmentSize");
                // Number of memory segments.
                const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.numberOfMemorySegments");
                // Run recorder in asynchronous mode to allow real-time recording in background.
                const bool THREADING = true;
                // Dump shared images and shared data?
                const bool DUMP_SHARED_DATA = getKeyValueConfiguration().getValue<uint32_t>("proxy.recorder.dumpshareddata") == 1;

                m_recorder = unique_ptr<Recorder>(new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));
            }

            // Create the camera grabber.
            const string NAME = getKeyValueConfiguration().getValue<string>("proxy.camera.name");
            string TYPE = getKeyValueConfiguration().getValue<string>("proxy.camera.type");
            std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
            const uint32_t ID = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.id");
            const uint32_t WIDTH = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.width");
            const uint32_t HEIGHT = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.height");
            const uint32_t BPP = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.bpp");

            if (TYPE.compare("opencv") == 0) {
                m_camera = unique_ptr<Camera>(new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP));
            }
            if (TYPE.compare("ueye") == 0) {
#ifdef HAVE_UEYE
                m_camera = unique_ptr<Camera>(new uEyeCamera(NAME, ID, WIDTH, HEIGHT, BPP));
#endif
            }

            if (m_camera.get() == NULL) {
                cerr << "No valid camera type defined." << endl;
            }

            //
        }

        void Proxy::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        void Proxy::distribute(Container c) {
            // Store data to recorder.
            if (m_recorder.get() != NULL) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                m_recorder->store(c);
            }

            // Share data.
            getConference().send(c);
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {

            uint32_t captureCounter = 0;
            unsigned char old=0;

            // Get the Serial port and the baud rate from the superComponent config file
            const string SERIAL_PORT= getKeyValueConfiguration().getValue<string>("proxy.Arduino.SerialPort");;
            const uint32_t BAUD_RATE = getKeyValueConfiguration().getValue<uint32_t>("proxy.Arduino.SerialSpeed");

            // Create the serial port
            std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
            // Start the listener and attach it to the proxy
            serial->setStringListener(this);
            // Start listening
            serial->start();


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // Capture frame.
                if (m_camera.get() != NULL)
                {
                    odcore::data::image::SharedImage si = m_camera->capture();

                    Container c(si);
                    distribute(c);

                    captureCounter++;
                }
                //............................code...............................
                // Vehicle control data from the conference
                Container container = getKeyValueDataStore().get(VehicleControl::ID());
                VehicleControl vc = container.getData<VehicleControl>();

                // turn the steering value to an angle
                int steerAngle =  vc.getSteeringWheelAngle() * 180 / M_PI;
                unsigned char angle = (unsigned char)(steerAngle + 90);

                // keep the angle in this range (60 - 120)
                angle = (angle < 70 ? 60 : (angle > 120 ? 120 : angle));
                // set the 8th bit if speed is 2 (move forward) or 0 if it's 1 (move backward) 
                // angle = angle | 128 * ((int32_t) vc.getSpeed() == 2);
                // create the string to send
                std::string toSend(1, angle);
                // Send an order to the arduino only if the previous order is not euqal
                if((angle != old)) serial->send(toSend);

                // update the old value
                old = angle;
            }

            cout << "Proxy: Captured " << captureCounter << " frames." << endl;

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY; 
        }
        /*
        * This function reads from the buffer, process the reads then creats a sensorBoardData objects 
        * put them in a container then sends
        */

        /*
        *   The protocol to decode the reads from the Sensors:
        *   the last two bits in the byte (7th && 8th) are the flag to distinguish between the sensors
        *   00(0) : UI reads, 01(1): IR reads, 10(2): IR third Sensor, 11(3): Odometer read
        *   the rest of the bits in the case of IR and UI are 3 for each sensor so each byte holds 2 sensors at a time
        *   Excpet for the IR third it holds the value for on sensor, for the Odometer it takes all the 6 bits to be presneted 
        */

        void Proxy::nextString(const std::string &buffer)
        {   
            // A byte to read at a time
            unsigned char byte; 
            // SensorBoardData  object to collect the sensor reads
            SensorBoardData SBD;
            // Number of sensors in the object
            SBD.setNumberOfSensors(6);

            // ID's for the sensors in the map (must be the same in the overtaking)
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
            const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;
            const int32_t INFRARED_REAR_LEFT = 1;
            const int32_t ODOMETER = 5;

            for(uint32_t i=0; i < buffer.size(); i++)
            {
                //Check if the map contains the reads for all the sensors
                if (map.count(ULTRASONIC_FRONT_CENTER) &&
                    map.count(ULTRASONIC_FRONT_RIGHT) &&
                    map.count(INFRARED_FRONT_RIGHT) &&
                    map.count(INFRARED_REAR_RIGHT) &&
                    map.count(INFRARED_REAR_LEFT) && 
                    map.count(ODOMETER))
                {
                    // Fill the SBD with the reads
                    SBD.setMapOfDistances(map);

                    uint32_t x = 0;
                    while(x < 6) 
                        {
                            cout << map[x] << ", ";
                            x++;
                        }
                    cout << '\n';
                    // Clear the map for new reads
                    map.clear();

                    //Create a container out of the SBD
                    Container container(SBD);
                    //Distribute the container
                    distribute(container);
                }
                // Read on byte from the buffer
                byte = buffer.at(i);

                // Switch the flag
                // UltraSonic reads
                if((byte >> 6) == 0)
                {
                    // UltraSonic1, read the first 3 bits
                    unsigned char UI1 = byte & 7;
                    // UltraSonic2, read the second 3 bits
                    unsigned char UI2 = (byte >> 3) & 7;

                    map[ULTRASONIC_FRONT_RIGHT] = (double) UI1;
                    map[ULTRASONIC_FRONT_CENTER] = (double) UI2;
                }
                // IR reads
                else if((byte >> 6) == 1)
                {
                    // Read the first 3 bits
                    unsigned char IR1 = byte & 7;
                    // Read the second 3 bits
                    unsigned char IR2 = (byte >> 3) & 7;
                    map[INFRARED_REAR_RIGHT] = IR1;
                    map[INFRARED_FRONT_RIGHT] = IR2;
                }
                // IR third sensor read only
                else if((byte >> 6) == 2)
                {
                    // Read the first 3 bits
                    unsigned char IR3 = byte & 7;
                    unsigned char Odometer = (byte >> 3) & 7;
                    map[INFRARED_REAR_LEFT] = IR3;
                    map[ODOMETER] = Odometer;
                }
                // // Odometer read TODO
                // if((byte >> 6) == 3)
                //{}
                // break;
            }
        }
    }
} // automotive::miniature
