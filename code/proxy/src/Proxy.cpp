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

            const string SERIAL_PORT= getKeyValueConfiguration().getValue<string>("proxy.Arduino.SerialPort");;
            const uint32_t BAUD_RATE = getKeyValueConfiguration().getValue<uint32_t>("proxy.Arduino.SerialSpeed");

            // Create the serial port
            std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
            serial->setStringListener(this);
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
                angle = (angle < 70 ? 60 : (angle > 120 ? 110 : angle));
                // set the 8th bit if speed is 2 (move forward) or 0 if it's 1 (move backward) 
                angle = angle | 128 * ((int32_t) vc.getSpeed() == 2);

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
        void Proxy::nextString(const std::string &buffer)
        {
            cout << "ECHO: " << buffer << "\n";
        }
    }
} // automotive::miniature
// // Read from the serial bus
// void SerialReceiveBytes::nextString(const std::string &buffer)
// { 

//     // will wait here
//     lock.lock();
//     std::copy(buffer.begin() + (buffer.size() % 2), buffer.end(), sensor_data_buffer);
//     sensor_data_buffer[buffer.size()] = '\0';
//     lock.unlock();    
//     // unsigned char byte;   
//     // automotive::miniature::SensorBoardData SBD;
//     // SBD.setNumberOfSensors(4);
//     // // Ignore the extra packets
//     // buffer = buffer.substr(0, (buffer.length() - buffer.length() % 2));
//     // // A map to address the sensors with the corresponding values
//     // std::map<uint32_t, double> sensordata;

//     // for(unsigned int i = 0; i < buffer.length(); i++)
//     // {
//     //     // Read one byte at a time from the buffer
//     //     byte = buffer.at(i);
//     //     // if the 8th bit is set then it's the ultrasonic sensors
//         // if(byte >> 7)
//         // {
//         //     // Read the first 3 bits
//         //     unsigned char ultra1 = 7 & byte;
//         //     // Read the second 3 bits
//         //     unsigned char ultra2 = 7 & byte >> 3;
//         //     // Ultrasonic FRONT_CENTER (ultra1) at 0
//         //     sensordata[0] = (double) ultra1;
//         //     // Ultrasonic FRONT_RIGHT (ultra2) at 1
//         //     sensordata[1] = (double) ultra2;
//         // }
//         // else
//         // {
//         //     unsigned char ir1 = 15 & byte;
//         //     unsigned char ir2 = 15 & byte >> 3;
//         //     // INFRARED_FRONT_RIGHT (ir1) is at 2
//         //     sensordata[2] = (double) ir1;
//         //     // INFRARED_REAR_RIGHT (ir2) is at 3
//         //     sensordata[3] = (double) ir2;
//         // }
//     //     if(i % 2 == 0)
//     //     {
//     //         SBD.setMapOfDistances(sensordata);
//     //         // re-init the map
//     //         sensordata.clear();
//     //     }
//     // }
// } password? : 517 can you bring all of my stuff when you finish and meet me at the track room? awesome 