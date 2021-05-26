/* Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "LSPC.h"
#include <MessageTypes.h>
#include <utils.hpp>

#include <array>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>      // for file output (std::ofstream)
#include <boost/filesystem.hpp>   // for directory creation (boost::filesystem)

void TestThread(lspc::Socket& socket);
void TestMessage(lspc::Socket& socket);

void handl(const std::vector<uint8_t>& payload)
{
    std::cout << "Handling:";
    std::stringstream ss;
    ss << std::setfill('0');
    for (int i = 0; i < payload.size(); ++i)
    {
        // ss << " " << std::setw(2) << std::hex << 55;
        ss << " 0x" << std::setw(2) << std::uppercase << std::hex << static_cast<int>(payload[i]);
    }
    std::cout << ss.str();
    std::cout << std::endl;
    return;
}

void LSPC_Callback_ArrayDump(std::shared_ptr<std::ofstream> log_file, const std::vector<uint8_t>& payload)
{
    int numberOfFloats = payload.size() / 4;
    const float * floatArray = reinterpret_cast<const float *>(payload.data());

    if (log_file->is_open()) {
        for (int i = 0; i < numberOfFloats; i++) {
            if (i > 0) *log_file << "\t";
            *log_file << std::setprecision(10) << floatArray[i];
        }
        *log_file << std::endl;
    }
}

void LSPC_Callback_Sensors(std::shared_ptr<std::ofstream> log_file, const std::vector<uint8_t>& payload)
{
    const auto * msg = reinterpret_cast<const lspc::MessageTypesToPC::Sensors_t *>(payload.data());
    if (sizeof(*msg) != payload.size()) {
        std::cout << "Error parsing Sensors message" << std::endl;
        return;
    }

    if (log_file->is_open()) {
        *log_file << std::setprecision(10) << msg->timestamp;

        *log_file << "\t";
        *log_file << std::setprecision(10) << msg->encoders.front;

        *log_file << "\t";
        *log_file << std::setprecision(10) << msg->encoders.back;

        *log_file << "\t";
        *log_file << std::setprecision(10) << msg->rc.throttle;

        *log_file << "\t";
        *log_file << std::setprecision(10) << msg->rc.steering;

        *log_file << "\t";
        *log_file << std::setprecision(10) << msg->motors.throttle;

        *log_file << "\t";
        *log_file << std::setprecision(10) << msg->motors.steering;

        *log_file << std::endl;
    }
}

void debugHandler(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    std::cout << message;
}

bool shouldExit = false;

void exitHandler(int signum) {
    shouldExit = true;
}

int main(int argc, char** argv)
{
    signal(SIGINT, exitHandler);

    if (!boost::filesystem::is_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/esc_logs"))) {
        if (boost::filesystem::exists(boost::filesystem::path(std::string(getenv("HOME")) + "/esc_logs"))) {
            std::cout << "ESC dump path (~/esc_logs) already exists but without write permissions" << std::endl;
        } else {
            if (!boost::filesystem::create_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/esc_logs")))
                std::cout << "Could not create log folder (~/esc_logs)" << std::endl;
            else
                std::cout << "Successfully created log folder (~/esc_logs)" << std::endl;
        }
    }

    std::shared_ptr<std::ofstream> dumpFile = std::make_shared<std::ofstream>();

    while (!shouldExit) {
        { // create scope wherein the lspc object is created - this enforces destruction if connection is lost
            lspc::Socket lspc;
            while (!lspc.isOpen() && !shouldExit) {
                try {
                    std::cout << "Trying to connect to ESC" << std::endl;
                    lspc.open("/dev/ttyACM0", 1612800);
                }
                catch (boost::system::system_error &e) {
                    std::cout << "Error: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            if (shouldExit) break;

            std::string dumpFilename = utils::GetFormattedTimestampCurrent() + ".txt";
            dumpFile->open(std::string(getenv("HOME")) + "/esc_logs/" + dumpFilename, std::ofstream::trunc);
            std::cout << "Created dump file: ~/esc_logs/" << dumpFilename << std::endl;

            std::cout << "Connected to ESC" << std::endl;
            boost::thread testThread = boost::thread(boost::bind(&TestThread, boost::ref(lspc)));

            lspc.registerCallback(lspc::MessageTypesToPC::Test, &handl);
            lspc.registerCallback(lspc::MessageTypesToPC::CPUload, &debugHandler);
            //lspc.registerCallback(lspc::MessageTypesToPC::Sensors, boost::bind(&LSPC_Callback_Sensors, dumpFile, _1));
            lspc.registerCallback(lspc::MessageTypesToPC::Sensors, boost::bind(&LSPC_Callback_ArrayDump, dumpFile, _1));
            lspc.registerCallback(lspc::MessageTypesToPC::Debug, &debugHandler);

            while (lspc.isOpen() && !shouldExit) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (testThread.joinable())
                testThread.join();

            dumpFile->close();

            if (shouldExit)
                break;
        }

        std::cout << "Connection lost to ESC" << std::endl;
    }

    //std::cout << "Exiting..." << std::endl;
}

void TestThread(lspc::Socket& socket)
{
    while (socket.isOpen() && !shouldExit) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
}

void TestMessage(lspc::Socket& socket)
{
    std::vector<uint8_t> payload;
    payload.push_back(0x12);
    payload.push_back(0x34);
    payload.push_back(0x56);
    payload.push_back(0x78);

    socket.send(0xE0, payload);
}
