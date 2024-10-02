#ifdef __linux__

#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <vector>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"
#include "rev/CANStatus.h"
#include "rev/Drivers/DriverDeviceThread.h"

#include "utils/ThreadUtils.h"

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {

class SocketCANDeviceThread : public DriverDeviceThread {
public:
    SocketCANDeviceThread() = delete;
    SocketCANDeviceThread(std::string port, long long threadIntervalMs = 1) 
        : DriverDeviceThread(0xe45b5597, threadIntervalMs), m_port(port), m_socket(-1) { }
    
    ~SocketCANDeviceThread() {
        if (m_socket != -1) {
            close(m_socket);
        }
    }

    void Start() override {
        if (m_thread.get() != nullptr && m_thread->joinable()) {
            m_thread->join();
        }

        // Initialize SocketCAN
        if (!initializeSocketCAN()) {
            m_threadStatus = CANStatus::kError;
            return;
        }

        m_thread = std::make_unique<std::thread>(&SocketCANDeviceThread::SocketRun, this);

        // Set to high priority to prevent buffer overflow on the device on high client CPU load
        utils::SetThreadPriority(m_thread.get(), utils::ThreadPriority::High);
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus *status) override {
        std::lock_guard<std::mutex> lock(m_streamMutex);

        // Create the handle
        *handle = m_counter++;

        // Add to the map
        m_readStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<std::shared_ptr<CANMessage>>{maxSize}});

        *status = CANStatus::kOk;
    }

private:
    std::string m_port;
    int m_socket;

    bool initializeSocketCAN() {
        struct ifreq ifr;
        struct sockaddr_can addr;

        // Create socket
        m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (m_socket < 0) {
            perror("SocketCAN socket");
            return false;
        }

        // Specify CAN interface
        std::strncpy(ifr.ifr_name, m_port.c_str(), IFNAMSIZ - 1);
        if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0) {
            perror("SocketCAN ioctl");
            return false;
        }

        // Bind the socket to the CAN interface
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("SocketCAN bind");
            return false;
        }

        return true;
    }

    void ReadMessages(bool &reading) {
        struct can_frame frame;
        int nbytes = read(m_socket, &frame, sizeof(struct can_frame));

        reading = (nbytes > 0);
        if (reading) {
            if (frame.can_id & CAN_ERR_FLAG) {
                // Handle error frame
                m_statusDetails.busOffCount++;
                m_threadStatus = CANStatus::kError;
                // Parse error data
                if (frame.can_id & 0x00000040) {
                    m_statusDetails.busOffCount++;
                } 
                if (frame.data[1] & 0x02) {
                    m_statusDetails.txFullCount++;
                }
                if (frame.data[1] & 0x10 || frame.data[1] & 0x04) {
                    m_statusDetails.receiveErrCount++;
                }
                if (frame.data[1] & 0x20 || frame.data[1] & 0x08 || frame.data[2] & 0x80 || frame.data[4]) {
                    m_statusDetails.transmitErrCount++;
                }
            } else {
                auto msg = std::make_shared<CANMessage>(frame.can_id,
                                                        frame.data,
                                                        frame.can_dlc,
                                                        std::chrono::steady_clock::now().time_since_epoch().count() / 1000);

                // Read functions
                {
                    std::lock_guard<std::mutex> lock(m_readMutex);
                    m_readStore[frame.can_id] = msg;
                }

                // Streaming functions
                {
                    std::lock_guard<std::mutex> lock(m_streamMutex);
                    for (auto& stream : m_readStream) {
                        if (!stream.second->messages.IsFull()
                            && rev::usb::CANBridge_ProcessMask({stream.second->messageId, stream.second->messageMask},
                            msg->GetMessageId())) {
                            stream.second->messages.Add(msg);
                        }
                    }
                }
            }

            m_threadStatus = CANStatus::kOk;
        }
    }

    bool WriteMessages(detail::CANThreadSendQueueElement el, std::chrono::steady_clock::time_point now) {
        if (el.m_intervalMs <= 1 || (now - el.m_prevTimestamp >= std::chrono::milliseconds(el.m_intervalMs)) ) {
            struct can_frame frame;
            frame.can_id = el.m_msg.GetMessageId();
            frame.can_dlc = el.m_msg.GetSize();
            std::memcpy(frame.data, el.m_msg.GetData(), frame.can_dlc);

            int nbytes = write(m_socket, &frame, sizeof(struct can_frame));
            if (nbytes != sizeof(struct can_frame)) {
                m_threadStatus = CANStatus::kDeviceWriteError;
                m_statusErrCount++;
                return false;
            } else {
                m_threadStatus = CANStatus::kOk;
                return true;
            }
        }
        return false;
    }

    void SocketRun() {
        while (m_threadComplete == false) {
            m_threadStatus = CANStatus::kOk; // Start each loop with the status being good. Really only a write issue.
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);

            // 1) Handle all received CAN traffic
            bool reading = false;
            ReadMessages(reading);

            // 2) Schedule CANMessage queue
            {
                std::lock_guard<std::mutex> lock(m_writeMutex);
                if (m_sendQueue.size() > 0) {
                    detail::CANThreadSendQueueElement el = m_sendQueue.front();
                    if (el.m_intervalMs == -1) {
                        m_sendQueue.pop_front();
                        continue;
                    }

                    auto now = std::chrono::steady_clock::now();

                    // Don't pop queue if send fails
                    if (WriteMessages(el, now)) {
                        m_sendQueue.pop_front();

                        // Return to end of queue if repeated
                        if (el.m_intervalMs > 0 ) {
                            el.m_prevTimestamp = now;
                            m_sendQueue.push_back(el);
                        }
                    }
                }
            }

            // 3) Stall thread
            if (!reading && m_sendQueue.empty()) {
                std::this_thread::sleep_until(sleepTime);
            }
        }
    }
};

} // namespace usb
} // namespace rev

#endif