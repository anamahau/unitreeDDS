/*
 * Copyright(c) 2006 to 2021 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
 */
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>
#include <fstream>
#include <iomanip>
// #include <example_module\example_device.h>
// #include <example_module\example_channel.h>
// #include <opendaq\opendaq.h>
// #include <C:\Program Files\openDAQ\include\opendaq\opendaq.h>
// #undef Seconds
// #undef duration

 /* Include the C++ DDS API. */
#include "dds/dds.hpp"
#include "dds/domain/qos/DomainParticipantQos.hpp"

// Include generated headers
#include "LowState.hpp"
#include "SportModeState.hpp"

// Create .csv file
std::ofstream writeFile("forces.csv");

bool show_imu_quaternion = false;
bool show_imu_gyro = false;
bool show_imu_acc = false;
bool show_imu_rpy = false;
bool show_foot_forces = true;
bool show_foot_forces_est = false;
bool show_battery = false;
bool show_motors = false;
bool show_fan = false;
bool show_head = false;
bool show_version = false;
bool show_level_flag = false;
bool show_sn = false;
bool show_crc = false;
bool show_temp_ntc = false;
bool show_power = false;
bool show_frame_reserve = false;
bool show_reserve = false;
bool show_bandwidth = false;
bool show_tick = false;
bool show_bit_flag = false;
bool show_adc_reel = false;
bool show_wireless_remote = false;

int time_int = 0;

auto now = std::chrono::system_clock::now();
// auto duration = now.time_since_epoch(); // !!!
// auto milliseconds_start = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); // !!!

class SensorDataListener : public virtual dds::sub::DataReaderListener<unitree_go::msg::dds_::LowState_>
{
public:
    virtual void on_data_available(dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader)
    {
        auto samples = reader.take();
        for (const auto& sample : samples) {
            if (sample.info().valid()) {
                const auto& state = sample.data();

                // auto now = std::chrono::system_clock::now(); // !!!
                // auto duration = now.time_since_epoch(); // !!!
                // auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); // !!!

                // writeFile << (float)(milliseconds - milliseconds_start) / 1000; // !!!

                // Access foot forces
                if (show_foot_forces) {
                    auto forces = state.foot_force();
                    std::cout << "foot forces:"
                        << " FL=" << forces[0]
                        << " FR=" << forces[1]
                        << " RL=" << forces[2]
                        << " RR=" << forces[3] << std::endl;
                    writeFile << ", " << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3];
                    // time_int += 1;

                    //// Dump raw bytes around foot forces
                    //const unsigned char* rawPtr = reinterpret_cast<const unsigned char*>(forces.data());
                    //// int bytesToShow = 32;  // show 32 bytes starting at forces[0]
                    //int bytesToShow = 8;

                    //std::cout << "Raw bytes around foot forces: ";
                    //for (int i = 0; i < bytesToShow; i++) {
                    //    printf("%02X ", rawPtr[i]);
                    //}
                    //std::cout << std::endl;
                    //std::cout << "Raw bytes around foot forces: ";
                    //for (int i = 0; i < bytesToShow; i++) {
                    //    unsigned char byte = rawPtr[i];
                    //    std::cout << std::bitset<8>(byte) << " ";
                    //}
                    //std::cout << std::endl;

                    //const unsigned char* rawPtr2 = reinterpret_cast<const unsigned char*>(&state);
                    //size_t size = sizeof(state);

                    //std::cout << "Dumping state struct (" << size << " bytes): ";
                    //for (size_t i = 0; i < size; i++) {
                    //    printf("%02X ", rawPtr2[i]);
                    //}
                    //std::cout << std::endl;
                    //for (size_t i = 0; i < size; i++) {
                    //    unsigned char byte = rawPtr2[i];
                    //    std::cout << std::bitset<8>(byte) << " ";
                    //    // if ((i + 1) % 8 == 0) std::cout << "\n";  // newline every 8 bytes for readability
                    //}
                    //std::cout << std::endl;

                }

                if (show_foot_forces_est) {
                    auto forces_est = state.foot_force_est();
                    std::cout << "foot forces est:"
                        << " FL=" << int(forces_est[0])
                        << " FR=" << int(forces_est[1])
                        << " RL=" << int(forces_est[2])
                        << " RR=" << int(forces_est[3]) << std::endl;
                }

                // Access battery data
                if (show_battery) {
                    auto bms = state.bms_state();
                    std::cout << "Battery SOC: " << (int)bms.soc()
                        << "% Current: " << bms.current() << "mA" << std::endl;
                }

                // Access motor data (20 motors for Go2)
                if (show_motors) {
                    for (int i = 0; i < 20; i++) {
                        auto motor = state.motor_state()[i];
                        /*if (i < 12) {
                            std::cout << "Motor " << i
                                << " pos=" << motor.q()
                                << " vel=" << motor.dq()
                                << " torque=" << motor.tau_est() << std::endl;
                            writeFile << ", " << motor.q();
                        }*/
                        std::cout << "Motor " << i << " temp=" << int(motor.temperature()) << std::endl;
                        /*if (i < 12) {
                            writeFile << ", " << motor.q() << ", " << motor.dq() << ", " << motor.tau_est();
                        }*/
                    }
                }

                if (show_imu_quaternion || show_imu_gyro || show_imu_acc || show_imu_rpy) {
                    auto imu = state.imu_state();
                    if (show_imu_quaternion) {
                        auto imu_quaternion = imu.quaternion();
                        std::cout << "IMU quaternion:";
                        for (int i = 0; i < sizeof(imu_quaternion) / sizeof(imu_quaternion[0]); i++) {
                            std::cout << " " << imu_quaternion[i];
                        }
                        std::cout << "\n";
                        writeFile << ", " << imu_quaternion[0] << ", " << imu_quaternion[1] << ", " << imu_quaternion[2] << ", " << imu_quaternion[3];
                    }
                    if (show_imu_gyro) {
                        auto imu_gyroscope = imu.gyroscope();
                        std::cout << "IMU gyroscope:";
                        for (int i = 0; i < sizeof(imu_gyroscope) / sizeof(imu_gyroscope[0]); i++) {
                            std::cout << " " << imu_gyroscope[i];
                        }
                        std::cout << "\n";
                        writeFile << ", " << imu_gyroscope[0] << ", " << imu_gyroscope[1] << ", " << imu_gyroscope[2];
                    }
                    if (show_imu_acc) {
                        auto imu_acc = imu.accelerometer();
                        std::cout << "IMU accelerometer:";
                        for (int i = 0; i < sizeof(imu_acc) / sizeof(imu_acc[0]); i++) {
                            std::cout << " " << imu_acc[i];
                        }
                        std::cout << "\n";
                        writeFile << ", " << imu_acc[0] << ", " << imu_acc[1] << ", " << imu_acc[2];
                    }
                    if (show_imu_rpy) {
                        auto imu_rpy = imu.rpy();
                        std::cout << "IMU RPY:";
                        for (int i = 0; i < sizeof(imu_rpy) / sizeof(imu_rpy[0]); i++) {
                            std::cout << " " << imu_rpy[i];
                        }
                        std::cout << "\n";
                        writeFile << ", " << imu_rpy[0] << ", " << imu_rpy[1] << ", " << imu_rpy[2];
                    }
                }

                if (show_version) {
                    auto version = state.version();
                    std::cout << "version: " << int(version[0]) << " " << int(version[1]) << std::endl;
                }

                if (show_head) {
                    auto head = state.head();
                    std::cout << "head: " << int(head[0]) << " " << int(head[1]) << std::endl;
                }

                // Access fan frequency
                if (show_fan) {
                    auto fan = state.fan_frequency();
                    std::cout << "fan:";
                    for (int i = 0; i < sizeof(fan) / sizeof(fan[0]); i++) {
                        std::cout << " " << int(fan[i]);
                    }
                    std::cout << "\n";
                }

                if (show_wireless_remote) {
                    auto wireless_remote = state.wireless_remote();
                    std::cout << "wireless remote:";
                    for (int i = 0; i < sizeof(wireless_remote) / sizeof(wireless_remote[0]); i++) {
                        std::cout << " " << wireless_remote[i];
                    }
                    std::cout << "\n";
                }

                if (show_power) {
                    auto pow_v = state.power_v();
                    auto pow_a = state.power_a();
                    std::cout << "power_a=" << pow_a << ", power_v=" << pow_v << std::endl;
                }

                if (show_temp_ntc) {
                    auto temp1 = state.temperature_ntc1();
                    auto temp2 = state.temperature_ntc2();
                    std::cout << "temp_ntc1=" << int(temp1) << ", temp_ntc2=" << int(temp2) << std::endl;
                }

                if (show_level_flag) {
                    auto level_flag = state.level_flag();
                    // std::cout << "type of level flag: " << typeid(level_flag).name() << std::endl;
                    std::cout << "level flag: " << int(level_flag) << std::endl;
                }

                if (show_sn) {
                    auto sn = state.sn();
                    std::cout << "sn:";
                    for (int i = 0; i < sizeof(sn) / sizeof(sn[0]); i++) {
                        std::cout << " " << sn[i];
                    }
                    std::cout << "\n";
                }

                if (show_bandwidth) {
                    auto bandwidth = state.bandwidth();
                    std::cout << "bandwidth: " << bandwidth << std::endl;
                }

                if (show_crc) {
                    auto crc = state.crc();
                    std::cout << "crc: " << crc << std::endl;
                }

                if (show_frame_reserve) {
                    auto frame_res = state.frame_reserve();
                    std::cout << "frame_reserve: " << frame_res << std::endl;
                }

                if (show_reserve) {
                    auto res = state.reserve();
                    std::cout << "reserve: " << res << std::endl;
                }

                if (show_tick) {
                    auto tick = state.tick();
                    std::cout << "tick: " << tick << std::endl;
                }

                if (show_bit_flag) {
                    auto bit_flag = state.bit_flag();
                    std::cout << "bit flag: " << bit_flag << std::endl;
                }

                if (show_adc_reel) {
                    auto adc_reel = state.adc_reel();
                    std::cout << "adc reel: " << adc_reel << std::endl;
                }

                writeFile << "\n";

                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    }

    virtual void on_subscription_matched(
        dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader,
        const dds::core::status::SubscriptionMatchedStatus& status)
    {
        std::cout << "on_subscription_matched" << std::endl;
    }

    virtual void on_sample_lost(
        dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader,
        const dds::core::status::SampleLostStatus& status)
    {
        std::cout << "on_sample_lost" << std::endl;
    }

    virtual void on_requested_deadline_missed(
        dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader,
        const dds::core::status::RequestedDeadlineMissedStatus& status)
    {
        std::cout << "on_requested_deadline_missed" << std::endl;
    }

    virtual void on_requested_incompatible_qos(
        dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader,
        const dds::core::status::RequestedIncompatibleQosStatus& status)
    {
        std::cout << "on_requested_incompatible_qos" << std::endl;
    }

    virtual void on_sample_rejected(
        dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader,
        const dds::core::status::SampleRejectedStatus& status)
    {
        std::cout << "on_sample_rejected" << std::endl;
    }

    virtual void on_liveliness_changed(
        dds::sub::DataReader<unitree_go::msg::dds_::LowState_>& reader,
        const dds::core::status::LivelinessChangedStatus& status)
    {
        std::cout << "on_liveliness_changed" << std::endl;
    }

};

class SensorDataListener2 : public virtual dds::sub::DataReaderListener<unitree_go::msg::dds_::SportModeState_>
{
public:
    virtual void on_data_available(dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader)
    {
        std::cout << ":)\n";

        auto samples = reader.take();
        for (const auto& sample : samples) {
            if (sample.info().valid()) {
                const auto& state = sample.data();

                // auto now = std::chrono::system_clock::now(); // !!!
                // auto duration = now.time_since_epoch(); // !!!
                // auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); // !!!

                auto path_point = state.path_point();
                auto point = path_point[0];
                float pose_x = point.x();
                float pose_y = point.y();
                float pose_rot = point.yaw();
                std::cout << "pose x=" << pose_x << " y=" << pose_y << " phi=" << pose_rot << std::endl;

                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    }

    virtual void on_subscription_matched(
        dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader2,
        const dds::core::status::SubscriptionMatchedStatus& status)
    {
        std::cout << "on_subscription_matched" << std::endl;
    }

    virtual void on_sample_lost(
        dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader2,
        const dds::core::status::SampleLostStatus& status)
    {
        std::cout << "on_sample_lost" << std::endl;
    }

    virtual void on_requested_deadline_missed(
        dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader2,
        const dds::core::status::RequestedDeadlineMissedStatus& status)
    {
        std::cout << "on_requested_deadline_missed" << std::endl;
    }

    virtual void on_requested_incompatible_qos(
        dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader2,
        const dds::core::status::RequestedIncompatibleQosStatus& status)
    {
        std::cout << "on_requested_incompatible_qos" << std::endl;
    }

    virtual void on_sample_rejected(
        dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader2,
        const dds::core::status::SampleRejectedStatus& status)
    {
        std::cout << "on_sample_rejected" << std::endl;
    }

    virtual void on_liveliness_changed(
        dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_>& reader2,
        const dds::core::status::LivelinessChangedStatus& status)
    {
        std::cout << "on_liveliness_changed" << std::endl;
    }

};




// Function to create QoS with the specified policies
dds::sub::qos::DataReaderQos createDataReaderQos() {
   dds::sub::qos::DataReaderQos reader_qos;
  
    reader_qos
        // Deadline: infinite (9223372036854775807 nanoseconds)
        << dds::core::policy::Deadline(dds::core::Duration::infinite())

        // DestinationOrder: By reception timestamp  
        << dds::core::policy::DestinationOrder(dds::core::policy::DestinationOrderKind::BY_RECEPTION_TIMESTAMP)

        // Durability: Volatile
        << dds::core::policy::Durability(dds::core::policy::DurabilityKind::VOLATILE)

        // History: Keep last with depth 1
        << dds::core::policy::History(dds::core::policy::HistoryKind::KEEP_LAST, 1)

        // LatencyBudget: 0 (immediate)
        << dds::core::policy::LatencyBudget(dds::core::Duration::zero())

        // Liveliness: Automatic with infinite lease duration
        << dds::core::policy::Liveliness(
            dds::core::policy::LivelinessKind::AUTOMATIC,
            dds::core::Duration::infinite())

        // Ownership: Shared
        << dds::core::policy::Ownership(dds::core::policy::OwnershipKind::SHARED)

        // ReaderDataLifecycle: infinite delays for autopurge
        << dds::core::policy::ReaderDataLifecycle(
            dds::core::Duration::infinite(),  // autopurge_nowriter_samples_delay
            dds::core::Duration::infinite())  // autopurge_disposed_samples_delay

        // Reliability: Best effort
        << dds::core::policy::Reliability(dds::core::policy::ReliabilityKind::BEST_EFFORT)

        // ResourceLimits: unlimited (-1 values)
        << dds::core::policy::ResourceLimits(-1, -1, -1)  // max_samples, max_instances, max_samples_per_instance

        // TimeBasedFilter: 0 (no filtering)
        << dds::core::policy::TimeBasedFilter(dds::core::Duration::zero())
        
        
        
        ;
    return reader_qos;
}

// Function to create Topic QoS (for policies that apply to Topic)
dds::topic::qos::TopicQos createTopicQos() {
    dds::topic::qos::TopicQos topic_qos;

    topic_qos
        // Deadline: infinite
        << dds::core::policy::Deadline(dds::core::Duration::infinite())

        // DestinationOrder: By reception timestamp
        << dds::core::policy::DestinationOrder(dds::core::policy::DestinationOrderKind::BY_RECEPTION_TIMESTAMP)

        // Durability: Volatile
        << dds::core::policy::Durability(dds::core::policy::DurabilityKind::VOLATILE)

        // DurabilityService: cleanup_delay=0, KeepLast(1), unlimited resources
        << dds::core::policy::DurabilityService(
            dds::core::Duration::zero(),                    // cleanup_delay = 0
            dds::core::policy::HistoryKind::KEEP_LAST,      // history policy
            1,                                              // history depth = 1
            -1,                                             // max_samples = -1 (unlimited)
            -1,                                             // max_instances = -1 (unlimited)  
            -1)                                             // max_samples_per_instance = -1 (unlimited)

        // History: Keep last with depth 1
        //<< dds::core::policy::History(dds::core::policy::HistoryKind::KEEP_LAST, 1)

        // LatencyBudget: 0
        << dds::core::policy::LatencyBudget(dds::core::Duration::zero())

        // Lifespan: infinite (applies to Topic)
        << dds::core::policy::Lifespan(dds::core::Duration::infinite())

        // Liveliness: Automatic with infinite lease duration
        << dds::core::policy::Liveliness(
            dds::core::policy::LivelinessKind::AUTOMATIC,
            dds::core::Duration::infinite())

        // Ownership: Shared
        << dds::core::policy::Ownership(dds::core::policy::OwnershipKind::SHARED)

        // Reliability: Best effort
        << dds::core::policy::Reliability(dds::core::policy::ReliabilityKind::BEST_EFFORT)

        // ResourceLimits: unlimited
        << dds::core::policy::ResourceLimits(-1, -1, -1);

    return topic_qos;
}

// Function to create Subscriber QoS (for policies that apply to Subscriber)
dds::sub::qos::SubscriberQos createSubscriberQos() {
    dds::sub::qos::SubscriberQos subscriber_qos;

    subscriber_qos
        // Presentation: Instance scope, no coherent/ordered access
        << dds::core::policy::Presentation(
            dds::core::policy::PresentationAccessScopeKind::INSTANCE,
            false,  // coherent_access = False
            false); // ordered_access = False

    return subscriber_qos;
}

// Example usage
void setupDDSEntities() {
    // Create domain participant
    dds::domain::DomainParticipant participant(0);

    // Create topic with Topic QoS
    dds::topic::qos::TopicQos topic_qos = participant.default_topic_qos();
    dds::topic::Topic<unitree_go::msg::dds_::SportModeState_> topic3(participant, "rt/sportmodestate", topic_qos);

    // Create subscriber with Subscriber QoS  
    dds::sub::qos::SubscriberQos subscriber_qos = createSubscriberQos();
    dds::sub::Subscriber subscriber3(participant);

    // Create DataReader with DataReader QoS
    dds::sub::qos::DataReaderQos reader_qos = createDataReaderQos();
    dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_> reader2(subscriber3, topic3, reader_qos, new SensorDataListener2(), dds::core::status::StatusMask::all());
    // dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_> reader(subscriber, topic, reader_qos);
}




// Function to create DataReader QoS matching the Insight configuration
dds::sub::qos::DataReaderQos createSportModeStateReaderQos() {
    dds::sub::qos::DataReaderQos reader_qos;

    reader_qos
        // Policy.Deadline(deadline=9223372036854775807) - infinite
        << dds::core::policy::Deadline(dds::core::Duration::infinite())

        // Policy.DestinationOrder.ByReceptionTimestamp
        << dds::core::policy::DestinationOrder(dds::core::policy::DestinationOrderKind::BY_RECEPTION_TIMESTAMP)

        // Policy.Durability.Volatile
        << dds::core::policy::Durability(dds::core::policy::DurabilityKind::VOLATILE)

        // Policy.History.KeepLast(depth=1)
        << dds::core::policy::History(dds::core::policy::HistoryKind::KEEP_LAST, 1)

        // Policy.LatencyBudget(budget=0)
        << dds::core::policy::LatencyBudget(dds::core::Duration::zero())

        // Policy.Liveliness.Automatic(lease_duration=9223372036854775807)
        << dds::core::policy::Liveliness(
            dds::core::policy::LivelinessKind::AUTOMATIC,
            dds::core::Duration::infinite())

        // Policy.Ownership.Shared
        << dds::core::policy::Ownership(dds::core::policy::OwnershipKind::SHARED)

        // Policy.ReaderDataLifecycle(autopurge_nowriter_samples_delay=9223372036854775807, autopurge_disposed_samples_delay=9223372036854775807)
        << dds::core::policy::ReaderDataLifecycle(
            dds::core::Duration::infinite(),  // autopurge_nowriter_samples_delay
            dds::core::Duration::infinite())  // autopurge_disposed_samples_delay

        // Policy.Reliability.BestEffort
        << dds::core::policy::Reliability(dds::core::policy::ReliabilityKind::BEST_EFFORT)

        // Policy.ResourceLimits(max_samples=-1, max_instances=-1, max_samples_per_instance=-1)
        << dds::core::policy::ResourceLimits(-1, -1, -1)

        // Policy.TimeBasedFilter(filter_time=0)
        << dds::core::policy::TimeBasedFilter(dds::core::Duration::zero());

    return reader_qos;
}

// Function to create Subscriber QoS for rt/sportmodestate
dds::sub::qos::SubscriberQos createSportModeStateSubscriberQos() {
    dds::sub::qos::SubscriberQos subscriber_qos;

    subscriber_qos
        // Policy.PresentationAccessScope.Instance(coherent_access=False, ordered_access=False)
        << dds::core::policy::Presentation(
            dds::core::policy::PresentationAccessScopeKind::INSTANCE,
            false,  // coherent_access = False
            false); // ordered_access = False

    return subscriber_qos;
}

// Function to create Topic QoS for rt/sportmodestate 
dds::topic::qos::TopicQos createSportModeStateTopicQos() {
    dds::topic::qos::TopicQos topic_qos;

    topic_qos
        // Policy.Deadline(deadline=9223372036854775807)
        << dds::core::policy::Deadline(dds::core::Duration::infinite())

        // Policy.DestinationOrder.ByReceptionTimestamp
        << dds::core::policy::DestinationOrder(dds::core::policy::DestinationOrderKind::BY_RECEPTION_TIMESTAMP)

        // Policy.Durability.Volatile
        << dds::core::policy::Durability(dds::core::policy::DurabilityKind::VOLATILE)

        // Policy.History.KeepLast(depth=1)
        << dds::core::policy::History(dds::core::policy::HistoryKind::KEEP_LAST, 1)

        // Policy.LatencyBudget(budget=0)
        << dds::core::policy::LatencyBudget(dds::core::Duration::zero())

        // Policy.Liveliness.Automatic(lease_duration=9223372036854775807)
        << dds::core::policy::Liveliness(
            dds::core::policy::LivelinessKind::AUTOMATIC,
            dds::core::Duration::infinite())

        // Policy.Ownership.Shared
        << dds::core::policy::Ownership(dds::core::policy::OwnershipKind::SHARED)

        // Policy.Reliability.BestEffort
        << dds::core::policy::Reliability(dds::core::policy::ReliabilityKind::BEST_EFFORT)

        // Policy.ResourceLimits(max_samples=-1, max_instances=-1, max_samples_per_instance=-1)
        << dds::core::policy::ResourceLimits(-1, -1, -1);

    return topic_qos;
}

// Complete setup function for Unitree GO2 SportModeState reader
void setupUnitreeGO2SportModeStateReader() {
    // Create domain participant
    dds::domain::DomainParticipant participant(0);

    // Create topic with Topic QoS for rt/sportmodestate
    auto topic_qos = createSportModeStateTopicQos();
    dds::topic::Topic<unitree_go::msg::dds_::SportModeState_> topic(
        participant,
        "rt/sportmodestate",
        topic_qos);

    // Create subscriber with Subscriber QoS  
    auto subscriber_qos = createSportModeStateSubscriberQos();
    dds::sub::Subscriber subscriber(participant, subscriber_qos);

    // Create DataReader with DataReader QoS
    auto reader_qos = createSportModeStateReaderQos();
    dds::sub::DataReader<unitree_go::msg::dds_::SportModeState_> reader(
        subscriber,
        topic,
        reader_qos);

    std::cout << "SportModeState DataReader created successfully!" << std::endl;
    std::cout << "Topic: rt/sportmodestate" << std::endl;
    std::cout << "Type: unitree_go::msg::dds_::SportModeState_" << std::endl;
}





void lowStateRandS()
{
    // Create DDS domain participant (domain 0)
    dds::domain::DomainParticipant participant(0);

    // Create topic for low-level state
    dds::topic::Topic<unitree_go::msg::dds_::LowState_> topic(participant, "rt/lowstate");
    // Create subscriber and data reader
    dds::sub::Subscriber subscriber(participant);
    dds::sub::DataReader<
        unitree_go::msg::dds_::LowState_> reader(subscriber, topic, subscriber.default_datareader_qos(), new SensorDataListener(), dds::core::status::StatusMask::data_available());
    std::cout << "Listening for Unitree Go2 sensor data on topic 'rt/lowstate'..." << std::endl;

    std::cout << "Press Ctrl+C to exit" << std::endl;

    // Keep running
    while (true) {
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}



int main()
{
    try {
        // Create .csv file
        // std::ofstream writeFile("forces.csv");
        // writeFile << "time, FL, FR, RL, RR, q0, q1, q2, q3, gyro1, gyro2, gyro3, acc1, acc2, acc3, rpy1, rpy2, rpy3\n";
        // writeFile << "time, FL, FR, RL, RR, gyro1, gyro2, gyro3, acc1, acc2, acc3\n";
        writeFile << "time, FL, FR, RL, RR\n";
        // writeFile << "time, m00, m01, m02, m03, m04, m05, m06, m07, m08, m09, m10, m11\n";
        // writeFile << "time, yaw, pitch, roll\n";
        // writeFile << "q0, dq0, tau0, q1, dq1, tau1, q2, dq2, tau2, q3, dq3, tau3, q4, dq4, tau4, q5, dq5, tau5, q6, dq6, tau6, q7, dq7, tau7, q8, dq8, tau8, q9, dq9, tau9, q10, dq10, tau10, q11, dq11, tau11\n";

        lowStateRandS();
        std::cout << ":)" << std::endl;

        // Create DDS domain participant (domain 0)
        // dds::domain::DomainParticipant participant(0); // moved to lowStateRandS function


        /**************************************** lowState subscriber ****************************************/
        // moved to lowStateRandS function
        //// Create topic for low-level state
        //dds::topic::Topic<unitree_go::msg::dds_::LowState_> topic(participant, "rt/lowstate");
        //// Create subscriber and data reader
        //dds::sub::Subscriber subscriber(participant);
        //dds::sub::DataReader<
        //    unitree_go::msg::dds_::LowState_> reader(subscriber, topic, subscriber.default_datareader_qos(), new SensorDataListener(), dds::core::status::StatusMask::data_available());
        //std::cout << "Listening for Unitree Go2 sensor data on topic 'rt/lowstate'..." << std::endl;
        /*****************************************************************************************************/

        /**************************************** sportModeState subscriber ****************************************/
        //// Create topic for sport-mode state
        ////setupDDSEntities();

        //dds::topic::qos::TopicQos topic_qos = createTopicQos();
        //// dds::topic::qos::TopicQos topic_qos2 = createSportModeStateTopicQos();
        //topic_qos << dds::core::policy::Lifespan(dds::core::Duration::infinite());

        //topic_qos << dds::core::policy::Durability(dds::core::policy::DurabilityKind::VOLATILE) <<
        //    dds::core::policy::DurabilityService(
        //        dds::core::Duration::zero(),
        //        dds::core::policy::HistoryKind::KEEP_LAST,
        //        1,
        //        -1,
        //        -1,
        //        -1
        //    );

        //dds::topic::Topic<unitree_go::msg::dds_::SportModeState_> topic2(participant, "rt/sportmodestate", topic_qos);

        //dds::sub::qos::SubscriberQos sub_qos = createSubscriberQos();
        //dds::sub::Subscriber subscriber2(participant, sub_qos);

        //dds::sub::qos::DataReaderQos qos2 = createDataReaderQos();
        //// qos2 << dds::core::policy::DurabilityService(0, dds::core::policy::History::KeepLast(1), -1, -1, -1);
        //     // << dds::core::policy::DurabilityService::max_samples();
        ///*qos2 << dds::core::policy::DurabilityService(
        //            dds::core::Duration(0),
        //            dds::core::policy::HistoryKind_def::KEEP_LAST(1),
        //            -1, -1, -1
        //        );*/
        //        /*qos2 << dds::core::policy::DurabilityService(dds::core::Duration(0), dds::core::policy::History::KeepLast(1), -1, -1, -1);*/

        //        // qos2 << dds::core::policy::DurabilityService(dds::core::Duration(0));


        //dds::sub::DataReader<
        //    unitree_go::msg::dds_::SportModeState_> reader2(subscriber2, topic2, qos2, new SensorDataListener2(), dds::core::status::StatusMask::all());
        //std::cout << "Listening for Unitree Go2 sensor data on topic 'rt/sportmode'..." << std::endl;
        /***********************************************************************************************************/

        // std::cout << "Press Ctrl+C to exit" << std::endl; // moved to lowStateRandS function

        // Keep running
        //while (true) { // moved to lowStateRandS function
        //    // std::this_thread::sleep_for(std::chrono::seconds(1));
        //}

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;

        // close .csv file
        // writeFile.close();

        return 1;
    }

    return 0;
}