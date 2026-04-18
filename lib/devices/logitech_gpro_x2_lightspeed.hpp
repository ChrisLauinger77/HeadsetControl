#pragma once

#include "../utility.hpp"
#include "protocols/logitech_centurion_protocol.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <string_view>
#include <vector>

using namespace std::string_view_literals;

namespace headsetcontrol {

/**
 * @brief Logitech G PRO X 2 LIGHTSPEED (PID 0x0af7)
 *
 * This variant uses a vendor-specific 64-byte protocol on usage page 0xffa0
 * for battery data instead of HID++.
 */
class LogitechGProX2Lightspeed : public protocols::LogitechCenturionProtocol {
public:
    static constexpr std::array<uint16_t, 1> SUPPORTED_PRODUCT_IDS { 0x0af7 };
    static constexpr size_t PACKET_SIZE                                        = 64;
    static constexpr uint8_t REPORT_PREFIX                                     = 0x51;
    static constexpr uint8_t SIDETONE_DEVICE_MAX                               = 100;
    static constexpr uint8_t SIDETONE_MIC_ID                                   = 0x01;
    static constexpr uint8_t PLAYBACK_DIRECTION                                = 0x00;

    constexpr uint16_t getVendorId() const override
    {
        return VENDOR_LOGITECH;
    }

    std::vector<uint16_t> getProductIds() const override
    {
        return { SUPPORTED_PRODUCT_IDS.begin(), SUPPORTED_PRODUCT_IDS.end() };
    }

    std::string_view getDeviceName() const override
    {
        return "Logitech G PRO X 2 LIGHTSPEED"sv;
    }

    constexpr int getCapabilities() const override
    {
        return B(CAP_SIDETONE) | B(CAP_BATTERY_STATUS) | B(CAP_INACTIVE_TIME)
            | B(CAP_EQUALIZER) | B(CAP_PARAMETRIC_EQUALIZER);
    }

    std::optional<EqualizerInfo> getEqualizerInfo() const override
    {
        return EqualizerInfo {
            .bands_count    = cached_band_count_,
            .bands_baseline = 0,
            .bands_step     = 1.0f,
            .bands_min      = cached_gain_min_,
            .bands_max      = cached_gain_max_
        };
    }

    std::optional<ParametricEqualizerInfo> getParametricEqualizerInfo() const override
    {
        return ParametricEqualizerInfo {
            .bands_count  = cached_band_count_,
            .gain_base    = 0.0f,
            .gain_step    = 1.0f,
            .gain_min     = static_cast<float>(cached_gain_min_),
            .gain_max     = static_cast<float>(cached_gain_max_),
            .q_factor_min = 1.0f,
            .q_factor_max = 1.0f,
            .freq_min     = 20,
            .freq_max     = 20000,
            .filter_types = B(static_cast<int>(EqualizerFilterType::Peaking))
        };
    }

    constexpr capability_detail getCapabilityDetail(enum capabilities cap) const override
    {
        switch (cap) {
        case CAP_BATTERY_STATUS:
        case CAP_SIDETONE:
        case CAP_INACTIVE_TIME:
        case CAP_EQUALIZER:
        case CAP_PARAMETRIC_EQUALIZER:
            return { .usagepage = 0xffa0, .usageid = 0x0001, .interface_id = 3 };
        default:
            return HIDDevice::getCapabilityDetail(cap);
        }
    }

    Result<BatteryResult> getBattery(hid_device* device_handle) override
    {
        auto start_time = std::chrono::steady_clock::now();

        std::array<uint8_t, PACKET_SIZE> request = buildBatteryRequest();
        if (auto write_result = writeHID(device_handle, request, PACKET_SIZE); !write_result) {
            return write_result.error();
        }

        std::vector<uint8_t> raw_packets;
        raw_packets.reserve(PACKET_SIZE * 4);

        for (int attempt = 0; attempt < 4; ++attempt) {
            std::array<uint8_t, PACKET_SIZE> response {};
            if (auto read_result = readHIDTimeout(device_handle, response, hsc_device_timeout); !read_result) {
                return read_result.error();
            }

            raw_packets.insert(raw_packets.end(), response.begin(), response.end());

            if (isPowerOffPacket(response)) {
                return DeviceError::deviceOffline("Headset is powered off or not connected");
            }

            if (isPowerEventPacket(response)) {
                continue;
            }

            if (isAckPacket(response)) {
                continue;
            }

            if (!isBatteryResponsePacket(response)) {
                continue;
            }

            auto battery_result = parseBatteryResponse(response);
            if (!battery_result) {
                return battery_result.error();
            }

            battery_result->raw_data       = std::move(raw_packets);
            auto end_time                  = std::chrono::steady_clock::now();
            battery_result->query_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            return *battery_result;
        }

        return DeviceError::protocolError("Battery response packet not received");
    }

    Result<SidetoneResult> setSidetone(hid_device* device_handle, uint8_t level) override
    {
        uint8_t mapped = map<uint8_t>(level, 0, 128, 0, SIDETONE_DEVICE_MAX);

        auto sidetone_state = sendCenturionFeatureRequest(
            device_handle,
            static_cast<uint16_t>(protocols::CenturionFeature::HeadsetAudioSidetone),
            0x00);
        if (!sidetone_state) {
            return sidetone_state.error();
        }

        std::array<uint8_t, 3> version2_payload { SIDETONE_MIC_ID, 0xFF, mapped };
        std::array<uint8_t, 2> version1_payload { SIDETONE_MIC_ID, mapped };
        std::span<const uint8_t> payload = sidetone_state->size() >= 4
            ? std::span<const uint8_t>(version2_payload)
            : std::span<const uint8_t>(version1_payload);

        if (auto sidetone_write = sendCenturionFeatureRequest(
                device_handle,
                static_cast<uint16_t>(protocols::CenturionFeature::HeadsetAudioSidetone),
                0x10,
                payload);
            !sidetone_write) {
            return sidetone_write.error();
        }

        return SidetoneResult {
            .current_level = level,
            .min_level     = 0,
            .max_level     = 128,
            .device_min    = 0,
            .device_max    = SIDETONE_DEVICE_MAX,
            .is_muted      = level == 0
        };
    }

    Result<EqualizerResult> setEqualizer(hid_device* device_handle, const EqualizerSettings& settings) override
    {
        auto descriptor = readAdvancedEqDescriptor(device_handle);
        if (!descriptor) {
            return descriptor.error();
        }

        if (settings.size() != static_cast<int>(descriptor->bands.size())) {
            return DeviceError::invalidParameter("Equalizer requires one gain value per playback band");
        }

        std::vector<AdvancedEqBand> bands;
        bands.reserve(descriptor->bands.size());

        for (size_t i = 0; i < descriptor->bands.size(); ++i) {
            float gain = settings.bands[i];
            if (gain < descriptor->gain_min || gain > descriptor->gain_max) {
                return DeviceError::invalidParameter("Equalizer gain is outside the supported range");
            }

            bands.push_back(AdvancedEqBand {
                .frequency = descriptor->bands[i].frequency,
                .gain_db   = encodeGain(gain)
            });
        }

        if (auto write_result = writePlaybackAdvancedEq(device_handle, descriptor->active_slot, bands); !write_result) {
            return write_result.error();
        }

        return EqualizerResult {};
    }

    Result<ParametricEqualizerResult> setParametricEqualizer(
        hid_device* device_handle,
        const ParametricEqualizerSettings& settings) override
    {
        auto descriptor = readAdvancedEqDescriptor(device_handle);
        if (!descriptor) {
            return descriptor.error();
        }

        if (settings.size() != static_cast<int>(descriptor->bands.size())) {
            return DeviceError::invalidParameter("Parametric equalizer requires exactly one entry per playback band");
        }

        std::vector<AdvancedEqBand> bands;
        bands.reserve(descriptor->bands.size());

        for (const auto& band : settings.bands) {
            if (band.type != EqualizerFilterType::Peaking) {
                return DeviceError::invalidParameter("This headset only supports peaking EQ bands");
            }

            if (std::fabs(band.q_factor - 1.0f) > 0.001f) {
                return DeviceError::invalidParameter("This headset uses a fixed Q factor of 1.0");
            }

            if (band.frequency < 20.0f || band.frequency > 20000.0f) {
                return DeviceError::invalidParameter("Frequency must be between 20 Hz and 20000 Hz");
            }

            if (band.gain < descriptor->gain_min || band.gain > descriptor->gain_max) {
                return DeviceError::invalidParameter("Gain is outside the supported range");
            }

            bands.push_back(AdvancedEqBand {
                .frequency = static_cast<uint16_t>(band.frequency),
                .gain_db   = encodeGain(band.gain)
            });
        }

        if (auto write_result = writePlaybackAdvancedEq(device_handle, descriptor->active_slot, bands); !write_result) {
            return write_result.error();
        }

        return ParametricEqualizerResult {};
    }

    Result<InactiveTimeResult> setInactiveTime(hid_device* device_handle, uint8_t minutes) override
    {
        auto command = buildInactiveTimeCommand(minutes);
        if (auto write_result = writeHID(device_handle, command, PACKET_SIZE); !write_result) {
            return write_result.error();
        }

        return InactiveTimeResult {
            .minutes     = minutes,
            .min_minutes = 0,
            .max_minutes = 90
        };
    }

    static constexpr bool isAckPacket(std::span<const uint8_t> packet)
    {
        return packet.size() >= 2 && packet[0] == REPORT_PREFIX && packet[1] == 0x03;
    }

    static constexpr bool isPowerOffPacket(std::span<const uint8_t> packet)
    {
        return packet.size() >= 7 && packet[0] == REPORT_PREFIX && packet[1] == 0x05 && packet[6] == 0x00;
    }

    static constexpr bool isPowerEventPacket(std::span<const uint8_t> packet)
    {
        return packet.size() >= 2 && packet[0] == REPORT_PREFIX && packet[1] == 0x05;
    }

    static constexpr bool isBatteryResponsePacket(std::span<const uint8_t> packet)
    {
        return packet.size() >= 13 && packet[0] == REPORT_PREFIX && packet[1] == 0x0b && packet[8] == 0x04;
    }

    static Result<BatteryResult> parseBatteryResponse(std::span<const uint8_t> packet)
    {
        if (!isBatteryResponsePacket(packet)) {
            return DeviceError::protocolError("Unexpected battery response packet");
        }

        auto level = static_cast<int>(packet[10]);
        if (level > 100) {
            return DeviceError::protocolError("Battery percentage out of range");
        }

        auto status = packet[12] == 0x02 ? BATTERY_CHARGING : BATTERY_AVAILABLE;

        BatteryResult result {
            .level_percent = level,
            .status        = status,
        };

        return result;
    }

private:
    static constexpr std::array<uint8_t, PACKET_SIZE> buildBatteryRequest()
    {
        std::array<uint8_t, PACKET_SIZE> request {};
        request[0] = REPORT_PREFIX;
        request[1] = 0x08;
        request[3] = 0x03;
        request[4] = 0x1a;
        request[6] = 0x03;
        request[8] = 0x04;
        request[9] = 0x0a;
        return request;
    }

    static constexpr std::array<uint8_t, PACKET_SIZE> buildInactiveTimeCommand(uint8_t minutes)
    {
        std::array<uint8_t, PACKET_SIZE> command {};
        command[0]  = REPORT_PREFIX;
        command[1]  = 0x09;
        command[3]  = 0x03;
        command[4]  = 0x1c;
        command[6]  = 0x03;
        command[8]  = 0x06;
        command[9]  = 0x1d;
        command[10] = minutes;
        return command;
    }

    struct AdvancedEqBand {
        uint16_t frequency = 0;
        int8_t gain_db     = 0;
    };

    struct AdvancedEqDescriptor {
        uint8_t active_slot = 0;
        float gain_min      = -12.0f;
        float gain_max      = 12.0f;
        std::vector<AdvancedEqBand> bands;
    };

    static constexpr std::array<uint8_t, 2> buildPlaybackEqSelector(uint8_t slot)
    {
        return { PLAYBACK_DIRECTION, slot };
    }

    static constexpr int8_t decodeSignedByte(uint8_t value)
    {
        return static_cast<int8_t>(value);
    }

    static constexpr int8_t encodeGain(float gain)
    {
        return static_cast<int8_t>(gain);
    }

    Result<AdvancedEqDescriptor> readAdvancedEqDescriptor(hid_device* device_handle) const
    {
        auto info_reply = sendCenturionFeatureRequest(
            device_handle,
            static_cast<uint16_t>(protocols::CenturionFeature::HeadsetAdvancedParaEQ),
            0x00);
        if (!info_reply) {
            return info_reply.error();
        }

        if (info_reply->size() < 5) {
            return DeviceError::protocolError("Advanced EQ info response was too short");
        }

        auto active_slot_reply = sendCenturionFeatureRequest(
            device_handle,
            static_cast<uint16_t>(protocols::CenturionFeature::HeadsetAdvancedParaEQ),
            0x30,
            std::array<uint8_t, 1> { PLAYBACK_DIRECTION });
        if (!active_slot_reply) {
            return active_slot_reply.error();
        }
        if (active_slot_reply->empty()) {
            return DeviceError::protocolError("Advanced EQ active slot response was empty");
        }

        auto params_reply = sendCenturionFeatureRequest(
            device_handle,
            static_cast<uint16_t>(protocols::CenturionFeature::HeadsetAdvancedParaEQ),
            0x10,
            buildPlaybackEqSelector((*active_slot_reply)[0]));
        if (!params_reply) {
            return params_reply.error();
        }

        AdvancedEqDescriptor descriptor {
            .active_slot = (*active_slot_reply)[0],
            .gain_min    = static_cast<float>(decodeSignedByte((*info_reply)[3])),
            .gain_max    = static_cast<float>(decodeSignedByte((*info_reply)[4])),
        };

        for (size_t offset = 0; offset + 2 < params_reply->size(); offset += 3) {
            auto frequency = static_cast<uint16_t>(
                (static_cast<uint16_t>((*params_reply)[offset]) << 8)
                | static_cast<uint16_t>((*params_reply)[offset + 1]));
            if (frequency == 0) {
                break;
            }

            descriptor.bands.push_back(AdvancedEqBand {
                .frequency = frequency,
                .gain_db   = decodeSignedByte((*params_reply)[offset + 2])
            });
        }

        if (descriptor.bands.empty()) {
            return DeviceError::protocolError("Advanced EQ band response was empty");
        }

        cached_band_count_ = static_cast<int>(descriptor.bands.size());
        cached_gain_min_   = static_cast<int>(descriptor.gain_min);
        cached_gain_max_   = static_cast<int>(descriptor.gain_max);
        return descriptor;
    }

    Result<void> writePlaybackAdvancedEq(
        hid_device* device_handle,
        uint8_t slot,
        const std::vector<AdvancedEqBand>& bands) const
    {
        std::vector<uint8_t> payload;
        payload.reserve(2 + bands.size() * 3);
        payload.push_back(PLAYBACK_DIRECTION);
        payload.push_back(slot);

        for (const auto& band : bands) {
            payload.push_back(static_cast<uint8_t>((band.frequency >> 8) & 0xFF));
            payload.push_back(static_cast<uint8_t>(band.frequency & 0xFF));
            payload.push_back(static_cast<uint8_t>(band.gain_db));
        }

        if (auto write_reply = sendCenturionFeatureRequest(
                device_handle,
                static_cast<uint16_t>(protocols::CenturionFeature::HeadsetAdvancedParaEQ),
                0x20,
                payload);
            !write_reply) {
            return write_reply.error();
        }

        return {};
    }

    mutable int cached_band_count_ = 5;
    mutable int cached_gain_min_   = -12;
    mutable int cached_gain_max_   = 12;
};

} // namespace headsetcontrol
