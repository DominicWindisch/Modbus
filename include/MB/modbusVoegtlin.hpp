#ifndef MODBUS_VOEGTLIN_HPP
#define MODBUS_VOEGTLIN_HPP

#include <MB/Serial/connection.hpp>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

namespace GSC {
using namespace MB;

static ModbusParam MeasGasFlow{0x0000, f32t, "Measured Gas Flow Rate"};
static ModbusParam MeasTemperature{0x0002, f32t, "Measured Temperature"};
static ModbusParam Totaliser1{0x0004, f32t, "Total Gas Flow"};
static ModbusParam SetGasFlow{0x0006, f32t, "Setpoint Gas Flow Rate"};
static ModbusParam Alarms{0x000c, u16t, "Alarm flags"};
static ModbusParam HardwareErrors{0x000d, u16t, "Hardware error flags"};
static ModbusParam ControlFunction{0x000e, u16t, "Control mode flags"};
static ModbusParam SerialNum{0x001E, u32t, "Device Serial Number"};
static ModbusParam HardwareVersion{0x0020, u16t, "Hardware Version"};
static ModbusParam SoftwareVersion{0x0021, u16t, "Software Version"};
static ModbusParam TypeCode1{0x0023, s8t, "Type Code Part 1"};
static ModbusParam SoftReset{0x0034, u16t, "Soft Reset"};
static ModbusParam TypeCode2{0x1004, s8t, "Type Code Part 2"};
static ModbusParam MeasPointName{0x5000, s50t, "Measurement point name"};
static ModbusParam FluidNameLong{0x6022, s50t, "Fluid Name (long)"};
static ModbusParam FluidName{0x6042, s8t, "Fluid Name"};
static ModbusParam MeasUnit{0x6046, s8t, "Measuring Unit"};
static ModbusParam Totaliser2{0x6382, f32t, "Total Gas Flow (all time)"};
static ModbusParam TotaliserUnit{0x6386, s8t, "Totaliser Unit"};
} // namespace GSC

namespace MB {

static const std::map<uint16_t, MB::ModbusParam> Params{
    {0x0000, GSC::MeasGasFlow},     {0x0002, GSC::MeasTemperature},
    {0x0004, GSC::Totaliser1},      {0x0006, GSC::SetGasFlow},
    {0x000C, GSC::Alarms},          {0x000D, GSC::HardwareErrors},
    {0x000E, GSC::ControlFunction}, {0x001E, GSC::SerialNum},
    {0x0020, GSC::HardwareVersion}, {0x0021, GSC::SoftwareVersion},
    {0x0023, GSC::TypeCode1},       {0x0034, GSC::SoftReset},
    {0x1004, GSC::TypeCode2},       {0x5000, GSC::MeasPointName},
    {0x6022, GSC::FluidNameLong},   {0x6042, GSC::FluidName},
    {0x6046, GSC::MeasUnit},        {0x6382, GSC::Totaliser2},
    {0x6386, GSC::TotaliserUnit}};

class VoegtlinGSC

{
  public:
    explicit VoegtlinGSC(const std::string &path);

    template <typename T> auto readParam(MB::ModbusParam param) -> T;

    template <typename T> auto writeParam(MB::ModbusParam param, T data) -> bool;

  private:
    Serial::Connection m_conn;

    template <typename T>
    auto convertPayload(std::vector<uint8_t> msg, MB::DataType type, T &val) -> bool;
    auto writeParam(MB::ModbusParam param, std::vector<uint8_t> &data) -> bool;
    static auto showByte(const uint8_t &byte) -> void;
};

template <typename T> inline auto VoegtlinGSC::readParam(MB::ModbusParam param) -> T {
    auto numBytes = getNumBytesFromDataType(param.type);
    auto msg      = m_conn.sendRequest(param);
    if (msg[2] != numBytes) {
        std::cout << "Expected " << numBytes << "B for " << param.desc
                  << ", but response has " << (int)msg[2] << "B:";
        std::for_each(msg.begin(), msg.end(), showByte);
        std::cout << "\n";
    } else {
        T ret;
        if (convertPayload(msg, param.type, ret)) {
            return ret;
        }
    }
    return T{};
}

template <>
inline auto VoegtlinGSC::convertPayload(std::vector<uint8_t> msg, MB::DataType type,
                                        std::string &val) -> bool {
    auto numBytes = getNumBytesFromDataType(type);

    if (msg[2] == numBytes) {
        std::string ret;
        using namespace MB;
        switch (type) {
        case s50t:
        case s8t:
            val = std::string(reinterpret_cast<char *>(&msg[3]), numBytes);
            return true;
        }
    }

    return false;
}

template <>
inline auto VoegtlinGSC::convertPayload(std::vector<uint8_t> msg, MB::DataType type,
                                        float &val) -> bool {
    auto numBytes = getNumBytesFromDataType(type);
    union {
        float f;
        char c[4];
    } cnv;

    if (msg[2] == numBytes) {
        using namespace MB;
        switch (type) {
        case f32t:
            cnv.c[0] = msg[6];
            cnv.c[1] = msg[5];
            cnv.c[2] = msg[4];
            cnv.c[3] = msg[3];
            val      = cnv.f;
            return true;
        }
    }

    return false;
}

template <>
inline auto VoegtlinGSC::convertPayload(std::vector<uint8_t> msg, MB::DataType type,
                                        unsigned int &val) -> bool {
    auto numBytes = getNumBytesFromDataType(type);
    union {
        uint32_t u;
        char c[4];
    } cnv;

    if (msg[2] == numBytes) {
        using namespace MB;
        switch (type) {
        case u8t:
            val = msg[3];
            return true;

        case u16t:
            cnv.c[0] = 0;
            cnv.c[1] = 0;
            cnv.c[2] = msg[4];
            cnv.c[3] = msg[3];
            val      = cnv.u;
            return true;

        case u32t:
            cnv.c[0] = msg[6];
            cnv.c[1] = msg[5];
            cnv.c[2] = msg[4];
            cnv.c[3] = msg[3];
            val      = cnv.u;
            return true;
        }
    }
    return false;
}

template <>
inline auto VoegtlinGSC::convertPayload(std::vector<uint8_t> msg, MB::DataType type,
                                        uint16_t &val) -> bool {
    auto numBytes = getNumBytesFromDataType(type);
    union {
        uint32_t u;
        char c[4];
    } cnv;

    if (msg[2] == numBytes) {
        using namespace MB;
        switch (type) {
        case u8t:
            val = msg[3];
            return true;

        case u16t:
            cnv.c[0] = 0;
            cnv.c[1] = 0;
            cnv.c[2] = msg[4];
            cnv.c[3] = msg[3];
            val      = cnv.u;
            return true;

        case u32t:
            cnv.c[0] = msg[6];
            cnv.c[1] = msg[5];
            cnv.c[2] = msg[4];
            cnv.c[3] = msg[3];
            val      = cnv.u;
            return true;
        }
    }
    return false;
}

inline auto VoegtlinGSC::writeParam(MB::ModbusParam param, std::vector<uint8_t> &data)
    -> bool {
    auto msg = m_conn.sendRequest(param, true, data);

    if (msg.size() == 8 &&
        msg[1] == MB::utils::WriteMultipleAnalogOutputHoldingRegisters &&
        MB::utils::bigEndianConv(&msg[2]) == param.addr) {
        return true;
    }

    std::cout << "RESPONSE: ";
    std::for_each(msg.begin(), msg.end(), showByte);
    std::cout << "\n";
    return false;
}

template <>
inline auto VoegtlinGSC::writeParam(MB::ModbusParam param, float val) -> bool {
    union {
        float f;
        char c[4];
    } cnv;
    std::vector<uint8_t> data(4, 0);

    cnv.f   = val;
    data[0] = cnv.c[3];
    data[1] = cnv.c[2];
    data[2] = cnv.c[1];
    data[3] = cnv.c[0];
    return writeParam(param, data);
}

template <>
inline auto VoegtlinGSC::writeParam(MB::ModbusParam param, uint16_t val) -> bool {
    union {
        uint16_t u;
        char c[2];
    } cnv;
    std::vector<uint8_t> data(2, 0);

    cnv.u   = val;
    data[0] = cnv.c[1];
    data[1] = cnv.c[0];

    return writeParam(param, data);
}

template <>
inline auto VoegtlinGSC::writeParam(MB::ModbusParam param, uint32_t val) -> bool {
    union {
        uint32_t u;
        char c[4];
    } cnv;
    std::vector<uint8_t> data(4, 0);

    cnv.u   = val;
    data[0] = cnv.c[3];
    data[1] = cnv.c[2];
    data[2] = cnv.c[1];
    data[3] = cnv.c[0];
    return writeParam(param, data);
}

template <>
inline auto VoegtlinGSC::writeParam(MB::ModbusParam param, std::string str) -> bool {

    auto numBytes = getNumBytesFromDataType(param.type);
    if (str.size() > numBytes) {
        std::cout << "Provided string length (" << str.size()
                  << ") exceeds number of bytes in the type (" << numBytes << ").\n";
        return false;
    }

    std::vector<uint8_t> data(numBytes, 0x00); // init full length string with NULLs
    std::copy(str.begin(), str.end(), data.begin());

    return writeParam(param, data);
}

template <>
inline auto VoegtlinGSC::writeParam(MB::ModbusParam param, char const* str) -> bool {
    return writeParam(param, std::string(str));
}

} // namespace MB

#endif // MODBUS_VOEGTLIN_HPP