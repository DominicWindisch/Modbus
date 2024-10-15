#include <MB/modbusVoegtlin.hpp>

namespace MB {

VoegtlinGSC::VoegtlinGSC(const std::string &path) {

    m_conn = MB::Serial::Connection(path);
    m_conn.connect();
    m_conn.setBaudRate(9600);
    m_conn.setTwoStopBits(true);
    m_conn.enableParity(false);
    m_conn.setTimeout(1500);
}

auto VoegtlinGSC::showByte(const uint8_t &byte) -> void {
    std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(byte);
};

} // namespace MB