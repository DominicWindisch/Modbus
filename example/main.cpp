// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#include "MB/modbusException.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "Serial/connection.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <chrono>
#include <thread>


namespace GSC {
	using namespace MB;

	static ModbusParam MeasGasFlow{ 0x0000, f32t, "Measured Gas Flow Rate" };
	static ModbusParam MeasTemperature{ 0x0002, f32t, "Measured Temperature" };
	static ModbusParam Totaliser1{ 0x0004, f32t, "Total Gas Flow" };
	static ModbusParam SetGasFlow{ 0x0006, f32t, "Setpoint Gas Flow Rate" };
	static ModbusParam Alarms{ 0x000c, u16t, "Alarm flags" };
	static ModbusParam HardwareErrors{ 0x000d, u16t, "Hardware error flags" };
	static ModbusParam ControlFunction{ 0x000e, u16t, "Control mode flags" };
	static ModbusParam SerialNum{ 0x001E, u32t, "Device Serial Number" };
	static ModbusParam HardwareVersion{ 0x0020, u16t, "Hardware Version" };
	static ModbusParam SoftwareVersion{ 0x0021, u16t, "Software Version" };
	static ModbusParam TypeCode1{ 0x0023, s8t, "Type Code Part 1" };
	static ModbusParam SoftReset{ 0x0034, u16t, "Soft Reset" };
	static ModbusParam TypeCode2{ 0x1004, s8t, "Type Code Part 2" };
	static ModbusParam MeasPointName{ 0x5000, s50t, "Measurement point name" };
	static ModbusParam FluidNameLong{ 0x6022, s50t, "Fluid Name (long)" };
	static ModbusParam FluidName{ 0x6042, s8t, "Fluid Name" };
	static ModbusParam MeasUnit{ 0x6046, s8t, "Measuring Unit" };
	static ModbusParam Totaliser2{ 0x6382, f32t, "Total Gas Flow (all time)" };
	static ModbusParam TotaliserUnit{ 0x6386, s8t, "Totaliser Unit" };
}

static std::map<uint16_t, MB::ModbusParam> Params{
	{0x0000, GSC::MeasGasFlow},
	{0x0002, GSC::MeasTemperature},
	{0x0004, GSC::Totaliser1},
	{0x0006, GSC::SetGasFlow},
	{0x000C, GSC::Alarms},
	{0x000D, GSC::HardwareErrors},
	{0x000E, GSC::ControlFunction},
	{0x001E, GSC::SerialNum},
	{0x0020, GSC::HardwareVersion},
	{0x0021, GSC::SoftwareVersion},
	{0x0023, GSC::TypeCode1},
	{0x0034, GSC::SoftReset},
	{0x1004, GSC::TypeCode2},
	{0x5000, GSC::MeasPointName},
	{0x6022, GSC::FluidNameLong},
	{0x6042, GSC::FluidName},
	{0x6046, GSC::MeasUnit},
	{0x6382, GSC::Totaliser2},
	{0x6386, GSC::TotaliserUnit}
};

MB::DataType getDataTypeFromAddr(uint16_t addr) {
	return Params[addr].type;
}

static auto showByte = [](const uint8_t& byte) {
	std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0')
		<< static_cast<int>(byte);
	};

std::string convertPayload(std::vector<uint8_t> msg, MB::DataType type) {
	auto numBytes = getNumBytesFromDataType(type);
	union {
		float f;
		uint32_t u;
		char c[4];
	} cnv;

	if (msg[2] == numBytes) {
		std::string ret;
		using namespace MB;
		switch (type) {
		case s50t:
		case s8t:
			return std::string(reinterpret_cast<char*>(&msg[3]), numBytes);
		}
		return ("Not a string type");
	}
	return ("Invalid message length");
}

bool convertPayload(std::vector<uint8_t> msg, MB::DataType type, std::string& val) {
	auto numBytes = getNumBytesFromDataType(type);

	if (msg[2] == numBytes) {
		std::string ret;
		using namespace MB;
		switch (type) {
		case s50t:
		case s8t:
			val = std::string(reinterpret_cast<char*>(&msg[3]), numBytes);
			return true;
		}
	}

	return false;
}

bool convertPayload(std::vector<uint8_t> msg, MB::DataType type, float& val) {
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
			val = cnv.f;
			return true;
		}
	}

	return false;
}

bool convertPayload(std::vector<uint8_t> msg, MB::DataType type, unsigned int& val) {
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
			val = cnv.u;
			return true;

		case u32t:
			cnv.c[0] = msg[6];
			cnv.c[1] = msg[5];
			cnv.c[2] = msg[4];
			cnv.c[3] = msg[3];
			val = cnv.u;
			return true;
		}
	}
	return false;
}

bool convertPayload(std::vector<uint8_t> msg, MB::DataType type, uint16_t& val) {
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
			val = cnv.u;
			return true;

		case u32t:
			cnv.c[0] = msg[6];
			cnv.c[1] = msg[5];
			cnv.c[2] = msg[4];
			cnv.c[3] = msg[3];
			val = cnv.u;
			return true;
		}
	}
	return false;
}

std::string convertPayloadToString(std::vector<uint8_t> msg, MB::DataType type) {
	auto numBytes = getNumBytesFromDataType(type);
	union {
		float f;
		uint32_t u;
		char c[4];
	} cnv;

	if (msg[2] == numBytes) {
		std::string ret;
		using namespace MB;
		switch (type) {
		case s50t:
		case s8t:
			return std::string(reinterpret_cast<char*>(&msg[3]), numBytes);

		case u8t:
			return std::to_string(msg[3]);

		case u16t:
			cnv.c[0] = 0;
			cnv.c[1] = 0;
			cnv.c[2] = msg[4];
			cnv.c[3] = msg[3];
			return std::to_string(cnv.u);

		case u32t:
			cnv.c[0] = msg[6];
			cnv.c[1] = msg[5];
			cnv.c[2] = msg[4];
			cnv.c[3] = msg[3];
			return std::to_string(cnv.u);

		case f32t:
			cnv.c[0] = msg[6];
			cnv.c[1] = msg[5];
			cnv.c[2] = msg[4];
			cnv.c[3] = msg[3];

			return std::to_string(cnv.f);
		}
	}

	return ("Invalid message length");

}

void handleResponse(std::vector<uint8_t>& msg, uint16_t addr) {
	MB::DataType type = getDataTypeFromAddr(addr);
	auto numBytes = getNumBytesFromDataType(type);
	if (msg[2] != numBytes) {
		std::cout << "Expected " << numBytes << "B for " << Params[addr].desc << ", but response has " << (int)msg[2] << "B:";
		std::for_each(msg.begin(), msg.end(), showByte);
		std::cout << "\n";
	}
	else {
		std::cout << " => " << convertPayloadToString(msg, type) << "\n";
	}
}

MB::ModbusRequest createRequest(MB::ModbusParam param) {
	static int modbusAddr = 247;
	return MB::ModbusRequest(modbusAddr, MB::utils::ReadAnalogOutputHoldingRegisters, param.addr, getNumBytesFromDataType(param.type) / 2);
}

template<typename T>
auto readParam(MB::Serial::Connection& conn, MB::ModbusParam param) -> T {
	auto numBytes = getNumBytesFromDataType(param.type);
	auto msg = conn.sendRequest(param);
	if (msg[2] != numBytes) {
		std::cout << "Expected " << numBytes << "B for " << param.desc << ", but response has " << (int)msg[2] << "B:";
		std::for_each(msg.begin(), msg.end(), showByte);
		std::cout << "\n";
	}
	else {
		T ret;
		if (convertPayload(msg, param.type, ret)) {
			return ret;
		}
	}
	return T{};
}

auto writeParam(MB::Serial::Connection& conn, MB::ModbusParam param, std::vector<uint8_t>& data) -> bool {
	auto msg = conn.sendRequest(param, true, data);

	if (msg.size() == 8
		&& msg[1] == MB::utils::WriteMultipleAnalogOutputHoldingRegisters
		&& MB::utils::bigEndianConv(&msg[2]) == param.addr)
	{
		return true;
	}

	std::cout << "RESPONSE: ";
	std::for_each(msg.begin(), msg.end(), showByte);
	std::cout << "\n";
	return false;
}

auto writeParam(MB::Serial::Connection& conn, MB::ModbusParam param, float val) -> bool {
	union {
		float f;
		char c[4];
	} cnv;
	std::vector<uint8_t> data(4, 0);

	cnv.f = val;
	data[0] = cnv.c[3];
	data[1] = cnv.c[2];
	data[2] = cnv.c[1];
	data[3] = cnv.c[0];
	return writeParam(conn, param, data);
}

auto writeParam(MB::Serial::Connection& conn, MB::ModbusParam param, uint16_t val) -> bool {
	union {
		uint16_t u;
		char c[2];
	} cnv;
	std::vector<uint8_t> data(2, 0);

	cnv.u = val;
	data[0] = cnv.c[1];
	data[1] = cnv.c[0];

	return writeParam(conn, param, data);
}

struct Spinner {
	const std::string chars = "/-\\|/-\\|";
	int charIdx = 0;
	char next() {
		return (chars[(++charIdx) % 8]);
	}
};

void sendRequests(float gasFlowSetPoint) {

	MB::Serial::Connection myConn("/dev/ttyUSB0");
	myConn.connect();
	myConn.setBaudRate(9600);
	myConn.setTwoStopBits(true);
	myConn.enableParity(false);
	myConn.setTimeout(1500);

	Spinner spinner;

	try {
		auto serialNum = readParam<uint32_t>(myConn, GSC::SerialNum);
		auto type1 = readParam<std::string>(myConn, GSC::TypeCode1);
		auto type2 = readParam<std::string>(myConn, GSC::TypeCode2);
		auto measPointName = readParam<std::string>(myConn, GSC::MeasPointName);

		std::cout << "Connected to '" << measPointName << "' (" << type1 << "-" << type2 << " , SerialNum: " << serialNum << ")\n";

		std::string myMeasPointName = "ROOF Messstelle";
		std::vector<uint8_t> wrData(50, 0x00); // init full length string with NULLs
		std::copy(myMeasPointName.begin(), myMeasPointName.end(), wrData.begin());
		if (writeParam(myConn, GSC::MeasPointName, wrData)) {
			std::cout << "Write succeeded!\n";
		}
		else {
			std::cout << "Error during write: ";
		}

		static uint16_t controlModeAutomatic = 0u;
		static uint16_t controlModeDigital = 1u;
		static uint16_t testVentilClosed = 22u;
		static uint16_t testVentilOpen = 23u;

		if (writeParam(myConn, GSC::ControlFunction, controlModeDigital)) {
			std::cout << "Write succeeded!\n";
		}
		else {
			std::cout << "Error during write: ";
		}

		auto unit = readParam<std::string>(myConn, GSC::MeasUnit);
		if (!writeParam(myConn, GSC::SetGasFlow, gasFlowSetPoint)) {
			std::cout<<"Error when setting the gas flow setpoint.\n";
			return;
		}
		auto setp = readParam<float>(myConn, GSC::SetGasFlow);
		std::cout << ">> GasFlow setpoint is " << setp << " " << unit << "\n";


		for (auto i = 0; i < 2000; i++) {
			auto temperature = readParam<float>(myConn, GSC::MeasTemperature);
			auto flow = readParam<float>(myConn, GSC::MeasGasFlow);
			setp = readParam<float>(myConn, GSC::SetGasFlow);
			auto status = readParam<uint16_t>(myConn, GSC::Alarms);
			auto hwError = readParam<uint16_t>(myConn, GSC::HardwareErrors);
			std::cout << "\r" << spinner.next() << " Status ("<<status<<"|"<<hwError << "): " << flow << " " << unit << " / " << setp << " " << unit << " | " << temperature << "C" << std::flush;
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		std::cout << "\n";


	}
	catch (std::runtime_error& e) {
		std::cout << "Something went wrong: " << e.what() << "\n";
	}
	catch (MB::ModbusException& e) {
		std::cout << "Modbus exception: " << e.toString() << "\n";
	}
	catch (...) {
		std::cout << "Unknown error\n";
	}

	myConn.close();
}

int main(int argc, char *argv[]) {
	float setPoint = 0.0f;
	if (argc == 2)
	{
		setPoint = atof(argv[1]);
		std::cout << "Requested " << setPoint << " ln/min\n";
	}
	
	sendRequests(setPoint); 
}
