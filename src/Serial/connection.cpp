// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#include "Serial/connection.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace MB::Serial;

Connection::Connection(const std::string& path) {
	open(path);
	_lastSendTime = std::chrono::high_resolution_clock::now();
}

void Connection::open(const std::string& path) {
	_fd = ::open(path.c_str(), O_RDWR | O_SYNC | O_NONBLOCK);

	if (_fd < 0) {
		throw std::runtime_error("Cannot open serial port " + path);
	}

	if (tcgetattr(_fd, &_termios) != 0) {
		throw std::runtime_error("Error at tcgetattr - " +
			std::to_string(errno));
	}

	cfmakeraw(&_termios);

	_termios.c_iflag &= ~(PARMRK | INPCK);
	_termios.c_iflag |= IGNPAR;
}

void Connection::connect() {
	tcflush(_fd, TCIFLUSH);
	if (tcsetattr(_fd, TCSAFLUSH, &_termios) != 0) {
		throw std::runtime_error("Error {" + std::to_string(_fd) +
			"} at tcsetattr - " + std::to_string(errno));
	}
}

Connection::~Connection() {
	close();
}

void Connection::close() {
	if (_fd >= 0) ::close(_fd);
	_fd = -1;
}

std::vector<uint8_t> Connection::sendRequest(const MB::ModbusParam& param, bool writeParam, const std::vector<uint8_t>& data) {
	static int modbusAddr = 247;
	if (!writeParam) {
		MB::ModbusRequest req(modbusAddr, MB::utils::ReadAnalogOutputHoldingRegisters, param.addr, getNumBytesFromDataType(param.type) / 2);
		const auto expectedLen = 8 + 5 + getNumBytesFromDataType(param.type); // 8 for request, 5 for response overhead (addr, function, length, crc)
		return sendRequest(req, expectedLen);
	}
	else {
		MB::ModbusRequest req(modbusAddr, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, param.addr, getNumBytesFromDataType(param.type) / 2);
		std::vector<MB::ModbusCell> vals;
		for (auto idx = 0; idx < data.size(); idx += 2) {
			vals.push_back(utils::bigEndianConv(&data[idx]));
		}
		req.setValues(vals);
		const auto requestLength = 9 + data.size();
		const auto expectedLen = requestLength + 8;

		return sendRequest(req, expectedLen, requestLength);
	}
}

std::vector<uint8_t> Connection::sendRequest(const MB::ModbusRequest& request, const int expectedResponseLength, const int requestLength) {
	auto sent = send(request.toRaw());
	if (expectedResponseLength != 0) {
		auto resp = readRawMessage(expectedResponseLength);
		if (expectedResponseLength > 0)
		{
			resp.erase(resp.begin(), resp.begin() + requestLength); // erase echoed response
		}
		return resp;
	}
	return sent;
}

std::vector<uint8_t> Connection::sendResponse(const MB::ModbusResponse& response) {
	return send(response.toRaw());
}

std::vector<uint8_t> Connection::sendException(const MB::ModbusException& exception) {
	return send(exception.toRaw());
}

std::vector<uint8_t> Connection::awaitRawMessage() {
	std::vector<uint8_t> data(1024);

	pollfd waitingFD = { .fd = _fd, .events = POLLIN, .revents = POLLIN };

	if (::poll(&waitingFD, 1, _timeout) <= 0) {
		throw MB::ModbusException(MB::utils::Timeout);
	}

	auto size = ::read(_fd, data.begin().base(), 1024);

	if (size < 0) {
		throw MB::ModbusException(MB::utils::SlaveDeviceFailure);
	}

	data.resize(size);
	data.shrink_to_fit();

	return data;
}

std::vector<uint8_t> Connection::readRawMessage(const int expectedResponseLength) {

	if (expectedResponseLength == 0) {
		// read whatever data is available

		std::vector<uint8_t> data(1024);
		auto size = ::read(_fd, data.begin().base(), 1024);

		if (size < 0) {
			if (errno == EBADF) {
				std::cout << "Connection closed during read call\n";
			}
			else if (errno != EAGAIN) {
				std::cout << "errno: " << errno << " | fd = " << _fd << "\n";
				throw MB::ModbusException(MB::utils::SlaveDeviceFailure);
			}
			data.clear();
			return data;
		}

		data.resize(size);
		data.shrink_to_fit();
		_lastSendTime = std::chrono::high_resolution_clock::now();
		return data;
	}
	else if (expectedResponseLength > 0) {
		// read exactly expectedResponseLength bytes

		int numReadBytes = 0;
		std::vector<uint8_t> data(1024);
		while (numReadBytes < expectedResponseLength) {
			auto size = ::read(_fd, data.begin().base() + numReadBytes, 1024 - numReadBytes);
			if (size > 0) {
				numReadBytes += size;
				//std::cout << " ( got " << numReadBytes << " of " << expectedResponseLength << ") ";
				if (numReadBytes >= expectedResponseLength) {
					std::vector<uint8_t> resp(data.begin(), data.begin() + expectedResponseLength);
					_lastSendTime = std::chrono::high_resolution_clock::now();
					return resp;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	else {
		// negative value => wait for however many bytes there are within a given timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(-expectedResponseLength));
		std::vector<uint8_t> data(1024);
		auto size = ::read(_fd, data.begin().base(), 1024);
		data.resize(size);
		data.shrink_to_fit();
		_lastSendTime = std::chrono::high_resolution_clock::now();
		return data;
	}

	return std::vector<uint8_t>();//unreachable
}

// TODO: Figure out how to return raw data when exception is being thrown
std::tuple<MB::ModbusResponse, std::vector<uint8_t>> Connection::awaitResponse() {
	std::vector<uint8_t> data;
	data.reserve(8);

	MB::ModbusResponse response(0, MB::utils::ReadAnalogInputRegisters);

	while (true) {
		try {
			auto tmpResponse = awaitRawMessage();
			data.insert(data.end(), tmpResponse.begin(), tmpResponse.end());

			if (MB::ModbusException::exist(data)) throw MB::ModbusException(data);

			response = MB::ModbusResponse::fromRawCRC(data);
			break;
		}
		catch (const MB::ModbusException& ex) {
			if (MB::utils::isStandardErrorCode(ex.getErrorCode()) || ex.getErrorCode() == MB::utils::Timeout || ex.getErrorCode() == MB::utils::SlaveDeviceFailure) throw ex;
			continue;
		}
	}

	return std::tie(response, data);
}

std::tuple<MB::ModbusRequest, std::vector<uint8_t>> Connection::awaitRequest() {
	std::vector<uint8_t> data;
	data.reserve(8);

	MB::ModbusRequest request(0, MB::utils::ReadAnalogInputRegisters);

	while (true) {
		try {
			auto tmpResponse = awaitRawMessage();
			data.insert(data.end(), tmpResponse.begin(), tmpResponse.end());

			request = MB::ModbusRequest::fromRawCRC(data);
			break;
		}
		catch (const MB::ModbusException& ex) {
			if (ex.getErrorCode() == MB::utils::Timeout || ex.getErrorCode() == MB::utils::SlaveDeviceFailure) throw ex;
			continue;
		}
	}

	return std::tie(request, data);
}

std::vector<uint8_t> Connection::send(std::vector<uint8_t> data) {
	data.reserve(data.size() + 2);
	const auto crc = utils::calculateCRC(data.begin().base(), data.size());

	data.push_back(reinterpret_cast<const uint8_t*>(&crc)[0]);
	data.push_back(reinterpret_cast<const uint8_t*>(&crc)[1]);

	auto nextSendTime = _lastSendTime + std::chrono::milliseconds(MinPauseBetweenSendingMS);
	std::this_thread::sleep_until(nextSendTime);

	// Ensure that nothing will intervene in our communication
	// WARNING: It may conflict with something (although it may also help in
	// most cases)
	tcflush(_fd, TCOFLUSH);
	// Write
	std::ignore = write(_fd, data.begin().base(), data.size());
	// It may be a good idea to use tcdrain, although it has tendency to not
	// work as expected tcdrain(_fd);
	
	return data;
}

Connection::Connection(Connection&& moved) noexcept {
	_fd = moved._fd;
	_termios = moved._termios;
	moved._fd = -1;
}

Connection& Connection::operator=(Connection&& moved) {
	if (this == &moved) return *this;

	_fd = moved._fd;
	memcpy(&_termios, &(moved._termios), sizeof(moved._termios));
	moved._fd = -1;
	return *this;
}
