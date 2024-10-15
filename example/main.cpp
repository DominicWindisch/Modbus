// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#include <MB/modbusVoegtlin.hpp>

struct Spinner {
    const std::string chars = "/-\\|/-\\|";
    int charIdx             = 0;
    char next() { return (chars[(++charIdx) % 8]); }
};

int main(int argc, char *argv[]) {
    float gasFlowSetPoint = 0.0f;
    if (argc == 2) {
        gasFlowSetPoint = atof(argv[1]);
        std::cout << "Requested " << gasFlowSetPoint << " ln/min\n";
    }

    MB::VoegtlinGSC MFC("/dev/ttyUSB0");

    try {

        if (!MFC.writeParam(GSC::MeasPointName, "ROOF Messstelle")) {
            std::cout << "Error during writing meas point name.\n";
        }

        auto serialNum     = MFC.readParam<uint32_t>(GSC::SerialNum);
        auto type1         = MFC.readParam<std::string>(GSC::TypeCode1);
        auto type2         = MFC.readParam<std::string>(GSC::TypeCode2);
        auto measPointName = MFC.readParam<std::string>(GSC::MeasPointName);

        std::cout << "Connected to '" << measPointName << "' (" << type1 << "-" << type2
                  << " , SerialNum: " << serialNum << ")\n";

        Spinner spinner;

        static uint16_t controlModeAutomatic = 0u;
        static uint16_t controlModeDigital   = 1u;
        static uint16_t testVentilClosed     = 22u;
        static uint16_t testVentilOpen       = 23u;

        if (!MFC.writeParam(GSC::ControlFunction, controlModeDigital)) {
            std::cout << "Error during writing control function.\n";
        };

        auto unit = MFC.readParam<std::string>(GSC::MeasUnit);
        if (!MFC.writeParam(GSC::SetGasFlow, gasFlowSetPoint)) {
            std::cout << "Error when setting the gas flow setpoint.\n";
            return EXIT_FAILURE;
        }

        for (auto i = 0; i < 24; i++) {
            auto temperature = MFC.readParam<float>(GSC::MeasTemperature);
            auto flow        = MFC.readParam<float>(GSC::MeasGasFlow);
            auto setp        = MFC.readParam<float>(GSC::SetGasFlow);
            auto status      = MFC.readParam<uint16_t>(GSC::Alarms);
            auto hwError     = MFC.readParam<uint16_t>(GSC::HardwareErrors);

            std::cout << "\r" << spinner.next() << " Status (" << status << "|" << hwError
                      << "): " << flow << " " << unit << " / " << setp << " " << unit
                      << " | " << temperature << "C" << std::flush;
        }
        std::cout << "\nTerminating..." << std::flush;

        MFC.writeParam(GSC::SetGasFlow, 0.0f);
        std::cout << "done.\n";

    } catch (std::runtime_error &e) {
        std::cout << "Something went wrong: " << e.what() << "\n";
    } catch (MB::ModbusException &e) {
        std::cout << "Modbus exception: " << e.toString() << "\n";
    } catch (...) {
        std::cout << "Unknown error\n";
    }
}
