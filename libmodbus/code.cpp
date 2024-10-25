#include <iostream>
#include "modbus.h"
#include <vector>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <cstdint>
#include <algorithm>

#define pi 3.141592653589793
class Controller {
public:
    Controller(const std::string &port = "COM7") {
        // Initialize Modbus context
        ctx = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1);
        if (ctx == nullptr) {
            throw std::runtime_error("Unable to allocate modbus context"); 
        }
        modbus_set_slave(ctx, 1);  // Set slave ID to 1

        // Connect to the Modbus device
        if (modbus_connect(ctx) == -1) {
            throw std::runtime_error("Connection failed");
        }

        // Initialize fault list
        FAULT_LIST = {OVER_VOLT, UNDER_VOLT, OVER_CURR, OVER_LOAD, CURR_OUT_TOL,
                      ENCOD_OUT_TOL, MOTOR_BAD, REF_VOLT_ERROR, EEPROM_ERROR, WALL_ERROR, HIGH_TEMP};
    }

    ~Controller() {
        modbus_close(ctx);
        modbus_free(ctx);
    }
    
    double rpm_to_radPerSec(double rpm) 
    {
        return rpm * 2 * pi / 60;
    }
    double rpm_to_linear(double rpm)
    {
        double W_Wheel = rpm_to_radPerSec(rpm);
        return W_Wheel * R_Wheel;
    }
    void set_mode(int mode) {
        if (mode < 1 || mode > 3) {
            std::cerr << "set_mode ERROR: set only 1, 2, or 3" << std::endl;
            return;
        }
        if(mode == 1) std::cout << "POS_REL_CONTROL" << std::endl;
        else if(mode == 2) std::cout << "POS_ABS_CONTROL" << std::endl;
        else if(mode == 3) std::cout << "VEL_CONTROL" << std::endl;
        modbus_write_register(ctx, OPR_MODE, mode);
    }

    //not fixed
    int get_mode() {
    uint16_t mode_register;
    
    // Read 1 holding register starting from OPR_MODE
    int rc = modbus_read_registers(ctx, OPR_MODE, 1, &mode_register);
    
    // Error handling
    if (rc == -1) {
        std::cerr << "Error reading OPR_MODE register: " << modbus_strerror(errno) << std::endl;
        throw std::runtime_error("Failed to read OPR_MODE register");
    }
    
    // Return the mode value
    return mode_register;
}


    void enable_motor() {
        modbus_write_register(ctx, CONTROL_REG, ENABLE);
    }

    void disable_motor() {
        modbus_write_register(ctx, CONTROL_REG, DOWN_TIME);
    }

    std::pair<bool, uint16_t> get_fault_code() {
        uint16_t fault_codes[2];
        modbus_read_registers(ctx, L_FAULT, 2 , fault_codes);
        bool L_fault_flag = std::find(FAULT_LIST.begin(), FAULT_LIST.end(), fault_codes[0]) != FAULT_LIST.end();
        bool R_fault_flag = std::find(FAULT_LIST.begin(), FAULT_LIST.end(), fault_codes[1]) != FAULT_LIST.end();
        return {L_fault_flag, R_fault_flag};
    }

    void clear_alarm() {
        modbus_write_register(ctx, CONTROL_REG, ALRM_CLR);
    }

    void set_accel_time(int L_ms, int R_ms) {
        if(L_ms >32767) L_ms = 32767;
        else if(L_ms < 0) L_ms = 0;
        if(R_ms >32767) R_ms = 32767;
        else if(R_ms < 0) R_ms = 0;
        uint16_t values[2] = {static_cast<uint16_t>(L_ms), static_cast<uint16_t>(R_ms)};
        modbus_write_registers(ctx, L_ACL_TIME, 2, values);
    }

    void set_decel_time(int L_ms, int R_ms) {
        if(L_ms >32767) L_ms = 32767;
        else if(L_ms < 0) L_ms = 0;
        if(R_ms >32767) R_ms = 32767;
        else if(R_ms < 0) R_ms = 0;
        uint16_t values[2] = {static_cast<uint16_t>(L_ms), static_cast<uint16_t>(R_ms)};
        modbus_write_registers(ctx, L_DCL_TIME, 2, values);
    }

    uint16_t int16_to_uint16(int value) 
    {
        return (uint16_t)value;
    }
    void set_rpm(int16_t L_rpm, int16_t R_rpm) {
        if(L_rpm > 3000) L_rpm = 3000;
        else if(L_rpm < -3000) L_rpm = -3000;
        if(R_rpm > 3000) R_rpm = 3000;
        else if(R_rpm < -3000) R_rpm = -3000;
        uint16_t left_bytes = int16_to_uint16(L_rpm);
        uint16_t right_bytes = int16_to_uint16(R_rpm);
        
        uint16_t values[2] = {left_bytes, right_bytes};
        modbus_write_registers(ctx, L_CMD_RPM, 2, values);
    }

    std::pair<double, double> get_rpm() {
        uint16_t registers[2];
        modbus_read_registers(ctx, L_FB_RPM, 2, registers);
        double fb_L_rpm = static_cast<int16_t>(registers[0]) / 10.0;
        double fb_R_rpm = static_cast<int16_t>(registers[1]) / 10.0;
        return {fb_L_rpm, fb_R_rpm};
    }


    std::pair<double, double> get_linear_velocities() {
        auto [rpmL, rpmR] = get_rpm();
        return {rpm_to_linear(rpmL), rpm_to_linear(-rpmR)};
    }

    void set_maxRPM_pos(int max_L_rpm, int max_R_rpm) {
        if(max_L_rpm > 1000) max_L_rpm = 1000;
        else if(max_L_rpm < 1) max_L_rpm = 1;
        if(max_R_rpm > 1000) max_R_rpm = 1000;
        else if(max_R_rpm < 1) max_R_rpm = 1;
        uint16_t values[2] = {static_cast<uint16_t>(max_L_rpm), static_cast<uint16_t>(max_R_rpm)};
        modbus_write_registers(ctx, L_MAX_RPM_POS, 2, values);
    }
    
     int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Function to convert degrees to a 32-bit array (HI_WORD and LO_WORD)
    std::vector<uint16_t> deg_to_32bitArray(int32_t deg) {
        int32_t dec = map(deg, -1440, 1440, -65536, 65536);
        uint16_t HI_WORD = (dec & 0xFFFF0000) >> 16;
        uint16_t LO_WORD = dec & 0x0000FFFF;
        return {HI_WORD, LO_WORD};
    }

    // Function to set relative angles for left and right motors
    void set_relative_angle(int32_t ang_L, int32_t ang_R) {
        std::vector<uint16_t> L_array = deg_to_32bitArray(ang_L);
        std::vector<uint16_t> R_array = deg_to_32bitArray(ang_R);

        std::vector<uint16_t> all_cmds_array;
        all_cmds_array.insert(all_cmds_array.end(), L_array.begin(), L_array.end());
        all_cmds_array.insert(all_cmds_array.end(), R_array.begin(), R_array.end());
        modbus_write_registers(ctx, L_CMD_REL_POS_HI, all_cmds_array.size(), all_cmds_array.data());
    }

    std::pair<double, double> get_wheels_travelled() {
        uint16_t registers[4];
        modbus_read_registers(ctx, L_FB_POS_HI, 4, registers);
        int32_t l_pulse = ((registers[0] & 0xFFFF) << 16) | (registers[1] & 0xFFFF);
        int32_t r_pulse = ((registers[2] & 0xFFFF) << 16) | (registers[3] & 0xFFFF);
        double L_meters = travel_in_one_rev * l_pulse / cpr;
        double R_meters = travel_in_one_rev * r_pulse / cpr;
        return {L_meters, R_meters};
    }

private:
    modbus_t *ctx;
    std::vector<uint16_t> FAULT_LIST;
    /*
        *Register Address
        */

        // Initialize register addresses
        uint16_t CONTROL_REG = 0x200E;
        uint16_t OPR_MODE = 0x200D;
        uint16_t L_ACL_TIME = 0x2080;
        uint16_t R_ACL_TIME = 0x2081;
        uint16_t L_DCL_TIME = 0x2082;
        uint16_t R_DCL_TIME = 0x2083;

        //Velocity control
        uint16_t L_CMD_RPM = 0x2088;
        uint16_t R_CMD_RPM = 0x2089;
        uint16_t L_FB_RPM = 0x20AB;
        uint16_t R_FB_RPM = 0x20AC;

        //Position control
        uint16_t POS_CONTROL_TYPE = 0x200F;

        uint16_t L_MAX_RPM_POS = 0x208E;
        uint16_t R_MAX_RPM_POS = 0x208F;

        uint16_t L_CMD_REL_POS_HI = 0x208A;
        uint16_t L_CMD_REL_POS_LO = 0x208B;
        uint16_t R_CMD_REL_POS_HI = 0x208C;
        uint16_t R_CMD_REL_POS_LO = 0x208D;

        uint16_t L_FB_POS_HI = 0x20A7;
        uint16_t L_FB_POS_LO = 0x20A8;
        uint16_t R_FB_POS_HI = 0x20A9;
        uint16_t R_FB_POS_LO = 0x20AA;

        //Troubleshooting
        uint16_t L_FAULT = 0x20A5;
        uint16_t R_FAULT = 0x20A6;

        /*
        *Control commands
        */
        uint16_t EMER_STOP = 0x05;
        uint16_t ALRM_CLR = 0x06;
        uint16_t DOWN_TIME = 0x07;
        uint16_t ENABLE = 0x08;
        uint16_t POS_SYNC = 0x10;
        uint16_t POS_L_START = 0x11;
        uint16_t POS_R_START = 0x12;

        /*
        *Operation modes
        */
        uint16_t POS_REL_CONTROL = 1;
        uint16_t POS_ABS_CONTROL = 2;
        uint16_t VEL_CONTROL = 3;

        /*
        *Fault codes
        */
        uint16_t NO_FAULT = 0x0000;
        uint16_t OVER_VOLT = 0x0001;
        uint16_t UNDER_VOLT = 0x0002;
        uint16_t OVER_CURR = 0x0004;
        uint16_t OVER_LOAD = 0x0008;
        uint16_t CURR_OUT_TOL = 0x0010;
        uint16_t ENCOD_OUT_TOL = 0x0020;
        uint16_t MOTOR_BAD = 0x0040;
        uint16_t REF_VOLT_ERROR = 0x0080;
        uint16_t EEPROM_ERROR = 0x0100;
        uint16_t WALL_ERROR = 0x0200;
        uint16_t HIGH_TEMP = 0x0400;

        // Wheel parameters
        double travel_in_one_rev = 0.655; // meters
        int cpr = 16385; // counts per revolution
        double R_Wheel = 0.105; // meters
};