// ============================================================================
// FILE: satellite_doppler_calculator.cc
// DESCRIPTION: Implementation of Satellite Doppler Calculator
// ============================================================================

#include "doppler_calculator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// SATELLITE STATE CONSTRUCTOR
// ============================================================================

SatelliteDopplerCalculator::SatelliteState::SatelliteState()
    : x(0), y(0), z(0), vx(0), vy(0), vz(0), valid(false) {
    std::memset(&epoch_time, 0, sizeof(epoch_time));
}

// ============================================================================
// CONSTRUCTORS AND DESTRUCTOR
// ============================================================================

SatelliteDopplerCalculator::SatelliteDopplerCalculator()
    : sp3_file_path(""),
      receiver_lat(0.0),
      receiver_lon(0.0),
      receiver_alt(0.0),
      rx_x(0), rx_y(0), rx_z(0) {
}

SatelliteDopplerCalculator::SatelliteDopplerCalculator(
    const std::string& sp3_path,
    double lat, double lon, double alt)
    : sp3_file_path(sp3_path),
      receiver_lat(lat),
      receiver_lon(lon),
      receiver_alt(alt) {
    
    convertLLAtoECEF(lat, lon, alt, rx_x, rx_y, rx_z);
    std::cout << "Receiver position (ECEF): (" 
              << rx_x << ", " << rx_y << ", " << rx_z << ")" << std::endl;
}

SatelliteDopplerCalculator::~SatelliteDopplerCalculator() {
    satellite_data.clear();
}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

void SatelliteDopplerCalculator::setSP3FilePath(const std::string& path) {
    sp3_file_path = path;
}

void SatelliteDopplerCalculator::setReceiverPosition(double lat, double lon, double alt) {
    receiver_lat = lat;
    receiver_lon = lon;
    receiver_alt = alt;
    convertLLAtoECEF(lat, lon, alt, rx_x, rx_y, rx_z);
    // rx_x = 1429140.63;
    // rx_y = 6143753.36;
    // rx_z = 941589.32;
    //Surveyed position
    rx_x = 1429148.23;
    rx_y = 6143689.77;
    rx_z = 941611.88;
    std::cout << "Updated receiver position to: " 
              << rx_x << " X, " << rx_y << " Y, " << rx_z << " Z" << std::endl;
}

bool SatelliteDopplerCalculator::loadSP3File() {
    return loadSP3File(sp3_file_path);
}

bool SatelliteDopplerCalculator::loadSP3File(const std::string& path) {
    std::cout << "Loading SP3 file: " << path << std::endl;
    
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open SP3 file: " << path << std::endl;
        return false;
    }
    
    satellite_data.clear();
    std::string line;
    std::tm current_epoch = {};
    bool in_position_block = false;
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        // Parse epoch line: *  2024  1 15  0  0  0.00000000
        if (line[0] == '*') {
            in_position_block = true;
            current_epoch = parseEpochLine(line);
        }
        // Parse position line: PG01  12345.678  23456.789  34567.890  0.123456
        else if (line[0] == 'P' && in_position_block && line.length() > 4) {
            std::string sat_id = line.substr(1, 3);  // e.g., "G01"
            
            SatelliteState state;
            state.epoch_time = current_epoch;
            
            // Parse position (in kilometers, convert to meters)
            std::istringstream iss(line.substr(4));
            iss >> state.x >> state.y >> state.z;
            state.x *= 1000.0;  // km to m
            state.y *= 1000.0;
            state.z *= 1000.0;
            
            state.valid = true;
            satellite_data[sat_id].push_back(state);
        }
        // Parse velocity line: VG01  0.123  0.456  0.789  0.001
        else if (line[0] == 'V' && in_position_block && line.length() > 4) {
            std::string sat_id = line.substr(1, 3);
            
            if (!satellite_data[sat_id].empty()) {
                auto& last_state = satellite_data[sat_id].back();
                
                // Parse velocity (in dm/s, convert to m/s)
                std::istringstream iss(line.substr(4));
                iss >> last_state.vx >> last_state.vy >> last_state.vz;
                last_state.vx /= 10.0;  // dm/s to m/s
                last_state.vy /= 10.0;
                last_state.vz /= 10.0;
            }
        }
    }
    
    file.close();
    
    std::cout << "Loaded data for " << satellite_data.size() << " satellites" << std::endl;
    // for (const auto& sat : satellite_data) {
    //     std::cout << "  " << sat.first << ": " << sat.second.size() << " epochs" << std::endl;
    // }
    
    // Calculate velocities if not provided in SP3
    calculateVelocities();
    
    return !satellite_data.empty();
}

double SatelliteDopplerCalculator::getDopplerShift(const std::string& sat_id, 
                                                   const std::tm& utc_time) {
    // Normalize satellite ID
    std::string normalized_id = normalizeSatelliteID(sat_id);
    
    // Get satellite state at given time
    SatelliteState state = interpolateSatelliteState(normalized_id, utc_time);
    
    if (!state.valid) {
        // std::cerr << "ERROR: No valid state for satellite " << normalized_id 
        //           << " at given time" << std::endl;
        return 0.0;
    }
    
    // Calculate range vector (satellite to receiver)
    double dx = state.x - rx_x;
    double dy = state.y - rx_y;
    double dz = state.z - rx_z;
    
    double range = std::sqrt(dx*dx + dy*dy + dz*dz);
    //std::cout <<"Range is: " <<range<<std::endl;
    // Unit vector from receiver to satellite
    double ux = dx / range;
    double uy = dy / range;
    double uz = dz / range;
    
    double travel_time = range/SPEED_OF_LIGHT;
    // Calculate relative velocity
    // Account for Earth rotation effect on receiver

    double rotation_angle = EARTH_ROTATION_RATE * travel_time;
    
    // Calculate sin and cos once
    double cos_angle = std::cos(rotation_angle);
    double sin_angle = std::sin(rotation_angle);
    
    // Rotation matrix for Earth rotation correction (Z-axis rotation)
    // [cos_angle,  sin_angle, 0]
    // [-sin_angle, cos_angle, 0]
    // [0,          0,         1]
    
    // Apply rotation matrix to satellite position

    
    // Apply rotation matrix to satellite velocity
    double sat_x = cos_angle * state.x + sin_angle * state.y;
    double sat_y = -sin_angle * state.x + cos_angle * state.y;

    dx = sat_x - rx_x;
    dy = sat_y - rx_y;

    range = std::sqrt(dx*dx + dy*dy + dz*dz);
    //std::cout <<"Range is: " <<range<<std::endl;
    // Unit vector from receiver to satellite
    ux = dx / range;
    uy = dy / range;
    uz = dz / range;

    double sat_vx = cos_angle*state.vx+sin_angle*state.vy;
    double sat_vy = -sin_angle*state.vx +cos_angle*state.vy;

    // Radial velocity (projection onto line of sight)
    double radial_velocity = sat_vx* ux + sat_vy * uy + state.vz * uz;
    
    // Doppler shift: f_d = -f_0 * (v_r / c)
    double doppler = -GPS_L1_FREQ * (radial_velocity / SPEED_OF_LIGHT);
    
    return doppler;
}

void SatelliteDopplerCalculator::printDopplerInfo(const std::string& sat_id, 
                                                  const std::tm& utc_time) {
    std::string normalized_id = normalizeSatelliteID(sat_id);
    SatelliteState state = interpolateSatelliteState(normalized_id, utc_time);
    
    if (!state.valid) {
        std::cerr << "ERROR: No valid state for satellite " << normalized_id << std::endl;
        return;
    }
    
    double doppler = getDopplerShift(sat_id, utc_time);
    double range = calculateRange(state.x, state.y, state.z);
    double elevation = calculateElevation(state.x, state.y, state.z);
    double azimuth = calculateAzimuth(state.x, state.y, state.z);
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "DOPPLER CALCULATION FOR SATELLITE " << normalized_id << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    std::cout << "\nTime: " << std::put_time(&utc_time, "%Y-%m-%d %H:%M:%S UTC") << std::endl;
    
    std::cout << "\nSatellite Position (ECEF):" << std::endl;
    std::cout << "  X: " << std::fixed << std::setprecision(3) << state.x << " m" << std::endl;
    std::cout << "  Y: " << state.y << " m" << std::endl;
    std::cout << "  Z: " << state.z << " m" << std::endl;
    
    std::cout << "\nSatellite Velocity (ECEF):" << std::endl;
    std::cout << "  Vx: " << state.vx << " m/s" << std::endl;
    std::cout << "  Vy: " << state.vy << " m/s" << std::endl;
    std::cout << "  Vz: " << state.vz << " m/s" << std::endl;
    
    std::cout << "\nGeometry:" << std::endl;
    std::cout << "  Range: " << range / 1000.0 << " km" << std::endl;
    std::cout << "  Elevation: " << elevation << "°" << std::endl;
    std::cout << "  Azimuth: " << azimuth << "°" << std::endl;
    
    std::cout << "\n*** DOPPLER SHIFT: " << std::setprecision(2) << doppler << " Hz ***" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
}

bool SatelliteDopplerCalculator::getSatellitePosition(const std::string& sat_id,
                                                      const std::tm& utc_time,
                                                      double& x, double& y, double& z) {
    SatelliteState state = interpolateSatelliteState(normalizeSatelliteID(sat_id), utc_time);
    if (state.valid) {
        x = state.x;
        y = state.y;
        z = state.z;
        return true;
    }
    return false;
}

bool SatelliteDopplerCalculator::getSatelliteVelocity(const std::string& sat_id,
                                                      const std::tm& utc_time,
                                                      double& vx, double& vy, double& vz) {
    SatelliteState state = interpolateSatelliteState(normalizeSatelliteID(sat_id), utc_time);
    if (state.valid) {
        vx = state.vx;
        vy = state.vy;
        vz = state.vz;
        return true;
    }
    return false;
}

double SatelliteDopplerCalculator::getElevation(const std::string& sat_id, 
                                                const std::tm& utc_time) {
    SatelliteState state = interpolateSatelliteState(normalizeSatelliteID(sat_id), utc_time);
    if (state.valid) {
        return calculateElevation(state.x, state.y, state.z);
    }
    return -999.0;
}

double SatelliteDopplerCalculator::getAzimuth(const std::string& sat_id, 
                                              const std::tm& utc_time) {
    SatelliteState state = interpolateSatelliteState(normalizeSatelliteID(sat_id), utc_time);
    if (state.valid) {
        return calculateAzimuth(state.x, state.y, state.z);
    }
    return -999.0;
}

double SatelliteDopplerCalculator::getRange(const std::string& sat_id, 
                                            const std::tm& utc_time) {
    SatelliteState state = interpolateSatelliteState(normalizeSatelliteID(sat_id), utc_time);
    if (state.valid) {
        return calculateRange(state.x, state.y, state.z);
    }
    return -1.0;
}

bool SatelliteDopplerCalculator::hasSatelliteData(const std::string& sat_id) const {
    std::string normalized_id = normalizeSatelliteID(sat_id);
    return satellite_data.find(normalized_id) != satellite_data.end();
}

std::vector<std::string> SatelliteDopplerCalculator::getAvailableSatellites() const {
    std::vector<std::string> satellites;
    for (const auto& sat : satellite_data) {
        satellites.push_back(sat.first);
    }
    return satellites;
}

int SatelliteDopplerCalculator::getEpochCount(const std::string& sat_id) const {
    std::string normalized_id = normalizeSatelliteID(sat_id);
    auto it = satellite_data.find(normalized_id);
    if (it != satellite_data.end()) {
        return static_cast<int>(it->second.size());
    }
    return 0;
}

// ============================================================================
// PRIVATE METHODS
// ============================================================================

void SatelliteDopplerCalculator::convertLLAtoECEF(double lat_deg, double lon_deg, double alt,
                                                  double& x, double& y, double& z) {
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
    
    x = (N + alt) * std::cos(lat) * std::cos(lon);
    y = (N + alt) * std::cos(lat) * std::sin(lon);
    z = (N * (1.0 - WGS84_E2) + alt) * std::sin(lat);
}

std::tm SatelliteDopplerCalculator::parseEpochLine(const std::string& line) {
    std::tm epoch = {};
    
    // Format: *  2024  1 15  0  0  0.00000000
    std::istringstream iss(line.substr(3));
    int year, month, day, hour, minute;
    double second;
    
    iss >> year >> month >> day >> hour >> minute >> second;
    
    epoch.tm_year = year - 1900;
    epoch.tm_mon = month - 1;
    epoch.tm_mday = day;
    epoch.tm_hour = hour;
    epoch.tm_min = minute;
    epoch.tm_sec = static_cast<int>(second);
    
    return epoch;
}

std::string SatelliteDopplerCalculator::normalizeSatelliteID(const std::string& sat_id) const {
    if (sat_id.empty()) return "G01";
    
    std::string id = sat_id;
    std::transform(id.begin(), id.end(), id.begin(), ::toupper);
    
    // If just a number, assume GPS
    if (std::isdigit(id[0])) {
        int prn = std::stoi(id);
        char buffer[4];
        std::snprintf(buffer, sizeof(buffer), "G%02d", prn);
        return std::string(buffer);
    }
    
    // If format is "G1", convert to "G01"
    if (id.length() == 2 && std::isalpha(id[0]) && std::isdigit(id[1])) {
        char buffer[4];
        std::snprintf(buffer, sizeof(buffer), "%c%02d", id[0], id[1] - '0');
        return std::string(buffer);
    }
    
    return id;
}

void SatelliteDopplerCalculator::calculateVelocities() {
    for (auto& sat_pair : satellite_data) {
        auto& states = sat_pair.second;
        
        if (states.size() < 2) continue;
        
        // Check if velocities are already present
        bool has_velocity = false;
        for (const auto& state : states) {
            if (state.vx != 0.0 || state.vy != 0.0 || state.vz != 0.0) {
                has_velocity = true;
                break;
            }
        }
        
        if (has_velocity) continue;
        
        // Calculate velocities using finite differences
        for (size_t i = 1; i < states.size() - 1; ++i) {
            double dt1 = timeDifference(states[i-1].epoch_time, states[i].epoch_time);
            double dt2 = timeDifference(states[i].epoch_time, states[i+1].epoch_time);
            
            if (dt1 > 0 && dt2 > 0) {
                states[i].vx = (states[i+1].x - states[i-1].x) / (dt1 + dt2);
                states[i].vy = (states[i+1].y - states[i-1].y) / (dt1 + dt2);
                states[i].vz = (states[i+1].z - states[i-1].z) / (dt1 + dt2);
            }
        }
        
        // Handle endpoints
        if (states.size() >= 2) {
            double dt = timeDifference(states[0].epoch_time, states[1].epoch_time);
            if (dt > 0) {
                states[0].vx = (states[1].x - states[0].x) / dt;
                states[0].vy = (states[1].y - states[0].y) / dt;
                states[0].vz = (states[1].z - states[0].z) / dt;
                
                size_t last = states.size() - 1;
                states[last].vx = (states[last].x - states[last-1].x) / dt;
                states[last].vy = (states[last].y - states[last-1].y) / dt;
                states[last].vz = (states[last].z - states[last-1].z) / dt;
            }
        }
    }
}

double SatelliteDopplerCalculator::timeDifference(const std::tm& t1, const std::tm& t2) const {
    std::tm temp1 = t1;
    std::tm temp2 = t2;
    std::time_t time1 = std::mktime(&temp1);
    std::time_t time2 = std::mktime(&temp2);
    return std::difftime(time2, time1);
}

SatelliteDopplerCalculator::SatelliteState 

// SatelliteDopplerCalculator::interpolateSatelliteState(const std::string& sat_id, 
//                                                       const std::tm& target_time) {
//     SatelliteState result;
    
//     auto it = satellite_data.find(sat_id);
//     if (it == satellite_data.end() || it->second.empty()) {
//         return result;  // Invalid
//     }
    
//     const auto& states = it->second;
    
//     // Find closest epoch
//     int closest_idx = -1;
//     double min_diff = 1e10;
    
//     for (size_t i = 0; i < states.size(); ++i) {
//         double diff = std::abs(timeDifference(states[i].epoch_time, target_time));
//         if (diff < min_diff) {
//             min_diff = diff;
//             closest_idx = i;
//         }
//     }
    
//     if (closest_idx < 0) return result;
    
//     // If very close to an epoch (within 0.1 second), use it directly
//     if (min_diff < 0.1) {
//         return states[closest_idx];
//     }
    
//     // Degree 10 Lagrange polynomial interpolation
//     const int POLY_DEGREE = 10;
//     const int N_POINTS = POLY_DEGREE + 1;  // 11 points for degree 10
    
//     // Check if we have enough points
//     if (states.size() < N_POINTS) {
//         // Fall back to linear interpolation if not enough points
//         if (closest_idx > 0 && closest_idx < static_cast<int>(states.size()) - 1) {
//             size_t idx1 = closest_idx - 1;
//             size_t idx2 = closest_idx;
            
//             double dt_total = timeDifference(states[idx1].epoch_time, states[idx2].epoch_time);
//             double dt_target = timeDifference(states[idx1].epoch_time, target_time);
            
//             if (dt_total > 0) {
//                 double factor = dt_target / dt_total;
//                 result.x = states[idx1].x + factor * (states[idx2].x - states[idx1].x);
//                 result.y = states[idx1].y + factor * (states[idx2].y - states[idx1].y);
//                 result.z = states[idx1].z + factor * (states[idx2].z - states[idx1].z);
//                 result.vx = states[idx1].vx + factor * (states[idx2].vx - states[idx1].vx);
//                 result.vy = states[idx1].vy + factor * (states[idx2].vy - states[idx1].vy);
//                 result.vz = states[idx1].vz + factor * (states[idx2].vz - states[idx1].vz);
//                 result.valid = true;
//             }
//         } else {
//             result = states[closest_idx];
//         }
//         result.epoch_time = target_time;
//         return result;
//     }
    
//     // Determine the range of points for interpolation
//     // Center around the closest epoch
//     //std::cout <<"Reached Polynomial ineterpolation"<<std::endl;
//     int start_idx = closest_idx - N_POINTS / 2;
    
//     // Adjust boundaries
//     if (start_idx < 0) {
//         start_idx = 0;
//     } else if (start_idx + N_POINTS > static_cast<int>(states.size())) {
//         start_idx = states.size() - N_POINTS;
//     }
    
//     // Collect interpolation points
//     std::vector<double> t_points;
//     std::vector<double> x_points, y_points, z_points;
//     std::vector<double> vx_points, vy_points, vz_points;
    
//     for (int i = 0; i < N_POINTS; ++i) {
//         int idx = start_idx + i;
//         t_points.push_back(timeDifference(states[0].epoch_time, states[idx].epoch_time));
//         x_points.push_back(states[idx].x);
//         y_points.push_back(states[idx].y);
//         z_points.push_back(states[idx].z);
//         vx_points.push_back(states[idx].vx);
//         vy_points.push_back(states[idx].vy);
//         vz_points.push_back(states[idx].vz);
//     }
    
//     // Target time in seconds from reference
//     double t_target = timeDifference(states[0].epoch_time, target_time);
    
//     // Lagrange polynomial interpolation
//     result.x = lagrangeInterpolation(t_points, x_points, t_target);
//     result.y = lagrangeInterpolation(t_points, y_points, t_target);
//     result.z = lagrangeInterpolation(t_points, z_points, t_target);
//     result.vx = lagrangeInterpolation(t_points, vx_points, t_target);
//     result.vy = lagrangeInterpolation(t_points, vy_points, t_target);
//     result.vz = lagrangeInterpolation(t_points, vz_points, t_target);
//     result.valid = true;
//     result.epoch_time = target_time;
    
//     return result;
// }


SatelliteDopplerCalculator::interpolateSatelliteState(const std::string& sat_id, 
                                                      const std::tm& target_time) {
    SatelliteState result;
    
    auto it = satellite_data.find(sat_id);
    if (it == satellite_data.end() || it->second.empty()) {
        return result;  // Invalid
    }
    
    const auto& states = it->second;
    
    // Find closest epochs
    int closest_idx = -1;
    double min_diff = 1e10;
    
    for (size_t i = 0; i < states.size(); ++i) {
        double diff = std::abs(timeDifference(states[i].epoch_time, target_time));
        if (diff < min_diff) {
            min_diff = diff;
            closest_idx = i;
        }
    }
    
    if (closest_idx < 0) return result;
    // If very close to an epoch (within 1 second), use it directly
    if (min_diff < 1.0) {
        return states[closest_idx];
    }
    
    // Linear interpolation
    if (closest_idx > 0 && closest_idx < static_cast<int>(states.size()) - 1) {
        double diff_prev = timeDifference(states[closest_idx-1].epoch_time, target_time);
        
        size_t idx1, idx2;
        if (diff_prev < 0) {
            idx1 = closest_idx - 1;
            idx2 = closest_idx;
        } else {
            idx1 = closest_idx;
            idx2 = closest_idx + 1;
        }
        
        double dt_total = timeDifference(states[idx1].epoch_time, states[idx2].epoch_time);
        double dt_target = timeDifference(states[idx1].epoch_time, target_time);
        
        if (dt_total > 0) {
            double factor = dt_target / dt_total;
            
            result.x = states[idx1].x + factor * (states[idx2].x - states[idx1].x);
            result.y = states[idx1].y + factor * (states[idx2].y - states[idx1].y);
            result.z = states[idx1].z + factor * (states[idx2].z - states[idx1].z);
            result.vx = states[idx1].vx + factor * (states[idx2].vx - states[idx1].vx);
            result.vy = states[idx1].vy + factor * (states[idx2].vy - states[idx1].vy);
            result.vz = states[idx1].vz + factor * (states[idx2].vz - states[idx1].vz);
            result.valid = true;
        }
    } else {
        result = states[closest_idx];
    }
    
    result.epoch_time = target_time;
    return result;
}

double SatelliteDopplerCalculator::calculateElevation(double sat_x, double sat_y, double sat_z) const {
    double dx = sat_x - rx_x;
    double dy = sat_y - rx_y;
    double dz = sat_z - rx_z;
    
    double lat = receiver_lat * M_PI / 180.0;
    double lon = receiver_lon * M_PI / 180.0;
    
    double e = -std::sin(lon) * dx + std::cos(lon) * dy;
    double n = -std::sin(lat) * std::cos(lon) * dx 
               - std::sin(lat) * std::sin(lon) * dy 
               + std::cos(lat) * dz;
    double u = std::cos(lat) * std::cos(lon) * dx 
               + std::cos(lat) * std::sin(lon) * dy 
               + std::sin(lat) * dz;
    
    double elevation = std::atan2(u, std::sqrt(e*e + n*n)) * 180.0 / M_PI;
    return elevation;
}

double SatelliteDopplerCalculator::calculateAzimuth(double sat_x, double sat_y, double sat_z) const {
    double dx = sat_x - rx_x;
    double dy = sat_y - rx_y;
    double dz = sat_z - rx_z;
    
    double lat = receiver_lat * M_PI / 180.0;
    double lon = receiver_lon * M_PI / 180.0;
    
    double e = -std::sin(lon) * dx + std::cos(lon) * dy;
    double n = -std::sin(lat) * std::cos(lon) * dx 
               - std::sin(lat) * std::sin(lon) * dy 
               + std::cos(lat) * dz;
    
    double azimuth = std::atan2(e, n) * 180.0 / M_PI;
    if (azimuth < 0) azimuth += 360.0;
    
    return azimuth;
}

double SatelliteDopplerCalculator::calculateRange(double sat_x, double sat_y, double sat_z) const {
    double dx = sat_x - rx_x;
    double dy = sat_y - rx_y;
    double dz = sat_z - rx_z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double SatelliteDopplerCalculator::lagrangeInterpolation(const std::vector<double>& x,
                                                         const std::vector<double>& y,
                                                         double x_target) const {
    if ((x.size() != y.size()) || x.empty()) {
        return 0.0;
    }
    
    int n = x.size();
    double result = 0.0;
    
    // Lagrange polynomial: L(x) = sum_{i=0}^{n-1} y_i * l_i(x)
    // where l_i(x) = product_{j=0,j!=i}^{n-1} (x - x_j) / (x_i - x_j)
    
    for (int i = 0; i < n; ++i) {
        double term = y[i];
        
        for (int j = 0; j < n; ++j) {
            if (i != j) {
                double denominator = x[i] - x[j];
                if (std::abs(denominator) < 1e-10) {
                    // Avoid division by zero
                    continue;
                }
                term *= (x_target - x[j]) / denominator;
            }
        }
        
        result += term;
    }
    return result;
}    
    