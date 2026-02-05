// ============================================================================
// FILE: satellite_doppler_calculator.h
// DESCRIPTION: Satellite Doppler Calculator from SP3 Orbit Files
// ============================================================================

#ifndef SATELLITE_DOPPLER_CALCULATOR_H
#define SATELLITE_DOPPLER_CALCULATOR_H

#include <string>
#include <vector>
#include <map>
#include <ctime>

class SatelliteDopplerCalculator {
private:
    // Constants
    static constexpr double SPEED_OF_LIGHT = 299792458.0;      // m/s
    static constexpr double GPS_L1_FREQ = 1575.42e6;           // Hz
    static constexpr double EARTH_RADIUS = 6371000.0;          // meters
    static constexpr double EARTH_ROTATION_RATE = 7.2921151467e-5;  // rad/s
    static constexpr double WGS84_A = 6378137.0;               // WGS84 semi-major axis
    static constexpr double WGS84_E2 = 0.00669437999014;       // WGS84 eccentricity squared
    
    // SP3 file path
    std::string sp3_file_path;
    
    // Receiver position
    double receiver_lat;  // degrees
    double receiver_lon;  // degrees
    double receiver_alt;  // meters
    
    // Receiver position in ECEF
    double rx_x, rx_y, rx_z;
    
    // Structure to hold satellite position and velocity
    struct SatelliteState {
        double x, y, z;        // Position in ECEF (meters)
        double vx, vy, vz;     // Velocity in ECEF (m/s)
        std::tm epoch_time;
        bool valid;
        
        SatelliteState();
    };
    
    // Storage for parsed SP3 data
    std::map<std::string, std::vector<SatelliteState>> satellite_data;

public:
    // Constructor
    SatelliteDopplerCalculator();
    
    // Constructor with parameters
    SatelliteDopplerCalculator(const std::string& sp3_path,
                              double lat, double lon, double alt);
    
    // Destructor
    ~SatelliteDopplerCalculator();
    
    // Set SP3 file path
    void setSP3FilePath(const std::string& path);
    
    // Set receiver position (latitude, longitude in degrees, altitude in meters)
    void setReceiverPosition(double lat, double lon, double alt);
    
    // Load SP3 file
    bool loadSP3File();
    
    // Load SP3 file with specific path
    bool loadSP3File(const std::string& path);
    
    // Get Doppler shift for a satellite at given time
    double getDopplerShift(const std::string& sat_id, const std::tm& utc_time);
    
    // Print detailed Doppler information
    void printDopplerInfo(const std::string& sat_id, const std::tm& utc_time);
    
    // Get satellite position at given time
    bool getSatellitePosition(const std::string& sat_id, const std::tm& utc_time,
                             double& x, double& y, double& z);
    
    // Get satellite velocity at given time
    bool getSatelliteVelocity(const std::string& sat_id, const std::tm& utc_time,
                             double& vx, double& vy, double& vz);
    
    // Get elevation angle (degrees)
    double getElevation(const std::string& sat_id, const std::tm& utc_time);
    
    // Get azimuth angle (degrees)
    double getAzimuth(const std::string& sat_id, const std::tm& utc_time);
    
    // Get range to satellite (meters)
    double getRange(const std::string& sat_id, const std::tm& utc_time);
    
    // Check if satellite data is available
    bool hasSatelliteData(const std::string& sat_id) const;
    
    // Get list of available satellites
    std::vector<std::string> getAvailableSatellites() const;
    
    // Get number of epochs for a satellite
    int getEpochCount(const std::string& sat_id) const;

private:
    // Convert LLA to ECEF
    void convertLLAtoECEF(double lat_deg, double lon_deg, double alt,
                          double& x, double& y, double& z);
    
    // Parse epoch line from SP3
    std::tm parseEpochLine(const std::string& line);
    
    // Normalize satellite ID (e.g., "1" -> "G01", "G1" -> "G01")
    std::string normalizeSatelliteID(const std::string& sat_id) const;
    
    // Calculate velocities from positions if not in SP3
    void calculateVelocities();
    
    // Calculate time difference in seconds
    double timeDifference(const std::tm& t1, const std::tm& t2) const;
    
    // Interpolate satellite state at given time
    SatelliteState interpolateSatelliteState(const std::string& sat_id, 
                                            const std::tm& target_time);
    
    // Calculate elevation angle
    double calculateElevation(double sat_x, double sat_y, double sat_z) const;
    
    // Calculate azimuth angle
    double calculateAzimuth(double sat_x, double sat_y, double sat_z) const;
    
    // Calculate range
    double calculateRange(double sat_x, double sat_y, double sat_z) const;

    double lagrangeInterpolation(const std::vector<double>& x, 
                                const std::vector<double>& y, 
                                double x_target) const;
};

#endif // SATELLITE_DOPPLER_CALCULATOR_H