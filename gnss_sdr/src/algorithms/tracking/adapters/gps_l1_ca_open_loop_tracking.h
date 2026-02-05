/*!
 * \file gps_l1_ca_open_loop_tracking.h (Enhanced version)
 * \brief Enhanced adapter with external control methods
 */

#ifndef GNSS_SDR_GPS_L1_CA_OPEN_LOOP_TRACKING_H
#define GNSS_SDR_GPS_L1_CA_OPEN_LOOP_TRACKING_H

#include "tracking_interface.h"
#include "gps_l1_ca_open_loop_tracking_cc.h"
#include <string>

class ConfigurationInterface;

class GpsL1CaOpenLoopTracking : public TrackingInterface
{
public:
    GpsL1CaOpenLoopTracking(ConfigurationInterface* configuration,
                           const std::string& role,
                           unsigned int in_streams,
                           unsigned int out_streams);

    virtual ~GpsL1CaOpenLoopTracking();

    inline std::string role() override { return role_; }
    inline std::string implementation() override { return "GPS_L1_CA_Open_Loop_Tracking"; }
    inline size_t item_size() override { return item_size_; }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void set_channel(unsigned int channel) override;
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;
    void start_tracking() override;
    void stop_tracking() override;

    // ========== OPEN LOOP SPECIFIC METHODS ==========
    
    /*!
     * \brief Set carrier Doppler frequency externally (Hz)
     * This allows external control of the tracking frequency
     * Useful for:
     * - Using predicted Doppler from navigation solution
     * - Testing specific frequency profiles
     * - Compensating for known platform dynamics
     */
    void set_carrier_doppler_hz(double doppler_hz)
    {
        if (tracking_)
        {
            tracking_->set_carrier_doppler_hz(doppler_hz);
        }
    }
    
    /*!
     * \brief Set code frequency externally (chips/sec)
     * Normally derived from carrier Doppler, but can be set independently
     */
    void set_code_freq_chips(double code_freq_chips)
    {
        if (tracking_)
        {
            tracking_->set_code_freq_chips(code_freq_chips);
        }
    }
    
    /*!
     * \brief Update Doppler from a predicted trajectory
     * \param time_sec Time in seconds
     * \param doppler_hz Predicted Doppler at this time
     */
    void update_from_prediction(double time_sec, double doppler_hz)
    {
        last_update_time_ = time_sec;
        set_carrier_doppler_hz(doppler_hz);
    }
    
    /*!
     * \brief Get the currently set Doppler (for monitoring)
     */
    double get_current_doppler_hz() const
    {
        return current_doppler_hz_;
    }

private:
    gps_l1_ca_open_loop_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    
    std::string dump_filename_;
    bool dump_;
    float early_late_space_chips_;
    int vector_length_;
    int fs_in_;
    
    // For tracking current state
    double current_doppler_hz_;
    double last_update_time_;
};

#endif

// ============================================================================
// Example usage scenarios:
// ============================================================================

/*
SCENARIO 1: Static Doppler (use acquisition value throughout)
-------------------------------------------------------------
This is the default behavior - just start tracking and it uses
the acquisition Doppler without any updates.

tracker.start_tracking();


SCENARIO 2: Manually update Doppler at specific intervals
----------------------------------------------------------
Useful for testing or when you have external Doppler information

tracker.start_tracking();

// Later, update Doppler (e.g., every 100ms)
tracker.set_carrier_doppler_hz(1250.0);  // Update to new value


SCENARIO 3: Use predicted Doppler from navigation solution
-----------------------------------------------------------
If you have a navigation solution with platform velocity, you can
predict Doppler and update the tracker

// Get position and velocity from navigation solution
double user_velocity_x, user_velocity_y, user_velocity_z;
double sat_pos_x, sat_pos_y, sat_pos_z;
double sat_vel_x, sat_vel_y, sat_vel_z;

// Calculate line-of-sight velocity
double los_velocity = compute_los_velocity(user_velocity, sat_velocity, geometry);

// Calculate predicted Doppler
double predicted_doppler = -(los_velocity / SPEED_OF_LIGHT) * GPS_L1_FREQ_HZ;

// Update tracker
tracker.set_carrier_doppler_hz(predicted_doppler);


SCENARIO 4: Doppler profile for high dynamics
----------------------------------------------
For aircraft, rockets, or other high-dynamic platforms, you might
have a pre-computed Doppler profile

void update_doppler_from_profile(GpsL1CaOpenLoopTracking& tracker,
                                 double current_time,
                                 const std::vector<std::pair<double, double>>& profile)
{
    // Interpolate Doppler from profile
    double doppler = interpolate_profile(profile, current_time);
    tracker.update_from_prediction(current_time, doppler);
}


SCENARIO 5: Testing specific scenarios
---------------------------------------
Test how the receiver performs with known Doppler errors

// Simulate oscillator drift
tracker.start_tracking();
double base_doppler = 1250.0;
double drift_rate = 0.1;  // Hz/sec

for (double t = 0; t < 10.0; t += 0.1)
{
    double doppler = base_doppler + drift_rate * t;
    tracker.set_carrier_doppler_hz(doppler);
    sleep_ms(100);
}


SCENARIO 6: Integration with Kalman filter
-------------------------------------------
Use a Kalman filter to predict Doppler and update the tracker

class DopplerPredictor
{
    KalmanFilter kf_;
    
public:
    double predict_doppler(double dt)
    {
        kf_.predict(dt);
        return kf_.get_state().doppler;
    }
    
    void update_from_measurement(double measured_doppler)
    {
        kf_.update(measured_doppler);
    }
};

DopplerPredictor predictor;
tracker.start_tracking();

// In tracking loop
double predicted_doppler = predictor.predict_doppler(0.001);  // 1ms
tracker.set_carrier_doppler_hz(predicted_doppler);
*/
