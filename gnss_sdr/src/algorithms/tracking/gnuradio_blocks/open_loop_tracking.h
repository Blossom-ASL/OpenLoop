/*!
 * \file gps_l1_ca_open_loop_tracking_cc.h
 * \brief Interface of a code and carrier open loop tracking block for GPS L1 C/A
 * \author Your Name, 2026
 */

#ifndef GNSS_SDR_GPS_L1_CA_OPEN_LOOP_TRACKING_CC_H
#define GNSS_SDR_GPS_L1_CA_OPEN_LOOP_TRACKING_CC_H

#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include <gnuradio/block.h>
#include <fstream>
#include <string>

class gps_l1_ca_open_loop_tracking_cc;

typedef boost::shared_ptr<gps_l1_ca_open_loop_tracking_cc>
    gps_l1_ca_open_loop_tracking_cc_sptr;

gps_l1_ca_open_loop_tracking_cc_sptr
gps_l1_ca_open_loop_tracking_make_cc(
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    std::string dump_filename,
    float early_late_space_chips);

class gps_l1_ca_open_loop_tracking_cc : public gr::block
{
public:
    ~gps_l1_ca_open_loop_tracking_cc();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();
    
    // Open loop specific: set external Doppler
    void set_carrier_doppler_hz(double doppler_hz);
    void set_code_freq_chips(double code_freq_chips);

    int general_work(int noutput_items, gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

private:
    friend gps_l1_ca_open_loop_tracking_cc_sptr
    gps_l1_ca_open_loop_tracking_make_cc(
        int64_t fs_in,
        uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float early_late_space_chips);

    gps_l1_ca_open_loop_tracking_cc(
        int64_t fs_in,
        uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float early_late_space_chips);

    // Tracking parameters
    int64_t d_fs_in;
    uint32_t d_vector_length;
    bool d_dump;
    
    // Correlators
    gr_complex* d_ca_code;
    gr_complex* d_early_code;
    gr_complex* d_prompt_code;
    gr_complex* d_late_code;
    
    gr_complex d_Early;
    gr_complex d_Prompt;
    gr_complex d_Late;
    
    // Open loop parameters - NO FEEDBACK
    double d_carrier_doppler_hz;      // Fixed or externally set
    double d_code_freq_chips;         // Fixed or externally set
    double d_carrier_phase_rad;
    double d_code_phase_chips;
    
    // Carrier and code NCO
    double d_carrier_phase_step_rad;
    double d_code_phase_step_chips;
    
    // Baseband signal parameters
    double d_acq_code_phase_chips;
    double d_acq_carrier_doppler_hz;
    
    // PRN code period
    double d_code_period;
    
    // Spacing between correlators
    float d_early_late_spc_chips;
    
    // Integration period
    int32_t d_correlation_length_samples;
    
    // Channel and satellite info
    uint32_t d_channel;
    Gnss_Synchro* d_acquisition_gnss_synchro;
    
    // Sample counter
    uint64_t d_sample_counter;
    
    // Control flags
    bool d_enable_tracking;
    bool d_pull_in;
    
    // Dump file
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    
    // Functions
    void update_local_carriers();
    void update_local_code();
    void correlate();
    void save_correlation_results();
};

#endif