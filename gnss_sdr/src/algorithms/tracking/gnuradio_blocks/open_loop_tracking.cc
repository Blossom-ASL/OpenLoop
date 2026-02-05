/*!
 * \file gps_l1_ca_open_loop_tracking_cc.cc
 * \brief Implementation of open loop tracking for GPS L1 C/A
 */

#include "gnss_block_interface.h"
#include "GPS_L1_CA.h"
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <fstream>
#include <map>
#include <string>
#include <cmath>
#include <iostream>

gps_l1_ca_open_loop_tracking_cc_sptr
gps_l1_ca_open_loop_tracking_make_cc(
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    std::string dump_filename,
    float early_late_space_chips)
{
    return gps_l1_ca_open_loop_tracking_cc_sptr(
        new gps_l1_ca_open_loop_tracking_cc(
            fs_in, vector_length, dump, dump_filename, early_late_space_chips));
}

gps_l1_ca_open_loop_tracking_cc::gps_l1_ca_open_loop_tracking_cc(
    int64_t fs_in,
    uint32_t vector_length,
    bool dump,
    std::string dump_filename,
    float early_late_space_chips)
    : gr::block("gps_l1_ca_open_loop_tracking_cc",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_dump = dump;
    d_dump_filename = dump_filename;
    d_early_late_spc_chips = early_late_space_chips;
    
    // Initialize correlation length
    d_correlation_length_samples = static_cast<int32_t>(d_vector_length);
    
    // Allocate memory for local codes
    d_ca_code = static_cast<gr_complex*>(
        volk_gnsssdr_malloc(GPS_L1_CA_CODE_LENGTH_CHIPS * sizeof(gr_complex),
                           volk_gnsssdr_get_alignment()));
    
    d_early_code = static_cast<gr_complex*>(
        volk_gnsssdr_malloc(d_vector_length * sizeof(gr_complex),
                           volk_gnsssdr_get_alignment()));
    
    d_prompt_code = static_cast<gr_complex*>(
        volk_gnsssdr_malloc(d_vector_length * sizeof(gr_complex),
                           volk_gnsssdr_get_alignment()));
    
    d_late_code = static_cast<gr_complex*>(
        volk_gnsssdr_malloc(d_vector_length * sizeof(gr_complex),
                           volk_gnsssdr_get_alignment()));
    
    // Initialize code period (1 ms for GPS L1 C/A)
    d_code_period = 0.001;
    
    // Initialize tracking state
    d_enable_tracking = false;
    d_pull_in = false;
    
    // Initialize phase accumulators
    d_carrier_phase_rad = 0.0;
    d_code_phase_chips = 0.0;
    
    // Initialize open loop frequencies (will be set externally or from acquisition)
    d_carrier_doppler_hz = 0.0;
    d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ;
    
    d_sample_counter = 0;
    
    std::cout << "Open Loop Tracker initialized - NO FEEDBACK LOOPS" << std::endl;
}

gps_l1_ca_open_loop_tracking_cc::~gps_l1_ca_open_loop_tracking_cc()
{
    volk_gnsssdr_free(d_ca_code);
    volk_gnsssdr_free(d_early_code);
    volk_gnsssdr_free(d_prompt_code);
    volk_gnsssdr_free(d_late_code);
    
    if (d_dump_file.is_open())
    {
        d_dump_file.close();
    }
}

void gps_l1_ca_open_loop_tracking_cc::set_channel(uint32_t channel)
{
    d_channel = channel;
}

void gps_l1_ca_open_loop_tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}

void gps_l1_ca_open_loop_tracking_cc::start_tracking()
{
    // Get initial parameters from acquisition
    d_acq_code_phase_chips = d_acquisition_gnss_synchro->Acq_delay_samples * 
                             GPS_L1_CA_CODE_RATE_HZ / static_cast<double>(d_fs_in);
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    
    // Initialize with acquisition values (OPEN LOOP - these won't be updated by tracking)
    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ + 
                        (d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_HZ / GPS_L1_FREQ_HZ);
    
    d_carrier_phase_rad = 0.0;
    d_code_phase_chips = d_acq_code_phase_chips;
    
    // Calculate phase step rates (FIXED - no loop updates)
    d_carrier_phase_step_rad = (2.0 * M_PI * d_carrier_doppler_hz) / static_cast<double>(d_fs_in);
    d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
    
    // Generate local C/A code
    gps_l1_ca_code_gen_complex(d_ca_code, d_acquisition_gnss_synchro->PRN, 0);
    
    d_enable_tracking = true;
    d_pull_in = true;
    
    std::cout << "Open Loop Tracking started for PRN " << d_acquisition_gnss_synchro->PRN 
              << " with fixed Doppler: " << d_carrier_doppler_hz << " Hz" << std::endl;
    
    if (d_dump)
    {
        d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
    }
}

void gps_l1_ca_open_loop_tracking_cc::set_carrier_doppler_hz(double doppler_hz)
{
    // Allow external setting of Doppler (e.g., from navigation solution or prediction)
    d_carrier_doppler_hz = doppler_hz;
    d_carrier_phase_step_rad = (2.0 * M_PI * d_carrier_doppler_hz) / static_cast<double>(d_fs_in);
    
    // Update code frequency accordingly
    d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ + 
                        (d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_HZ / GPS_L1_FREQ_HZ);
    d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
}

void gps_l1_ca_open_loop_tracking_cc::set_code_freq_chips(double code_freq_chips)
{
    // Allow external setting of code frequency
    d_code_freq_chips = code_freq_chips;
    d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
}

void gps_l1_ca_open_loop_tracking_cc::update_local_code()
{
    // Generate early, prompt, and late codes based on current code phase
    double code_phase_early = d_code_phase_chips - d_early_late_spc_chips;
    double code_phase_late = d_code_phase_chips + d_early_late_spc_chips;
    
    for (int32_t i = 0; i < d_correlation_length_samples; i++)
    {
        // Early code
        int32_t early_idx = static_cast<int32_t>(code_phase_early) % GPS_L1_CA_CODE_LENGTH_CHIPS;
        d_early_code[i] = d_ca_code[early_idx];
        code_phase_early += d_code_phase_step_chips;
        
        // Prompt code
        int32_t prompt_idx = static_cast<int32_t>(d_code_phase_chips) % GPS_L1_CA_CODE_LENGTH_CHIPS;
        d_prompt_code[i] = d_ca_code[prompt_idx];
        d_code_phase_chips += d_code_phase_step_chips;
        
        // Late code
        int32_t late_idx = static_cast<int32_t>(code_phase_late) % GPS_L1_CA_CODE_LENGTH_CHIPS;
        d_late_code[i] = d_ca_code[late_idx];
        code_phase_late += d_code_phase_step_chips;
        
        // Wrap code phase
        if (d_code_phase_chips >= GPS_L1_CA_CODE_LENGTH_CHIPS)
        {
            d_code_phase_chips -= GPS_L1_CA_CODE_LENGTH_CHIPS;
            code_phase_early -= GPS_L1_CA_CODE_LENGTH_CHIPS;
            code_phase_late -= GPS_L1_CA_CODE_LENGTH_CHIPS;
        }
    }
}

void gps_l1_ca_open_loop_tracking_cc::correlate()
{
    // Perform correlation (this remains the same as closed loop)
    const gr_complex* in = static_cast<const gr_complex*>(input_items[0]);
    
    // Correlate with early, prompt, and late codes
    volk_gnsssdr_32fc_x2_dot_prod_32fc(&d_Early, in, d_early_code, d_correlation_length_samples);
    volk_gnsssdr_32fc_x2_dot_prod_32fc(&d_Prompt, in, d_prompt_code, d_correlation_length_samples);
    volk_gnsssdr_32fc_x2_dot_prod_32fc(&d_Late, in, d_late_code, d_correlation_length_samples);
}

int gps_l1_ca_open_loop_tracking_cc::general_work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    if (!d_enable_tracking)
    {
        consume_each(d_correlation_length_samples);
        return 0;
    }
    
    const gr_complex* in = static_cast<const gr_complex*>(input_items[0]);
    Gnss_Synchro* out = static_cast<Gnss_Synchro*>(output_items[0]);
    
    // Update local code
    update_local_code();
    
    // Carrier wipeoff (using FIXED Doppler - no PLL correction)
    gr_complex* carr_wipeoff = static_cast<gr_complex*>(
        volk_gnsssdr_malloc(d_correlation_length_samples * sizeof(gr_complex),
                           volk_gnsssdr_get_alignment()));
    
    for (int32_t i = 0; i < d_correlation_length_samples; i++)
    {
        carr_wipeoff[i] = gr_complex(std::cos(d_carrier_phase_rad), -std::sin(d_carrier_phase_rad));
        d_carrier_phase_rad += d_carrier_phase_step_rad;
        
        // Wrap phase
        if (d_carrier_phase_rad > 2.0 * M_PI)
            d_carrier_phase_rad -= 2.0 * M_PI;
    }
    
    // Apply carrier wipeoff
    gr_complex* signal_after_wipeoff = static_cast<gr_complex*>(
        volk_gnsssdr_malloc(d_correlation_length_samples * sizeof(gr_complex),
                           volk_gnsssdr_get_alignment()));
    
    volk_gnsssdr_32fc_x2_multiply_32fc(signal_after_wipeoff, in, carr_wipeoff, 
                                       d_correlation_length_samples);
    
    // Correlate
    volk_gnsssdr_32fc_x2_dot_prod_32fc(&d_Early, signal_after_wipeoff, d_early_code, 
                                       d_correlation_length_samples);
    volk_gnsssdr_32fc_x2_dot_prod_32fc(&d_Prompt, signal_after_wipeoff, d_prompt_code, 
                                       d_correlation_length_samples);
    volk_gnsssdr_32fc_x2_dot_prod_32fc(&d_Late, signal_after_wipeoff, d_late_code, 
                                       d_correlation_length_samples);
    
    volk_gnsssdr_free(carr_wipeoff);
    volk_gnsssdr_free(signal_after_wipeoff);
    
    // NO TRACKING LOOP UPDATES - This is the key difference!
    // In closed loop, discriminators would compute error and update NCOs
    // Here, we just keep using the same frequencies
    
    // Output current state
    *out = *d_acquisition_gnss_synchro;
    out->Prompt_I = d_Prompt.real();
    out->Prompt_Q = d_Prompt.imag();
    out->Carrier_Doppler_hz = d_carrier_doppler_hz; // Fixed value
    out->Code_phase_chips = d_code_phase_chips;
    out->Tracking_sample_counter = d_sample_counter;
    
    d_sample_counter += d_correlation_length_samples;
    
    if (d_dump)
    {
        save_correlation_results();
    }
    
    consume_each(d_correlation_length_samples);
    return 1;
}

void gps_l1_ca_open_loop_tracking_cc::save_correlation_results()
{
    // Save tracking data for analysis
    float dump_data[10];
    dump_data[0] = static_cast<float>(d_sample_counter);
    dump_data[1] = d_Early.real();
    dump_data[2] = d_Early.imag();
    dump_data[3] = d_Prompt.real();
    dump_data[4] = d_Prompt.imag();
    dump_data[5] = d_Late.real();
    dump_data[6] = d_Late.imag();
    dump_data[7] = static_cast<float>(d_carrier_doppler_hz);
    dump_data[8] = static_cast<float>(d_code_phase_chips);
    dump_data[9] = static_cast<float>(d_carrier_phase_rad);
    
    d_dump_file.write(reinterpret_cast<char*>(dump_data), sizeof(dump_data));
}

void gps_l1_ca_open_loop_tracking_cc::forecast(int noutput_items, 
                                               gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = d_correlation_length_samples;
}
