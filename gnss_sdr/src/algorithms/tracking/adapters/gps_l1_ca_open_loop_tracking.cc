/*!
 * \file gps_l1_ca_open_loop_tracking.cc
 * \brief Implementation of an adapter for GPS L1 C/A open loop tracking
 * \author Your Name, 2026
 */

#include "gps_l1_ca_open_loop_tracking.h"
#include "configuration_interface.h"
#include "GPS_L1_CA.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>

GpsL1CaOpenLoopTracking::GpsL1CaOpenLoopTracking(
    ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    DLOG(INFO) << "role " << role;
    
    // Read configuration parameters from .conf file
    std::string default_item_type = "gr_complex";
    std::string item_type = configuration->property(role + ".item_type", default_item_type);
    
    // Get sampling frequency
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", 2048000);
    
    // Get vector length (number of samples per correlation)
    // Typically fs_in / 1000 for 1ms correlation
    vector_length_ = configuration->property(role + ".vector_length", fs_in_ / 1000);
    
    // Early-Late spacing in chips
    early_late_space_chips_ = configuration->property(role + ".early_late_space_chips", 0.5);
    
    // Dump parameters
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", 
                                            std::string("./open_loop_tracking"));
    
    // Create the tracking block
    if (item_type == "gr_complex")
    {
        item_size_ = sizeof(gr_complex);
        tracking_ = gps_l1_ca_open_loop_tracking_make_cc(
            fs_in_,
            vector_length_,
            dump_,
            dump_filename_,
            early_late_space_chips_);
    }
    else
    {
        item_size_ = 0;
        LOG(WARNING) << item_type << " unknown tracking item type.";
    }
    
    channel_ = 0;
    
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
    
    if (in_streams_ > 1)
    {
        LOG(ERROR) << "This implementation only supports one input stream";
    }
    if (out_streams_ > 1)
    {
        LOG(ERROR) << "This implementation only supports one output stream";
    }
}

GpsL1CaOpenLoopTracking::~GpsL1CaOpenLoopTracking()
{
    // Destructor
}

void GpsL1CaOpenLoopTracking::stop_tracking()
{
    // Stop tracking implementation
}

void GpsL1CaOpenLoopTracking::start_tracking()
{
    tracking_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GpsL1CaOpenLoopTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

void GpsL1CaOpenLoopTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GpsL1CaOpenLoopTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
    {
        /* top_block is not null */
    }
    // Nothing to connect internally
}

void GpsL1CaOpenLoopTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
    {
        /* top_block is not null */
    }
    // Nothing to disconnect
}

gr::basic_block_sptr GpsL1CaOpenLoopTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GpsL1CaOpenLoopTracking::get_right_block()
{
    return tracking_;
}