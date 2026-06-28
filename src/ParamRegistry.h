#pragma once

#include <atomic>
#include <vector>

//==============================================================
// AUDIO PARAMETERS
//==============================================================

// --- TRIGGER SYSTEM ---
inline std::atomic<double> TRIGGER_MIX               {0.5};
inline std::atomic<double> TRIGGER_MIN_RATE          {10.0};
inline std::atomic<double> TRIGGER_MAX_RATE          {300.0};
inline std::atomic<double> TRIGGER_CURVE_EXPONENT    {0.7};
inline std::atomic<double> TRIGGER_ACTIVITY_GAIN     {1.0};
inline std::atomic<double> ABSOLUTE_DISTANCE_SCALE   {600.0};

// --- FREQUENCY MAPPING ---
inline std::atomic<double> FREQ_MIN                  {100.0};
inline std::atomic<double> FREQ_MAX                  {1000.0};
inline std::atomic<double> FREQ_DENSITY_INFLUENCE    {1.5};
inline std::atomic<double> FREQ_TRANSPOSE            {1.0};
inline std::atomic<double> FREQ_RADIAL_EXPONENT      {0.6};
inline std::atomic<double> FREQ_SMOOTHING            {0.99};

// --- MODAL STRUCTURE ---
inline std::atomic<double> MODE_FREQ_SPREAD          {0.01};
inline std::atomic<double> MODE_BW_BASE              {80.0};
inline std::atomic<double> MODE_AMP_DECAY            {0.4};
inline std::atomic<double> MODE_BRIGHTNESS           {1.0};
inline std::atomic<double> MODE_AMP_BASE             {0.3};

// --- SPATIAL RENDERING ---
inline std::atomic<double> SPATIAL_GAIN_EXP          {3.0};
inline std::atomic<double> SPATIAL_DISTANCE_MIN      {0.05};
inline std::atomic<double> SPATIAL_DISTANCE_MAX      {1.0};
inline std::atomic<double> SPATIAL_WIDTH             {1.0};
inline std::atomic<double> SPATIAL_DENSITY_EXP_SCALE {2.0};

// --- OUTPUT STAGE ---
inline std::atomic<double> OUTPUT_TANH_DRIVE         {4.0};
inline std::atomic<double> OUTPUT_MASTER_GAIN        {1.0};


//==============================================================
// PARAM REGISTRY
//==============================================================

struct ParamEntry
{
    const char* name;
    std::atomic<double>* value;
};


//==============================================================
// GLOBAL PARAMETER LIST
//==============================================================

inline std::vector<ParamEntry> gAudioParams =
{
    // --- TRIGGER ---
    {"TRIGGER_MIX",               &TRIGGER_MIX},
    {"TRIGGER_MIN_RATE",          &TRIGGER_MIN_RATE},
    {"TRIGGER_MAX_RATE",          &TRIGGER_MAX_RATE},
    {"TRIGGER_CURVE_EXPONENT",    &TRIGGER_CURVE_EXPONENT},
    {"TRIGGER_ACTIVITY_GAIN",     &TRIGGER_ACTIVITY_GAIN},
    {"ABSOLUTE_DISTANCE_SCALE",   &ABSOLUTE_DISTANCE_SCALE},

    // --- FREQUENCY ---
    {"FREQ_MIN",                  &FREQ_MIN},
    {"FREQ_MAX",                  &FREQ_MAX},
    {"FREQ_DENSITY_INFLUENCE",    &FREQ_DENSITY_INFLUENCE},
    {"FREQ_TRANSPOSE",            &FREQ_TRANSPOSE},
    {"FREQ_RADIAL_EXPONENT",      &FREQ_RADIAL_EXPONENT},
    {"FREQ_SMOOTHING",            &FREQ_SMOOTHING},

    // --- MODAL ---
    {"MODE_FREQ_SPREAD",          &MODE_FREQ_SPREAD},
    {"MODE_BW_BASE",              &MODE_BW_BASE},
    {"MODE_AMP_DECAY",            &MODE_AMP_DECAY},
    {"MODE_BRIGHTNESS",           &MODE_BRIGHTNESS},
    {"MODE_AMP_BASE",             &MODE_AMP_BASE},

    // --- SPATIAL ---
    {"SPATIAL_GAIN_EXP",          &SPATIAL_GAIN_EXP},
    {"SPATIAL_DISTANCE_MIN",      &SPATIAL_DISTANCE_MIN},
    {"SPATIAL_DISTANCE_MAX",      &SPATIAL_DISTANCE_MAX},
    {"SPATIAL_WIDTH",             &SPATIAL_WIDTH},
    {"SPATIAL_DENSITY_EXP_SCALE", &SPATIAL_DENSITY_EXP_SCALE},

    // --- OUTPUT ---
    {"OUTPUT_TANH_DRIVE",         &OUTPUT_TANH_DRIVE},
    {"OUTPUT_MASTER_GAIN",        &OUTPUT_MASTER_GAIN},
};
