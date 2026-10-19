#pragma once

#include <atomic>
#include <vector>

//==============================================================
// AUDIO PARAMETERS
//==============================================================

// --- ENVELOPE / ADSR ---
inline std::atomic<double> ENV_ATTACK_MS{ 1.0 };
inline std::atomic<double> ENV_DECAY_MS{ 5.0 };
inline std::atomic<double> ENV_SUSTAIN_LEVEL{ 0.0 };
inline std::atomic<double> ENV_SUSTAIN_MS{ 0.0 };
inline std::atomic<double> ENV_RELEASE_MS{ 10.0 };

// --- TRIGGER SYSTEM ---
inline std::atomic<double> TRIGGER_MIX{ 0.5498383641242981 };
inline std::atomic<double> TRIGGER_MIN_RATE{ 1.0352039337158203 };
inline std::atomic<double> TRIGGER_MAX_RATE{ 100.0 };
inline std::atomic<double> TRIGGER_CURVE_EXPONENT{ 1.9991533756256104 };
inline std::atomic<double> TRIGGER_ACTIVITY_GAIN{ 1.4994652271270752 };
inline std::atomic<double> ABSOLUTE_DISTANCE_SCALE{ 1596.7655029296875 };

// --- FREQUENCY MAPPING ---
inline std::atomic<double> FREQ_MIN{ 60.04456329345703 };
inline std::atomic<double> FREQ_MAX{ 399.910888671875 };
inline std::atomic<double> FREQ_DENSITY_INFLUENCE{ 1.2004797458648682 };
inline std::atomic<double> FREQ_TRANSPOSE{ 0.9997771978378296 };
inline std::atomic<double> FREQ_RADIAL_EXPONENT{ 0.4998663067817688 };
inline std::atomic<double> FREQ_SMOOTHING{ 0.9003350734710693 };

// --- MODAL STRUCTURE ---
inline std::atomic<double> MODE_FREQ_SPREAD{ 0.07996434718370438 };
inline std::atomic<double> MODE_BW_BASE{ 119.9910888671875 };
inline std::atomic<double> MODE_AMP_DECAY{ 0.9995543956756592 };
inline std::atomic<double> MODE_BRIGHTNESS{ 0.9995989203453064 };
inline std::atomic<double> MODE_AMP_BASE{ 0.8100846409797668 };

// --- SPATIAL RENDERING ---
inline std::atomic<double> SPATIAL_GAIN_EXP{ 2.6028082370758057 };
inline std::atomic<double> SPATIAL_DISTANCE_MIN{ 0.040032342076301575 };
inline std::atomic<double> SPATIAL_DISTANCE_MAX{ 1.1993533372879028 };
inline std::atomic<double> SPATIAL_WIDTH{ 2.9935293197631836 };
inline std::atomic<double> SPATIAL_DENSITY_EXP_SCALE{ 3.97359561920166 };

// --- OUTPUT STAGE ---
inline std::atomic<double> OUTPUT_TANH_DRIVE{ 0.5006595253944397 };
inline std::atomic<double> OUTPUT_MASTER_GAIN{ 1.5 };


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
    // --- ADSR ---
    {"ENV_ATTACK_MS",       &ENV_ATTACK_MS},
    {"ENV_DECAY_MS",        &ENV_DECAY_MS},
    {"ENV_SUSTAIN_LEVEL",   &ENV_SUSTAIN_LEVEL},
    {"ENV_SUSTAIN_MS",      &ENV_SUSTAIN_MS},
    {"ENV_RELEASE_MS",      &ENV_RELEASE_MS},

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