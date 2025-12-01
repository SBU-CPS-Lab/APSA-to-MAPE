#ifndef KERNELS_HPP
#define KERNELS_HPP

#include <forsyde.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <iomanip>
#include <algorithm> 
#include "globals.hpp"

using namespace sc_core;
using namespace ForSyDe;
using namespace std;

void bi_kernel_func(
    tuple<vector<setpoint>, vector<double>>& out,
    const simple_scenario_type& sc,
    const tuple<vector<double>>& inp)
{
    const auto& t_room_vec = get<0>(inp);   

    cout << ">>> [BI] fired\n";
    cout << "    t_room tokens: size=" << t_room_vec.size() << "  values=[";
    for (size_t i = 0; i < t_room_vec.size(); ++i)
        cout << (i? ", ":"") << t_room_vec[i];
    cout << "]\n";

    auto& outTemps = get<0>(out);  // vector< vector<double> >
    auto& outAvg   = get<1>(out);  // vector<double>

    outTemps.clear();
    outAvg.clear();

    if (sc == SelfAware_OPERATE)
    {
        if (t_room_vec.size() < 5)
        {
            cout << "    [BI] not enough tokens (need 5). Produced nothing.\n";
            return; 
        }

        double sum = 0.0;
        for (size_t i = 0; i < 5; ++i) sum += t_room_vec[i];
        const double avg  = sum / 5.0;            
        const double rate = (t_room_vec[4] - t_room_vec[0]) / 4.0;  

        outTemps.resize(1);
        outTemps[0].resize(2);
        outTemps[0][0] = avg;
        outTemps[0][1] = rate;

        outAvg.resize(1);
        outAvg[0] = avg;

        cout << "    [BI] avg=" << avg << "  rate=" << rate << "\n";
        cout << "    [BI] out[0]=[avg,rate]=[" << outTemps[0][0] << ", " << outTemps[0][1] << "]\n";
        cout << "    [BI] out[1]=[avg]=[" << outAvg[0] << "]\n";
    }
    else
    {
        cout << "    [BI] normal_OPERATE -> no outputs (rates = 0)\n";
    }

    cout << ">>> [BI] end\n";
}

void bo_kernel_func(
    tuple<vector<setpoint>>& out,
    const simple_scenario_type& sc,
    const tuple<vector<setpoint>,
    vector<setpoint>>& inp)
{
    auto& out_sp = get<0>(out);
    out_sp.clear();

    cout << ">>> [BO] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    if (sc != SelfAware_OPERATE) {
        cout << "    [BO] normal -> no outputs (rates=0)\n";
        return;
    }

    const auto& al_sp = get<0>(inp); // forward this unchanged

    if (al_sp.empty() || al_sp[0].size() < 2) {
        cout << "    [BO][WARN] AL setpoint malformed; no outputs.\n";
        return;
    }

    out_sp.resize(1);
    out_sp[0].resize(2);
    out_sp[0][0] = al_sp[0][0]; // clg
    out_sp[0][1] = al_sp[0][1]; // htg

    cout << "    [BO] pass-through: sp=[clg=" << out_sp[0][0]
              << ", htg=" << out_sp[0][1] << "]\n"
              << ">>> [BO] end\n";
}
 
void d1_kernel_func(
    tuple<vector<double>, vector<double>>& out,
    const simple_scenario_type& sc,
    const tuple<vector<setpoint>, vector<double>>& inp)
{
    auto& out_to_E_AL_vec    = get<0>(out);
    auto& out_to_J_AL_AL_vec = get<1>(out);

    // reset outputs; will resize according to production rates
    out_to_E_AL_vec.clear();
    out_to_J_AL_AL_vec.clear();

    cout << ">>> [D1] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    if (sc != SelfAware_OPERATE)
    {
        cout << "    [D1] normal_OPERATE -> no outputs (rates=0)\n";
        return; // if your scenario table requires outputs here, adjust accordingly
    }

    // ---- inputs ----
    const auto& bi_pack = get<0>(inp);  // [[mean, speed]]
    const auto& eal_vec = get<1>(inp);  // [state]

    if (bi_pack.empty() || bi_pack[0].size() < 2)
    {
        cout << "    [D1][WARN] bi_pack malformed (need [[mean, speed]]) "
                  << "bi_pack.size()=" << bi_pack.size();
        if (!bi_pack.empty())
            cout << " bi_pack[0].size()=" << bi_pack[0].size();
        cout << " -> no outputs.\n";
        return;
    }
    if (eal_vec.empty())
    {
        cout << "    [D1][WARN] eal_vec empty (need [state]) -> no outputs.\n";
        return;
    }

    const double bi_mean   = bi_pack[0][0];
    const double bi_speed  = bi_pack[0][1];
    const double eal_state = eal_vec[0];

    cout << "    [D1] inputs: BI_mean=" << bi_mean
              << "  BI_speed=" << bi_speed
              << "  E_AL_state=" << eal_state << "\n";

    enum Season { SEASON_UNKNOWN = 0, SEASON_WINTER = 1, SEASON_SUMMER = 2 };

    auto decode_season = [](double code) -> Season {
        if (code == 1.0) return SEASON_WINTER;
        if (code == 2.0) return SEASON_SUMMER;
        return SEASON_UNKNOWN;
    };

    Season model_season    = decode_season(eal_state);
    Season inferred_season = SEASON_UNKNOWN;

    // thresholds (inclusive so 22.0°C -> SUMMER, 15.0°C -> WINTER)
    const double winter_max   = 15.0;   // mean <= 15 -> winter
    const double summer_min   = 22.0;   // mean >= 22 -> summer
    const double speed_thresh = 0.05;   // tie-band bias

    if (bi_mean <= winter_max) {
        inferred_season = SEASON_WINTER;
        cout << "    [D1] rule: mean <= winter_max (" << bi_mean << " <= " << winter_max << ") -> WINTER\n";
    } else if (bi_mean >= summer_min) {
        inferred_season = SEASON_SUMMER;
        cout << "    [D1] rule: mean >= summer_min (" << bi_mean << " >= " << summer_min << ") -> SUMMER\n";
    } else {
        // tie-band: (winter_max, summer_min)
        if (bi_speed >  speed_thresh) {
            inferred_season = SEASON_WINTER;
            cout << "    [D1] tie-band: speed > +" << speed_thresh << " -> WINTER\n";
        } else if (bi_speed < -speed_thresh) {
            inferred_season = SEASON_SUMMER;
            cout << "    [D1] tie-band: speed < -" << speed_thresh << " -> SUMMER\n";
        } else {
            inferred_season = model_season;
            cout << "    [D1] tie-band: |speed| <= " << speed_thresh << " -> keep MODEL\n";
        }
    }

    double out_to_E_AL   = 0.0; // season suggestion/state (0=unknown)
    double mismatch_flag = 0.0; // 1=mismatch, 0=ok

    if (inferred_season != SEASON_UNKNOWN && inferred_season != model_season) {
        out_to_E_AL   = static_cast<double>(inferred_season); // 1 or 2
        mismatch_flag = 1.0;
        cout << "    [D1] mismatch: model=" << model_season
                  << " inferred=" << inferred_season
                  << " -> send inferred, flag=1\n";
    } else {
        out_to_E_AL   = static_cast<double>(model_season);
        mismatch_flag = 0.0;
        cout << "    [D1] agree/unknown -> keep model=" << model_season
                  << " flag=0\n";
    }

    out_to_E_AL_vec.resize(1);
    out_to_J_AL_AL_vec.resize(1);
    out_to_E_AL_vec[0]    = out_to_E_AL;
    out_to_J_AL_AL_vec[0] = mismatch_flag;

    cout << "    [D1] out_to_E_AL=" << out_to_E_AL
              << "  out_to_J_AL_AL=" << mismatch_flag << "\n"
              << ">>> [D1] end\n";
}

void d2_kernel_func(
    tuple<vector<setpoint>,
               vector<double>,
               vector<double>>& out,
    const simple_scenario_type& sc,
    const tuple<vector<setpoint>,
                     vector<setpoint>>& inp)
{
    auto& out_sp     = get<0>(out); // [[clg, htg]]
    auto& out_ok_aid = get<1>(out); // [ok_flag] (aux / optional)
    auto& out_flag   = get<2>(out); // [ok_flag] -> J_AL_AL

    // Reset outputs; we will resize per production rate
    out_sp.clear();
    out_ok_aid.clear();
    out_flag.clear();

    cout << ">>> [D2] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    if (sc != SelfAware_OPERATE) {
        cout << "    [D2] normal -> no outputs (rates=0)\n";
        return;
    }

    const auto& sp_pack   = get<0>(inp); // expect [[clg, htg]]
    const auto& abs_pack  = get<1>(inp); // expect [[abs_clg, abs_htg]]

    if (sp_pack.empty() || sp_pack[0].size() < 2) {
        cout << "    [D2][WARN] setpoint input malformed; no outputs.\n";
        return;
    }
    if (abs_pack.empty() || abs_pack[0].size() < 2) {
        cout << "    [D2][WARN] absolute bounds input malformed; no outputs.\n";
        return;
    }

    const double clg       = sp_pack[0][0];
    const double htg       = sp_pack[0][1];
    const double abs_clg   = abs_pack[0][0];
    const double abs_htg   = abs_pack[0][1];

    cout << "    [D2] inp: sp=[clg=" << clg << ", htg=" << htg
              << "] abs=[clg=" << abs_clg << ", htg=" << abs_htg << "]\n";

    bool logical = true;

    // absolute window must be ordered
    if (!(abs_clg < abs_htg)) {
        cout << "    [D2] invalid abs range (abs_clg >= abs_htg)\n";
        logical = false;
    }

    // current setpoints must be ordered clg < htg
    if (!(clg < htg)) {
        cout << "    [D2] invalid sp ordering (clg >= htg)\n";
        logical = false;
    }

    // current setpoints must lie inside absolute window
    if (clg < abs_clg) {
        cout << "    [D2] clg < abs_clg -> out of bounds\n";
        logical = false;
    }
    if (htg > abs_htg) {
        cout << "    [D2] htg > abs_htg -> out of bounds\n";
        logical = false;
    }

    const double ok_flag = logical ? 0.0 : 1.0;  // << requested: 1 when OK

    // Produce exactly one token per output
    out_sp.resize(1);
    out_sp[0].resize(2);
    out_sp[0][0] = clg;  // pass-through
    out_sp[0][1] = htg;

    out_ok_aid.resize(1);
    out_ok_aid[0] = ok_flag;

    out_flag.resize(1);
    out_flag[0] = ok_flag;

    cout << "    [D2] outputs: pass sp=[clg=" << out_sp[0][0]
              << ", htg=" << out_sp[0][1] << "], ok=" << ok_flag << "\n"
              << ">>> [D2] end\n";
}

void J_AL_kernel_func(
    tuple<vector<double>, vector<double>, vector<double>>& out,  // out[0]=activate?, out[1]=distance, out[2]=stagnation
    const simple_scenario_type& sc,
    const tuple<vector<double>, vector<setpoint>, vector<double>>& inp) // inp[0]=[T_room], inp[1]=[[sp_low, sp_high]], inp[2]=[prev_stag]
{
    auto& out_flag  = get<0>(out);
    auto& out_dist  = get<1>(out);
    auto& out_stag  = get<2>(out);

    // always clear; will resize right before writing
    out_flag.clear();
    out_dist.clear();
    out_stag.clear();

    cout << ">>> [J_AL] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    // Soft-input checks (avoid OOB)
    const auto& vT = get<0>(inp);
    const auto& vSP = get<1>(inp);
    const auto& vPrev = get<2>(inp);

    if (vT.empty() || vSP.empty() || vSP[0].size() < 2 || vPrev.empty()) {
        cout << "    [J_AL][WARN] malformed inputs. "
                  << "sizes: T=" << vT.size()
                  << " SP=" << (vSP.empty() ? 0 : (int)vSP[0].size())
                  << " Prev=" << vPrev.size() << " -> no outputs.\n";
        return;
    }

    const double T_room   = vT[0];
    const double sp_low   = vSP[0][0];
    const double sp_high  = vSP[0][1];
    const int    prev_in  = static_cast<int>(vPrev[0]);

    cout << "    [J_AL] inputs: Tin=" << T_room
              << "  sp_low=" << sp_low
              << "  sp_high=" << sp_high
              << "  prev_stag=" << prev_in << "\n";

    // distance from comfort band
    double distance = 0.0;
    if (T_room < sp_low)      distance = (sp_low  - T_room);
    else if (T_room > sp_high) distance = (T_room - sp_high);
    else                       distance = 0.0;

    // internal memory
    static bool   initialized      = false;
    static double last_distance    = 0.0;
    static int    stagnation_count = 0;

    if (!initialized) {
        last_distance    = distance;
        stagnation_count = max(0, prev_in);  // initialize from prev if you like; else 0
        initialized      = true;
        cout << "    [J_AL] init: last_distance=" << last_distance
                  << "  stagnation_count=" << stagnation_count << "\n";
    } else {
        const double improvement_eps = 0.1;
        const int    max_stuck_steps = 3;

        if (distance > 0.0) {
            if (distance > last_distance - improvement_eps) {
                ++stagnation_count;   // not improving
            } else {
                stagnation_count = 0; // improving
            }
        } else {
            stagnation_count = 0;     // inside band
        }

        last_distance = distance;

        cout << "    [J_AL] distance=" << distance
                  << "  stagnation_count=" << stagnation_count << "\n";
    }

    double flag_self_aware = 0.0;
    if (distance > 0.0 && stagnation_count >= 3) flag_self_aware = 1.0;

    // produce exactly 1 token per output
    out_flag.resize(1); out_dist.resize(1); out_stag.resize(1);
    out_flag[0] = flag_self_aware;
    out_dist[0] = distance;
    out_stag[0] = static_cast<double>(stagnation_count);

    cout << "    [J_AL] outputs: flag=" << out_flag[0]
              << "  distance=" << out_dist[0]
              << "  stagnation=" << out_stag[0] << "\n"
              << ">>> [J_AL] end\n";
}


#include <iostream>
#include <tuple>
#include <vector>
#include <algorithm>  // clamp

void AL_kernel_func(
    tuple<vector<setpoint>, vector<setpoint>>& out,        // out[0] -> E, out[1] -> BO
    const simple_scenario_type& sc,
    const tuple<vector<double>, vector<double>>& inp)       // inp[0]=[Tin], inp[1]=[distance]
{
    auto& out_to_E  = get<0>(out);
    auto& out_to_BO = get<1>(out);

    // clear; will resize as we produce
    out_to_E.clear();
    out_to_BO.clear();

    cout << ">>> [AL] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    const auto& troom_vec    = get<0>(inp);
    const auto& distance_vec = get<1>(inp);

    if (troom_vec.empty() || distance_vec.empty()) {
        cout << "    [AL][WARN] missing inputs -> no outputs.\n";
        return;
    }

    const double Tin      = troom_vec[0];
    const double distance = distance_vec[0];

    cout << "    [AL] inputs: Tin=" << Tin
              << "  distance=" << distance << "\n";

    // adaptive center + symmetric band
    static bool   initialized = false;
    static double center      = 22.0;  // mid of comfort band
    static double band        = 2.5;   // half-width => clg=center-band, htg=center+band

    if (!initialized) {
        center      = Tin;   // start from current temp
        band        = 2.5;
        initialized = true;
        cout << "    [AL] init: center=" << center << " band=" << band << "\n";
    }

    // distance is defined upstream as deviation from band => shift center toward Tin
    const double alpha = 0.2;
    center += alpha * distance;
    center  = clamp(center, 10.0, 35.0);

    // cooling_setpoint = LOWER bound, heating_setpoint = UPPER bound
    double clg = center - band;  // lower
    double htg = center + band;  // upper

    // minimum deadband
    const double DEADBAND = 0.5;
    if (htg < clg + DEADBAND) htg = clg + DEADBAND;

    // optional actuator limits
    clg = clamp(clg, 5.0, 35.0);
    htg = clamp(htg, 5.0, 40.0);

    // produce outputs
    out_to_E.resize(1);
    out_to_E[0] = setpoint{ clg, htg };   // [cooling(lower), heating(upper)]

    if (sc == SelfAware_OPERATE) {
        out_to_BO.resize(1);
        out_to_BO[0] = setpoint{ clg, htg };
    } // else keep BO at 0-rate

    cout << "    [AL] Tin=" << Tin
              << "  center=" << center
              << "  -> clg(lower)=" << clg
              << "  htg(upper)=" << htg << "\n"
              << ">>> [AL] end\n";
}


void A_ALL_kernel_func(
    tuple<vector<double>, vector<setpoint>>& out,
    const simple_scenario_type& sc,
    const tuple<vector<double>, vector<double>, vector<double>>& inp)
{
    auto& out_season  = get<0>(out); // vector<double>
    auto& out_sp_list = get<1>(out); // vector< setpoint=vector<double>{clg,htg} >

    // default: clear outputs; we will resize according to production rates
    out_season.clear();
    out_sp_list.clear();

    cout << ">>> [A_ALL] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    if (sc != SelfAware_OPERATE) {
        cout << "    [A_ALL] normal -> no outputs (rates=0)\n";
        return;
    }

    // inputs
    const auto& bi_mean_vec   = get<1>(inp);
    const auto& season_vec    = get<0>(inp);
    const auto& feedback_vec  = get<2>(inp); // unused here; just log size

    if (bi_mean_vec.empty() || season_vec.empty()) {
        cout << "    [A_ALL][WARN] missing inputs: "
                  << "bi_mean_size=" << bi_mean_vec.size()
                  << " season_size=" << season_vec.size()
                  << " -> no outputs.\n";
        return;
    }

    const double bi_mean     = bi_mean_vec[0];
    const double season_code = season_vec[0];

    cout << "    [A_ALL] inputs: BI_mean=" << bi_mean
              << " season_code=" << season_code
              << " feedback_size=" << feedback_vec.size() << "\n";

    enum Season { SEASON_UNKNOWN=0, SEASON_WINTER=1, SEASON_SUMMER=2 };

    Season season = SEASON_UNKNOWN;
    if (season_code == 1.0) season = SEASON_WINTER;
    else if (season_code == 2.0) season = SEASON_SUMMER;

    static double heater_setpoint = 22.0; // upper bound (htg)
    static double cooler_setpoint = 18.0; // lower bound (clg)

    const double winter_center = 22.0;
    const double summer_center = 24.0;

    const double heater_never_on_sp = 5.0;   // heater only turns on below this -> practically OFF
    const double cooler_never_on_sp = 40.0;  // cooler only turns on above this -> practically OFF

    const double beta = 0.3; // 0<beta<=1

    // minimal deadband: htg >= clg + DB
    const double DEADBAND = 0.5;

    switch (season)
    {
    case SEASON_WINTER:
    {
        // adapt heater (upper bound): if bi_mean < winter_center -> raise htg, else lower htg
        double err = winter_center - bi_mean;           // >0 -> too cold
        double htg = winter_center + beta * err;        // move toward reducing error
        htg = clamp(htg, 16.0, 28.0);

        double clg = cooler_never_on_sp;                // keep cooler “off” in winter

        // enforce deadband: htg >= clg + DEADBAND
        if (htg < clg + DEADBAND) htg = clg + DEADBAND;

        heater_setpoint = htg;
        cooler_setpoint = clg;

        cout << "    [A_ALL] WINTER: err=" << err
                  << " -> clg=" << cooler_setpoint
                  << " htg=" << heater_setpoint << "\n";
        break;
    }
    case SEASON_SUMMER:
    {
        // adapt cooler (lower bound): if bi_mean > summer_center -> lower clg, else raise clg
        double err = summer_center - bi_mean;           // <0 -> too hot
        double clg = summer_center + beta * err;        // move toward reducing error
        clg = clamp(clg, 10.0, 30.0);

        double htg = heater_never_on_sp;                // keep heater “off” in summer

        // enforce deadband: htg >= clg + DEADBAND
        if (htg < clg + DEADBAND) htg = clg + DEADBAND;

        cooler_setpoint = clg;
        heater_setpoint = htg;

        cout << "    [A_ALL] SUMMER: err=" << err
                  << " -> clg=" << cooler_setpoint
                  << " htg=" << heater_setpoint << "\n";
        break;
    }
    case SEASON_UNKNOWN:
    default:
        // keep previous setpoints; just log
        cout << "    [A_ALL] UNKNOWN: keep previous clg=" << cooler_setpoint
                  << " htg=" << heater_setpoint << "\n";
        break;
    }

    // produce exactly one token per output
    out_season.resize(1);
    out_sp_list.resize(1);
    out_sp_list[0].resize(2);

    out_season[0]      = season_code;
    out_sp_list[0][0]  = cooler_setpoint; // clg (lower)
    out_sp_list[0][1]  = heater_setpoint; // htg (upper)

    cout << "    [A_ALL] outputs: season=" << out_season[0]
              << " setpoint=[clg=" << out_sp_list[0][0]
              << ", htg=" << out_sp_list[0][1] << "]\n"
              << ">>> [A_ALL] end\n";
}


void JAL_AL_kernel_func(
    tuple<vector<double>, vector<double>, vector<double>>& out,        // out[0]=deactivate, out[1]=report, out[2]=status
    const simple_scenario_type& sc,
    const tuple<vector<double>, vector<double>, vector<double>>& inp)  // inp[0]=det1, inp[1]=det2, inp[2]=prev
{
    auto& out_deactivate = get<0>(out);
    auto& out_report     = get<1>(out);
    auto& out_status     = get<2>(out);

    out_deactivate.clear();
    out_report.clear();
    out_status.clear();

    cout << ">>> [J_AL_AL] fired. scenario =" << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    if (sc != SelfAware_OPERATE) {
        cout << "    [J_AL_AL] normal -> no outputs (rates=0)\n";
        return;
    }

    double det1 = 0.0;  // interpret as: D1 mismatch flag (0=OK, 1=mismatch)
    double det2 = 0.0;  // interpret as: D2 OK flag (1=OK, 0=bad)
    double prev = 0.0;  // previous OK (1=OK, 0=bad)

    const auto& v0 = get<0>(inp);
    const auto& v1 = get<1>(inp);
    const auto& v2 = get<2>(inp);

    bool d1_ok   = (v0[0] == 0.0);  // D1 mismatch flag: 0 = OK
    bool d2_ok   = (v1[0] == 0.0);  // D2 OK flag:       0 = OK
    bool prev_ok = (v2[0] == 0.0);  // previous OK:      0 = OK

    cout << "    [J_AL_AL] interpreted: d1_ok=" << d1_ok
              << " d2_ok=" << d2_ok
              << " prev_ok=" << prev_ok << "\n";

    // Final OK only if ALL are OK
    double ok_flag = (d1_ok && d2_ok && prev_ok) ? 1.0 : 0.0;

    // --- Produce exactly one token per output ---
    out_deactivate.resize(1);
    out_report.resize(1);
    out_status.resize(1);

    out_deactivate[0] = ok_flag;
    out_report[0]     = ok_flag;
    out_status[0]     = ok_flag;

    cout << "    [J_AL_AL] outputs: deactivate=" << out_deactivate[0]
              << " report=" << out_report[0]
              << " status=" << out_status[0] << "\n"
              << ">>> [J_AL_AL] end\n";
}

 
void E_AL_kernel_func(
    tuple<vector<double>, vector<double>>& out,
    const simple_scenario_type& sc,
    const tuple<vector<double>, vector<double>>& inp)
{
    auto& out_to_D1  = get<0>(out); // season mode to D1 (vector<double>)
    auto& out_to_AALL= get<1>(out); // season mode to A_ALL (vector<double>)

    // Clear outputs by default; we’ll resize according to production rates
    out_to_D1.clear();
    out_to_AALL.clear();

    cout << ">>> [E_AL] fired. scenario="
              << (sc == SelfAware_OPERATE ? "SelfAware" : "normal") << "\n";

    if (sc != SelfAware_OPERATE)
    {
        cout << "    [E_AL] normal -> no outputs (rates=0)\n";
        return;
    }

    // Inputs
    const auto& d1_suggestion_vec = get<0>(inp); // expected size >= 1 (suggested season code)
    const auto& aall_update_vec   = get<1>(inp); // optional (not used here, but logged)

    double suggested = 0.0; // 0=UNKNOWN, 1=WINTER, 2=SUMMER
    if (!d1_suggestion_vec.empty())
        suggested = d1_suggestion_vec[0];

    cout << "    [E_AL] inp: suggested=" << suggested << "\n";

    // Persistent season mode state
    static double season_mode = 0.0;  // 0=UNKNOWN, 1=WINTER, 2=SUMMER
    static bool   initialized = false;

    if (!initialized)
    {
        // First time: adopt a valid suggestion if present
        if (suggested == 1.0 || suggested == 2.0)
        {
            season_mode = suggested;
            initialized = true;
            cout << "    [E_AL] init: adopt suggestion -> season_mode = " << season_mode << "\n";
        }
        else
        {
            // remain UNKNOWN until a valid suggestion arrives
            cout << "    [E_AL] init: no valid suggestion, keep UNKNOWN\n";
        }
    }
    else
    {
        // After init: update only on valid change
        if (suggested == 1.0 || suggested == 2.0)
        {
            if (suggested != season_mode)
            {
                cout << "    [E_AL] update: change season_mode "
                          << season_mode << " -> " << suggested << "\n";
                season_mode = suggested;
            }
            else
            {
                cout << "    [E_AL] update: suggestion equals current mode (" 
                          << season_mode << "), keep\n";
            }
        }
        else
        {
            cout << "    [E_AL] update: invalid/unknown suggestion (" 
                      << suggested << "), keep season_mode=" << season_mode << "\n";
        }
    }

    // Produce exactly one token on each output (rate=1)
    out_to_D1.resize(1);
    out_to_AALL.resize(1);
    out_to_D1[0]   = season_mode;
    out_to_AALL[0] = season_mode;

    cout << "    [E_AL] outputs: to_D1=" << out_to_D1[0]
              << "  to_A_ALL=" << out_to_AALL[0] << "\n"
              << ">>> [E_AL] end\n";
}

#endif