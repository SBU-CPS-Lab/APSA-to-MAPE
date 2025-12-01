#ifndef DETECTORS_HPP
#define DETECTORS_HPP

#include <forsyde.hpp>
#include <cmath>
#include <limits>
#include "globals.hpp"

using namespace sc_core;
using namespace ForSyDe;
using namespace std;

inline void saf_cds_func(saf_scenario_type&       new_scenario,
                         const saf_scenario_type& previous_scenario,
                         tuple<vector<double>  , vector<double>> inp)
{
    auto inp1 = get<0>(inp)[0];
    auto inp2 = get<1>(inp)[0];

    switch (previous_scenario)
    {
        case normal:
            if(inp1 == 1.0)
                new_scenario = selfAware;
            break;
        case selfAware:
            if(inp2 == 1.0)
                new_scenario = normal;
            break;

    }
}

inline void saf_kss_func(tuple<vector<simple_scenario_type>>&    out,
                         const saf_scenario_type& current_scenario,
                         tuple<vector<double>  , vector<double>> inp)
{
    auto& [ks] = out;
    switch (current_scenario)
    {
        case normal:
            ks[0] = normal_OPERATE;
            break;
        case selfAware:
            ks[0] = SelfAware_OPERATE;
            break;
    }
}

#endif 