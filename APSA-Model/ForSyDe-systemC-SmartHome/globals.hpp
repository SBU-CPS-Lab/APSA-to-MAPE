#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <forsyde.hpp>
#include <array>
#include <map>
#include <tuple>
#include <vector>

using namespace ForSyDe;
using namespace std;

namespace model_types
{
    // Fixed-size array of size_t
    template<size_t N>
    using sz_array = array<size_t, N>;

    // Generic scenario table:
    //   Key         -> scenario enum type
    //   Nin, Nout   -> number of input/output tokens
    template<typename Key, size_t Nin, size_t Nout>
    using scenario_table = map<Key, tuple< sz_array<Nin>, sz_array<Nout> >>;
}

using model_types::sz_array;
using setpoint = vector<double>;

enum simple_scenario_type { SelfAware_OPERATE, normal_OPERATE };
enum saf_scenario_type    { normal, selfAware };

using E_scenario_table_type       = model_types::scenario_table<simple_scenario_type, 1, 1>;
using E_AL_scenario_table_type    = model_types::scenario_table<simple_scenario_type, 2, 2>;
using bo_scenario_table_type      = model_types::scenario_table<simple_scenario_type, 2, 1>;
using bi_scenario_table_type      = model_types::scenario_table<simple_scenario_type, 1, 2>;
using D1_scenario_table_type      = model_types::scenario_table<simple_scenario_type, 2, 2>;
using D2_scenario_table_type      = model_types::scenario_table<simple_scenario_type, 2, 3>;
using AL_scenario_table_type      = model_types::scenario_table<simple_scenario_type, 2, 2>;
using A_ALL_scenario_table_type   = model_types::scenario_table<simple_scenario_type, 3, 2>;
using J_AL_scenario_table_type    = model_types::scenario_table<simple_scenario_type, 3, 3>;
using J_AL_AL_scenario_table_type = model_types::scenario_table<simple_scenario_type, 3, 3>;

using saf_scenario_table_type     = map<saf_scenario_type, sz_array<1>>;

saf_scenario_table_type saf_table =
{
    { normal,    sz_array<1>{1} },
    { selfAware, sz_array<1>{1} }
};

AL_scenario_table_type AL_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<2>{1, 1}, sz_array<2>{1, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<2>{1, 1}, sz_array<2>{1, 0}) }  // no output to BO
};

J_AL_scenario_table_type J_AL_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<3>{1, 1, 1}, sz_array<3>{1, 1, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<3>{1, 1, 1}, sz_array<3>{1, 1, 1}) }
};

bi_scenario_table_type bi_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<1>{1}, sz_array<2>{1, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<1>{0}, sz_array<2>{0, 0}) }
};

bo_scenario_table_type bo_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<2>{1, 0}, sz_array<1>{1}) },
    { normal_OPERATE,    make_tuple(sz_array<2>{0, 0}, sz_array<1>{0}) }
};

D1_scenario_table_type d1_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<2>{1, 1}, sz_array<2>{2, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<2>{0, 0}, sz_array<2>{0, 0}) }
};

D2_scenario_table_type d2_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<2>{1, 1}, sz_array<3>{0, 0, 0}) },
    { normal_OPERATE,    make_tuple(sz_array<2>{0, 0}, sz_array<3>{0, 0, 0}) }
};

J_AL_AL_scenario_table_type J_ALAL_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<3>{1, 1, 1}, sz_array<3>{1, 1, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<3>{0, 0, 0}, sz_array<3>{0, 0, 0}) }
};

E_AL_scenario_table_type E_AL_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<2>{1, 1}, sz_array<2>{1, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<2>{0, 0}, sz_array<2>{0, 0}) }
};

A_ALL_scenario_table_type A_ALL_table =
{
    { SelfAware_OPERATE, make_tuple(sz_array<3>{1, 1, 1}, sz_array<2>{1, 1}) },
    { normal_OPERATE,    make_tuple(sz_array<3>{0, 0, 0}, sz_array<2>{0, 0}) }
};

#endif 
