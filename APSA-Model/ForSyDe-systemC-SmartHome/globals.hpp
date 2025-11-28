#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <forsyde.hpp>
#include <map>
#include <tuple>

using namespace ForSyDe;

using setpoint= std::vector<double>;

enum simple_scenario_type {SelfAware_OPERATE, normal_OPERATE};
enum saf_scenario_type {normal, selfAware};

typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,1>, std::array<size_t,1>>> E_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,2>, std::array<size_t,2>>> E_AL_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,2>, std::array<size_t,1>>> bo_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,1>, std::array<size_t,2>>> bi_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,2>, std::array<size_t,2>>> D1_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,2>, std::array<size_t,3>>> D2_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,2>, std::array<size_t,2>>> AL_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,3>, std::array<size_t,2>>> A_ALL_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,3>, std::array<size_t,3>>> J_AL_scenario_table_type;
typedef std::map< simple_scenario_type, std::tuple< std::array<size_t,3>, std::array<size_t,3>>> J_AL_AL_scenario_table_type;
typedef std::map< saf_scenario_type   , std::array<size_t, 1>> saf_scenario_table_type;


saf_scenario_table_type saf_table = 
{
    { normal,     std::array<size_t,1>({1}) },
    { selfAware,  std::array<size_t,1>({1}) }
};

AL_scenario_table_type AL_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,2>({1, 1}), std::array<size_t,2>({1, 1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,2>({1, 1}), std::array<size_t,2>({1, 0})) }  //no output to BO
};

J_AL_scenario_table_type J_AL_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,3>({1, 1, 1}), std::array<size_t,3>({1, 1, 1})) },
    { normal_OPERATE   , std::make_tuple(std::array<size_t,3>({1, 1, 1}), std::array<size_t,3>({1, 1, 1})) }
};

bi_scenario_table_type bi_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,1>({5}), std::array<size_t,2>({1, 1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,1>({0}), std::array<size_t,2>({0, 0})) }
};

bo_scenario_table_type bo_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,2>({1, 0}), std::array<size_t,1>({1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,2>({0, 0}), std::array<size_t,1>({0})) }
};

D1_scenario_table_type d1_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,2>({1, 1}), std::array<size_t,2>({2, 1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,2>({0, 0}), std::array<size_t,2>({0, 0})) }
};

D2_scenario_table_type d2_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,2>({1, 1}), std::array<size_t,3>({0, 0, 0})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,2>({0, 0}), std::array<size_t,3>({0, 0, 0})) }
};

J_AL_AL_scenario_table_type J_ALAL_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,3>({1, 1, 1}), std::array<size_t,3>({1, 1, 1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,3>({0, 0, 0}), std::array<size_t,3>({0, 0, 0})) } // if the senario is normal what about the delay???
};

E_AL_scenario_table_type E_AL_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,2>({1, 1}), std::array<size_t,2>({1, 1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,2>({0, 0}), std::array<size_t,2>({0, 0})) }
};

A_ALL_scenario_table_type A_ALL_table = 
{
    { SelfAware_OPERATE, std::make_tuple(std::array<size_t,3>({1, 1, 1}), std::array<size_t,2>({1, 1})) },
    { normal_OPERATE,    std::make_tuple(std::array<size_t,3>({0, 0, 0}), std::array<size_t,2>({0, 0})) }
};

#endif 
