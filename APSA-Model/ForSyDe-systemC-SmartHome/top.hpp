#ifndef TOP_HPP
#define TOP_HPP

#include <forsyde.hpp>
#include "globals.hpp"
#include "kernels.hpp"
#include "detectors.hpp"

using namespace sc_core;
using namespace ForSyDe;
using namespace std;

#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


SC_MODULE(top)
{

    SADF::signal<simple_scenario_type> BI_cntrol , BO_cntrol , D1_cntrol, D2_cntrol, AL_cntrol, J_AL_cntrol, J_ALAL_cntrol, AL_ALL_cntrol, E_AL_cntrol, s_mode;

    // From E (room temperature) to BI, AL, J_AL
    SADF::signal<double> s_T_room ,s_T2_J_AL , s_T2_AL , s_to_E , s_T2_print, Fs_T2_AL;

    // BI: input T_room, output avg and rate
    SADF::signal<vector<double>> s_BI_2D1 ;
    SADF::signal<double> s_fromBI_avgTemp , s_BI_2ALAL , s_BI_2EAL , s_T2_BI;     // avg/rate stream (consumed in blocks according to bi_table)

    // --------------------------------------------------------------------
    // D1 signals
    // --------------------------------------------------------------------
    SADF::signal<double> Ds_D1_to_EAL, s_D1_to_EAL;      // suggested / confirmed season mode to E_AL
    SADF::signal<double> s_D1_to_JALAL;    // mismatch flag to J_AL_AL

    // --------------------------------------------------------------------
    // E_AL signals
    // --------------------------------------------------------------------
    SADF::signal<double> s_EAL_to_D1,Ds_EAL_to_D1;      // current seasonMode back to D1
    SADF::signal<double> s_EAL_to_AALL;    // seasonMode to A_ALL

    // --------------------------------------------------------------------
    // A_ALL signals
    // --------------------------------------------------------------------
    SADF::signal<setpoint> Ds_AALL_to_D2, s_AALL_to_D2;     // expected setpoints for D2 (self-model)
    SADF::signal<double> s_AALL_to_EAL;    // updates to environment abstraction (model update)

    // --------------------------------------------------------------------
    // D2 signals
    // --------------------------------------------------------------------
    SADF::signal<setpoint> s_D2_to_BO, Ds_D2_to_BO;       // correction / selected action to BO
    SADF::signal<double> s_D2_to_AALL, Ds_D2_to_AALL;     // feedback to A_ALL
    SADF::signal<double> s_D2_to_JALAL;    // mismatch info to J_AL_AL

    // --------------------------------------------------------------------
    // BO signals
    // --------------------------------------------------------------------

    SADF::signal<setpoint> s_BO_to_D2, Ds_BO_to_D2;       // final command back to D2 for comparison

    // --------------------------------------------------------------------
    // AL signals
    // --------------------------------------------------------------------
    SADF::signal<setpoint> s_AL_to_JAL , s_AL_to_E, s_from_AL, Ds_from_AL, fs_AL_to_E, s_SP_print;      // setpoints to E (heater/cooler)
    SADF::signal<setpoint> s_AL_to_BO;       // setpoints to BO

    // --------------------------------------------------------------------
    // J_AL signals
    // --------------------------------------------------------------------
    SADF::signal<double> s_JAL_to_AL ,s_E_AL2_JAL , Ds_JAL_to_AL;      // distance / activation flag to AL
    SADF::signal<double> prev_state_JAL , Dprev_state_JAL;

    // --------------------------------------------------------------------
    // A_ALL signals
    // --------------------------------------------------------------------
    SADF::signal<double> s_A_ALL2EAL, Ds_A_ALL2EAL;   
    SADF::signal<double> s_EAL_2AALL, Ds_EAL_2AALL;

    // --------------------------------------------------------------------
    // J_AL_AL signals
    // --------------------------------------------------------------------
    SADF::signal<setpoint> s_A_ALL2_JAL , b;  

    // --------------------------------------------------------------------
    // A_SAF (self-aware supervisor) signals
    // --------------------------------------------------------------------
    SADF::signal<double> s_saf_activate, Ds_saf_activate;   // from J_AL (string command "activate")
    SADF::signal<double> s_saf_deactive, Ds_saf_deactive;   // from J_AL_AL (string command "deactive")

    // --------------------------------------------------------------------
    // Reporting sink signal (for F)
    // --------------------------------------------------------------------
    SADF::signal<double> s_JALAL_report, JALAL_prevState, DJALAL_prevState;         // from J_AL_AL to SDF::sink F

#ifdef FORSYDE_SELF_REPORTING
    FILE* report_pipe;
    int report_pipe_fd = 0;
#endif

    SC_CTOR(top)
    {
        SDF::make_fanout3("fan_tempreture", s_T2_J_AL, s_T2_AL ,s_T2_BI , s_T_room);
        SDF::make_fanout2("fan_setpoint", s_AL_to_JAL, s_AL_to_E, Ds_from_AL);
        SDF::make_fanout2("fan_avg_tempreture", s_BI_2ALAL, s_BI_2EAL, s_fromBI_avgTemp);

        // control signals fron the detector
        SDF::make_fanout9("fan_control" ,BI_cntrol , BO_cntrol , D1_cntrol, D2_cntrol, AL_cntrol, J_AL_cntrol, J_ALAL_cntrol, AL_ALL_cntrol, E_AL_cntrol, s_mode);

        // dalay s_from_AL
        SDF::make_delay("delay:setpoint",vector({18.0,22.0}) , Ds_from_AL, s_from_AL);

        // SADF::make_socketwrap("E", -1, string("0.0.0.0"), 9000, s_T_room, s_AL_to_E);

        // --- E_STUB: temperature source (daily sinus)
        SADF::make_source(
        "E_temp_src",
        []( double& t ,  const double& k) {
            const double Tmean  = 23.0;
            const double Amp    = 2.0;
            const double period = 24.0;
            t = Tmean + Amp * sin(2.0 * M_PI * static_cast<double>(k) / period);
            cout<<"in E :: tempreture -> "<<t<<endl;
        },
        22.0,
        100,        // number of samples
        s_T_room     // output signal
        );
        
        // --- E_STUB: actuator sink (log AL→E setpoints)
        SDF::make_sink(
        "E_act_sink",
        [] (const setpoint& sp) {
            double htg = (sp.size() > 0 ? sp[0] : numeric_limits<double>::quiet_NaN());
            double clg = (sp.size() > 1 ? sp[1] : numeric_limits<double>::quiet_NaN());
            cout << "[E_STUB] setpoint -> htg=" << htg << "  clg=" << clg << endl;
        },
        s_AL_to_E   // AFTER delay, i.e., what would go to the environment/socket
        );

        // dalay the distance
        SDF::make_delay("delay:distnace", 0.0, Ds_JAL_to_AL, s_JAL_to_AL);

        SADF::make_kernelMN("J_L", J_AL_kernel_func, J_AL_table, tie(s_JAL_to_AL, s_saf_activate, prev_state_JAL), J_AL_cntrol, tie(s_T2_J_AL, s_AL_to_JAL, Dprev_state_JAL));
        
        SADF::make_kernelMN("BI", bi_kernel_func, bi_table, tie(s_BI_2D1, s_fromBI_avgTemp), BI_cntrol, tie(s_T2_BI));

        SADF::make_kernelMN("A_L", AL_kernel_func, AL_table, tie(s_from_AL, s_AL_to_BO), AL_cntrol, tie(s_T2_AL, Ds_JAL_to_AL));

        // dalay prev_state_JAL 
        SDF::make_delay("delay:prev_state_JAL", 1.0, Dprev_state_JAL, prev_state_JAL);
        
        // dalay s_D1_to_EAL 
        SDF::make_delay("delay:s_D1_to_EAL", 1.0, Ds_EAL_to_D1, s_EAL_to_D1); 

        SADF::make_kernelMN("D1", d1_kernel_func, d1_table, tie(s_D1_to_EAL, s_D1_to_JALAL), D1_cntrol, tie(s_BI_2D1, Ds_EAL_to_D1));

        //dalay s_D2_to_AALL
        SDF::make_delay("delay:s_D2_to_BO", vector({18.0,22.0}), Ds_BO_to_D2, s_BO_to_D2); 

        SADF::make_kernelMN("D2", d2_kernel_func, d2_table, tie(s_D2_to_BO,  s_D2_to_JALAL , s_D2_to_AALL), D2_cntrol, tie(Ds_BO_to_D2 , s_AALL_to_D2));

        SADF::make_kernelMN("BO", bo_kernel_func, bo_table, tie(s_BO_to_D2), BO_cntrol, tie(s_AL_to_BO , s_D2_to_BO));   //doing nothing

        SADF::make_kernelMN("E_AL", E_AL_kernel_func, E_AL_table, tie(s_EAL_to_D1, s_EAL_2AALL), E_AL_cntrol, tie(s_D1_to_EAL, s_A_ALL2EAL));   

        // // // dalay JALAL_prevState  
        SDF::make_delay("delay:JALAL_prevState", 1.0, DJALAL_prevState, JALAL_prevState);

        SADF::make_kernelMN("J_ALAL", JAL_AL_kernel_func, J_ALAL_table, tie(s_saf_deactive, s_JALAL_report, JALAL_prevState), J_ALAL_cntrol, tie(s_D1_to_JALAL, s_D2_to_JALAL, DJALAL_prevState));  
        
        // // // s_EAL_2AALL 
        SDF::make_delay("delay:s_EAL_2AALL", 0.0, Ds_EAL_2AALL, s_EAL_2AALL);

        // //dalay s_D2_to_AALL
        SDF::make_delay("delay:s_D2_to_AALL", 1.0, Ds_D2_to_AALL, s_D2_to_AALL); 

        SADF::make_kernelMN("A_ALL", A_ALL_kernel_func, A_ALL_table, tie(s_A_ALL2EAL , s_AALL_to_D2), AL_ALL_cntrol, tie(Ds_EAL_2AALL , s_BI_2ALAL , Ds_D2_to_AALL));   
        // dalay the detectors inputs
        SDF::make_delay("delay:acvtive", 0.0, Ds_saf_activate, s_saf_activate);
        SDF::make_delay("delay:deactive", 0.0, Ds_saf_deactive, s_saf_deactive);
        
        SADF::make_detectorMN("AL_SAF", saf_cds_func, saf_kss_func, saf_table, selfAware, {1,1}, tie(s_mode), tie(Ds_saf_activate , Ds_saf_deactive));
  
        SDF::make_sink("F_report",[] (const double& v)
            {
                cout << "[REPORT] mismatch_summary = " << v << endl;
            },
            s_JALAL_report
        );
    }

#ifdef FORSYDE_INTROSPECTION
    void start_of_simulation()
    {
        ForSyDe::XMLExport dumper("gen/");
        dumper.traverse(this);
#ifdef FORSYDE_SELF_REPORTING
        while (report_pipe_fd <= 0)
        {
            report_pipe_fd = open("gen/self_report", O_WRONLY|O_NONBLOCK);
            if (report_pipe_fd > 0)
                report_pipe = fdopen(report_pipe_fd, "w");
        }
#endif
    }
#endif

#ifdef FORSYDE_SELF_REPORTING
    void end_of_simulation()
    {
        fclose(report_pipe);
    }
#endif
};

#endif // TOP_HPP
