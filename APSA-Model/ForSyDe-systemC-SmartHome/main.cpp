#include <systemc>
#include "top.hpp"

int sc_main(int, char**)
{
    top t("t");
    sc_core::sc_start(10, sc_core::SC_MS); // run a short demo
    return 0;
}