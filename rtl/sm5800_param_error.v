//==============================================================================
// Module      : sm5800_param_error
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001)
// Description : Elaboration-time parameter guard target (intentionally empty).
//
//               Verilog-2001 has no $error. Each parameter guard in this IP is
//               a generate-if that is FALSE for every legal configuration. For
//               an ILLEGAL configuration the guard block is elaborated and
//               connects a port that this module deliberately does not have,
//               e.g.
//                   sm5800_param_error u_param_error (
//                       .SM5800_PARAM_ERROR_RESET_PULSE_US_NOT_IN_1001_TO_65534(1'b0));
//               so elaboration / synthesis stops with "port ... is not a port
//               of u_param_error" (or equivalent) naming the problem.
//
//               Because this module exists, tool source-hierarchy views (e.g.
//               Vivado "Sources") show no missing module. It has no ports and
//               no logic; for legal configurations it is never elaborated.
//               Do NOT add ports to this module.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_param_error;
endmodule

`default_nettype wire
