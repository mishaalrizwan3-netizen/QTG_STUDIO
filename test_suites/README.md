# QTG test folders

Each folder describes one catalog test and points to its existing implementation. Shared runtime source stays in its current location. `test_definition.json` is a documentation export, not a new engine configuration loader.

| Test | Name | Availability |
| --- | --- | --- |
| [1.c.(1)](1_c_1_Normal_climb_all_engines_operating/README.md) | Normal climb -- all engines operating | SEP catalog test |
| [1.c.(2)](1_c_2_One_engine_inoperative_second_segment_climb/README.md) | One engine inoperative second segment climb | MEP disabled |
| [1.f.(1)](1_f_1_Engine_acceleration/README.md) | Engine acceleration | SEP catalog test |
| [1.f.(2)](1_f_2_Engine_deceleration/README.md) | Engine deceleration | SEP catalog test |
| [2.a.(1)](2_a_1_Pitch_control_column_position_vs_force/README.md) | Pitch control column position vs force | SEP catalog test |
| [2.a.(2)](2_a_2_Roll_control_wheel_position_vs_force/README.md) | Roll control wheel position vs force | SEP catalog test |
| [2.a.(3)](2_a_3_Rudder_pedal_position_vs_force/README.md) | Rudder pedal position vs force | SEP catalog test |
| [2.a.(4)](2_a_4_Pitch_trim_system_rate_and_authority/README.md) | Pitch trim system rate and authority | SEP catalog test |
| [2.a.(5)](2_a_5_Rudder_trim_system_rate_and_authority/README.md) | Rudder trim system rate and authority | SEP catalog test |
| [2.b.(1)](2_b_1_Longitudinal_trim/README.md) | Longitudinal trim | SEP catalog test |
| [2.b.(2)](2_b_2_Stick_force_per_g_normal_and_accelerated_turning/README.md) | Stick force per g -- normal and accelerated turning | SEP catalog test |
| [2.b.(3)](2_b_3_Longitudinal_static_stability_cruise_and_approach/README.md) | Longitudinal static stability -- cruise and approach | SEP catalog test |
| [2.b.(4)](2_b_4_Phugoid_dynamics/README.md) | Phugoid dynamics | SEP catalog test |
| [2.b.(5)](2_b_5_Short_period_dynamics/README.md) | Short period dynamics | SEP catalog test |
| [2.b.(6)](2_b_6_Pitch_response_to_pitch_controller_input_step_pulse/README.md) | Pitch response to pitch controller input (step/pulse) | SEP catalog test |
| [2.b.(7)](2_b_7_Pitch_control_authority_dynamic_response/README.md) | Pitch control authority -- dynamic response | SEP catalog test |
| [2.b.(8)](2_b_8_Pitch_trim_change_due_to_flap_deployment/README.md) | Pitch trim change due to flap deployment | SEP catalog test |
| [2.b.(9)](2_b_9_Pitch_trim_change_due_to_gear_deployment/README.md) | Pitch trim change due to gear deployment | SEP catalog test |
| [2.b.(10)](2_b_10_Pitch_trim_change_due_to_thrust_power_change/README.md) | Pitch trim change due to thrust / power change | SEP catalog test |
| [2.c.(1)](2_c_1_Roll_response_rate_and_time_constant/README.md) | Roll response (rate and time constant) | SEP catalog test |
| [2.c.(2)](2_c_2_Spiral_stability/README.md) | Spiral stability | SEP catalog test |
| [2.c.(3)](2_c_3_Dutch_roll_dynamics_yaw_damper_ON_and_OFF/README.md) | Dutch roll dynamics -- yaw damper ON and OFF | SEP catalog test |
| [2.c.(4)](2_c_4_Steady_sideslip_directional_and_lateral_static_stability/README.md) | Steady sideslip (directional and lateral static stability) | SEP catalog test |
| [2.c.(5)](2_c_5_Rudder_response_to_controller_input_step_pulse/README.md) | Rudder response to controller input (step/pulse) | SEP catalog test |
| [2.c.(6)](2_c_6_Aileron_response_to_controller_input_step_pulse/README.md) | Aileron response to controller input (step/pulse) | SEP catalog test |
| [2.c.(7)](2_c_7_Cross_coupling_roll_due_to_rudder_dihedral_effect/README.md) | Cross-coupling -- roll due to rudder (dihedral effect) | SEP catalog test |
| [2.c.(8)](2_c_8_Cross_coupling_yaw_due_to_roll_controller_adverse_yaw/README.md) | Cross-coupling -- yaw due to roll controller (adverse yaw) | SEP catalog test |
| [4.a.(1)](4_a_1_Visual_system_latency_transport_delay/README.md) | Visual system latency / transport delay | SEP catalog test |
| [4.b.(1)](4_b_1_Visual_field_of_view_FOV_geometry/README.md) | Visual field of view (FOV) geometry | SEP catalog test |
| [4.c.(1)](4_c_1_Visual_ground_runway_alignment_and_spatial_geometry/README.md) | Visual ground / runway alignment and spatial geometry | SEP catalog test |

[Shared implementation](_shared/README.md)

Historical recordings without authoritative test metadata have not been assigned to tests by filename guesswork. Their existing locations are preserved.
