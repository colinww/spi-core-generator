/*
 * spi_top.v
 * Top-level module which encompasses the SPI map containing all the
 * control registers, and the digital register test interface.
 */
module spi_top(
	       // Outputs
	       o_ena_spi_clk, o_sdo, o_drvb,
	       // Map read-only inputs
               vi_adc_data, vi_dco_freq_count, i_dtst_value_a, i_dtst_value_b,
	       // Map write-only outputs
               o_adc_samp, o_clk_step, o_dco_step_clk, o_dcycle_cal_0_step_clk,
               o_dcycle_cal_1_step_clk, o_dlc_f2p_step_clk,
               o_dlc_iir_0_step_clk, o_dlc_iir_1_step_clk,
               o_dlc_iir_2_step_clk, o_dlc_iir_3_step_clk, o_dlc_pi_step_clk,
               o_dlc_qnc_step_clk, o_dlc_step_clk, o_fdelta_cal_0_step_clk,
               o_fdelta_cal_1_step_clk, o_fdelta_lms_0_step_clk,
               o_fdelta_lms_1_step_clk, o_get_dco_freq, o_rfdc_0_step_clk,
               o_rfdc_1_step_clk, o_rfdc_state_0_step_clk,
               o_rfdc_state_1_step_clk, o_srst_dco, o_srst_dlc, o_srst_rfdc_0,
               o_srst_rfdc_1, o_sync_samp_0, o_sync_samp_1,
	       // Map registered outputs
               o_adc_pol, vo_atst_dac_sel, o_atst_drive_pad,
               o_atst_pwm_dac_sel, vo_atst_sel, o_clkgen_clk_dco_pol,
               vo_clkgen_clkf_sel, vo_clkgen_clkr_sel, vo_clkgen_clkv_sel,
               vo_clkgen_dco_del, vo_clkgen_dlc_del, vo_clkgen_rfdc_del_0,
               vo_clkgen_rfdc_del_1, vo_dac_data, vo_dco_amp_trim,
               vo_dco_buffer_bias, o_dco_byp_agc, o_dco_ena_div_buffer,
               o_dco_ena_fce_dem, o_dco_hold, o_dco_pol, vo_dco_requant_type,
               vo_dco_tune, vo_dcycle_byp_m_0, vo_dcycle_byp_m_1,
               o_dcycle_cal_0_hold, o_dcycle_cal_1_hold, o_dcycle_ena_cal_0,
               o_dcycle_ena_cal_1, vo_dcycle_hyst_0, vo_dcycle_hyst_1,
               vo_dcycle_target_0, vo_dcycle_target_1, o_dlc_ena_qnc,
               o_dlc_ena_type2, o_dlc_f2p_hold, o_dlc_hold, o_dlc_iir_0_hold,
               o_dlc_iir_1_hold, o_dlc_iir_2_hold, o_dlc_iir_3_hold,
               o_dlc_iir_ena_0, o_dlc_iir_ena_1, o_dlc_iir_ena_2,
               o_dlc_iir_ena_3, o_dlc_pi_hold, o_dlc_qnc_hold, vo_dmro_ifast_0,
               vo_dmro_ifast_1, vo_dmro_islow_0, vo_dmro_islow_1,
               vo_dmro_reg_frc_0, vo_dmro_reg_frc_1, o_dmro_rst_0,
               o_dmro_rst_1, o_dmro_sel_manual_0, o_dmro_sel_manual_1,
               vo_dmro_tune_0, vo_dmro_tune_1, o_dtst_drive_pad_a,
               o_dtst_drive_pad_b, vo_dtst_sel_a, vo_dtst_sel_b, o_ena_dco,
               o_ena_dlc, o_ena_ext_osc, o_ena_fbdiv_0, o_ena_fbdiv_1,
               o_ena_rfdc_0, o_ena_rfdc_1, vo_fdelta_byp_knorm_0,
               vo_fdelta_byp_knorm_1, o_fdelta_cal_0_hold, o_fdelta_cal_1_hold,
               vo_fdelta_cal_gain_0, vo_fdelta_cal_gain_1, o_fdelta_del_v_0,
               o_fdelta_del_v_1, o_fdelta_ena_cal_0, o_fdelta_ena_cal_1,
               o_fdelta_lms_0_hold, o_fdelta_lms_1_hold, vo_frac_alpha,
               vo_freq_meas_sel, o_gate_dco_fast, o_gate_dco_slow, o_gate_dlc,
               o_gate_rfdc_0, o_gate_rfdc_1, o_ibias_sel_ext, vo_iir_pole_0,
               vo_iir_pole_1, vo_iir_pole_2, vo_iir_pole_3, vo_integer_n,
               o_ldo_dco_byp, o_ldo_dco_dis, o_ldo_dig_byp, vo_ldo_dig_trim,
               o_ldo_dmro_byp_0, o_ldo_dmro_byp_1, vo_ldo_dmro_trim_0,
               vo_ldo_dmro_trim_1, vo_ldo_fce_trim, vo_ldo_osc_trim,
               o_ldo_pfd_byp_0, o_ldo_pfd_byp_1, vo_ldo_pfd_trim_0,
               vo_ldo_pfd_trim_1, o_ldo_rfdc_dis_0, o_ldo_rfdc_dis_1,
               vo_ldo_rfdiv_trim, o_ldo_sync_byp_0, o_ldo_sync_byp_1,
               vo_ldo_sync_trim_0, vo_ldo_sync_trim_1, o_ldo_xo_byp,
               o_ldo_xo_dis, vo_ldo_xo_trim, vo_num_ref_periods, vo_pfd_mode_0,
               vo_pfd_mode_1, o_pfd_pol_0, o_pfd_pol_1, vo_pi_ki, vo_pi_km,
               vo_pi_kp, vo_pswap_count_0, vo_pswap_count_1, o_pswap_ena_0,
               o_pswap_ena_1, vo_ref_clk_sel, o_ref_pol_0, o_ref_pol_1,
               o_rfdc_0_hold, o_rfdc_1_hold, vo_rfdc_path_sel, o_rfdc_pol_0,
               o_rfdc_pol_1, o_rfdc_state_0_hold, o_rfdc_state_1_hold,
               o_rng_ena_dco_req, o_rng_ena_dlc_req, o_rng_ena_iir_req_0,
               o_rng_ena_iir_req_1, o_rng_ena_iir_req_2, o_rng_ena_iir_req_3,
               o_rng_ena_pi_req, o_rng_ena_pswap_0, o_rng_ena_pswap_1,
               vo_sync_del_0, vo_sync_del_1, o_sync_mode_0, o_sync_mode_1,
               o_sync_pol_0, o_sync_pol_1,
	       // Register read signals
               o_dco_rd, o_dcycle_cal_0_rd, o_dcycle_cal_1_rd, o_dlc_rd,
               o_dlc_f2p_rd, o_dlc_iir_0_rd, o_dlc_iir_1_rd, o_dlc_iir_2_rd,
               o_dlc_iir_3_rd, o_dlc_pi_rd, o_dlc_qnc_rd, o_fdelta_cal_0_rd,
               o_fdelta_cal_1_rd, o_fdelta_lms_0_rd, o_fdelta_lms_1_rd,
               o_rfdc_0_rd, o_rfdc_1_rd, o_rfdc_state_0_rd, o_rfdc_state_1_rd,
	       // Register write signals
               o_dco_wr, o_dcycle_cal_0_wr, o_dcycle_cal_1_wr, o_dlc_wr,
               o_dlc_f2p_wr, o_dlc_iir_0_wr, o_dlc_iir_1_wr, o_dlc_iir_2_wr,
               o_dlc_iir_3_wr, o_dlc_pi_wr, o_dlc_qnc_wr, o_fdelta_cal_0_wr,
               o_fdelta_cal_1_wr, o_fdelta_lms_0_wr, o_fdelta_lms_1_wr,
               o_rfdc_0_wr, o_rfdc_1_wr, o_rfdc_state_0_wr, o_rfdc_state_1_wr,
	       // Inouts
	       io_success, vio_tbus,
	       // Inputs
	       i_rstb, i_clk_ext_osc, i_cs, i_sck, i_sdi);
  // Clock and reset
  input        i_rstb;
  input        i_clk_ext_osc;
  output       o_ena_spi_clk;
  // SPI interface
  output       o_sdo;
  output       o_drvb;
  input        i_cs;
  input        i_sck;
  input        i_sdi;
  // SPI map controls
  // Read-only inputs
  input      [7:0]   vi_adc_data;
  input      [15:0]  vi_dco_freq_count;
  input              i_dtst_value_a;
  input              i_dtst_value_b;
  // Write-only inputs
  output            o_adc_samp;
  output            o_clk_step;
  output            o_dco_step_clk;
  output            o_dcycle_cal_0_step_clk;
  output            o_dcycle_cal_1_step_clk;
  output            o_dlc_f2p_step_clk;
  output            o_dlc_iir_0_step_clk;
  output            o_dlc_iir_1_step_clk;
  output            o_dlc_iir_2_step_clk;
  output            o_dlc_iir_3_step_clk;
  output            o_dlc_pi_step_clk;
  output            o_dlc_qnc_step_clk;
  output            o_dlc_step_clk;
  output            o_fdelta_cal_0_step_clk;
  output            o_fdelta_cal_1_step_clk;
  output            o_fdelta_lms_0_step_clk;
  output            o_fdelta_lms_1_step_clk;
  output            o_get_dco_freq;
  output            o_rfdc_0_step_clk;
  output            o_rfdc_1_step_clk;
  output            o_rfdc_state_0_step_clk;
  output            o_rfdc_state_1_step_clk;
  output            o_srst_dco;
  output            o_srst_dlc;
  output            o_srst_rfdc_0;
  output            o_srst_rfdc_1;
  output            o_sync_samp_0;
  output            o_sync_samp_1;
  // Registered outputs
  output             o_adc_pol;
  output     [3:0]   vo_atst_dac_sel;
  output             o_atst_drive_pad;
  output             o_atst_pwm_dac_sel;
  output     [4:0]   vo_atst_sel;
  output             o_clkgen_clk_dco_pol;
  output     [1:0]   vo_clkgen_clkf_sel;
  output     [1:0]   vo_clkgen_clkr_sel;
  output     [1:0]   vo_clkgen_clkv_sel;
  output     [3:0]   vo_clkgen_dco_del;
  output     [3:0]   vo_clkgen_dlc_del;
  output     [3:0]   vo_clkgen_rfdc_del_0;
  output     [3:0]   vo_clkgen_rfdc_del_1;
  output     [9:0]   vo_dac_data;
  output     [2:0]   vo_dco_amp_trim;
  output     [2:0]   vo_dco_buffer_bias;
  output             o_dco_byp_agc;
  output             o_dco_ena_div_buffer;
  output             o_dco_ena_fce_dem;
  output             o_dco_hold;
  output             o_dco_pol;
  output     [1:0]   vo_dco_requant_type;
  output     [6:0]   vo_dco_tune;
  output     [6:0]   vo_dcycle_byp_m_0;
  output     [6:0]   vo_dcycle_byp_m_1;
  output             o_dcycle_cal_0_hold;
  output             o_dcycle_cal_1_hold;
  output             o_dcycle_ena_cal_0;
  output             o_dcycle_ena_cal_1;
  output     [3:0]   vo_dcycle_hyst_0;
  output     [3:0]   vo_dcycle_hyst_1;
  output     [7:0]   vo_dcycle_target_0;
  output     [7:0]   vo_dcycle_target_1;
  output             o_dlc_ena_qnc;
  output             o_dlc_ena_type2;
  output             o_dlc_f2p_hold;
  output             o_dlc_hold;
  output             o_dlc_iir_0_hold;
  output             o_dlc_iir_1_hold;
  output             o_dlc_iir_2_hold;
  output             o_dlc_iir_3_hold;
  output             o_dlc_iir_ena_0;
  output             o_dlc_iir_ena_1;
  output             o_dlc_iir_ena_2;
  output             o_dlc_iir_ena_3;
  output             o_dlc_pi_hold;
  output             o_dlc_qnc_hold;
  output     [6:0]   vo_dmro_ifast_0;
  output     [6:0]   vo_dmro_ifast_1;
  output     [3:0]   vo_dmro_islow_0;
  output     [3:0]   vo_dmro_islow_1;
  output     [1:0]   vo_dmro_reg_frc_0;
  output     [1:0]   vo_dmro_reg_frc_1;
  output             o_dmro_rst_0;
  output             o_dmro_rst_1;
  output             o_dmro_sel_manual_0;
  output             o_dmro_sel_manual_1;
  output     [2:0]   vo_dmro_tune_0;
  output     [2:0]   vo_dmro_tune_1;
  output             o_dtst_drive_pad_a;
  output             o_dtst_drive_pad_b;
  output     [4:0]   vo_dtst_sel_a;
  output     [4:0]   vo_dtst_sel_b;
  output             o_ena_dco;
  output             o_ena_dlc;
  output             o_ena_ext_osc;
  output             o_ena_fbdiv_0;
  output             o_ena_fbdiv_1;
  output             o_ena_rfdc_0;
  output             o_ena_rfdc_1;
  output     [9:0]   vo_fdelta_byp_knorm_0;
  output     [9:0]   vo_fdelta_byp_knorm_1;
  output             o_fdelta_cal_0_hold;
  output             o_fdelta_cal_1_hold;
  output     [2:0]   vo_fdelta_cal_gain_0;
  output     [2:0]   vo_fdelta_cal_gain_1;
  output             o_fdelta_del_v_0;
  output             o_fdelta_del_v_1;
  output             o_fdelta_ena_cal_0;
  output             o_fdelta_ena_cal_1;
  output             o_fdelta_lms_0_hold;
  output             o_fdelta_lms_1_hold;
  output     [15:0]  vo_frac_alpha;
  output     [1:0]   vo_freq_meas_sel;
  output             o_gate_dco_fast;
  output             o_gate_dco_slow;
  output             o_gate_dlc;
  output             o_gate_rfdc_0;
  output             o_gate_rfdc_1;
  output             o_ibias_sel_ext;
  output     [2:0]   vo_iir_pole_0;
  output     [2:0]   vo_iir_pole_1;
  output     [2:0]   vo_iir_pole_2;
  output     [2:0]   vo_iir_pole_3;
  output     [7:0]   vo_integer_n;
  output             o_ldo_dco_byp;
  output             o_ldo_dco_dis;
  output             o_ldo_dig_byp;
  output     [2:0]   vo_ldo_dig_trim;
  output             o_ldo_dmro_byp_0;
  output             o_ldo_dmro_byp_1;
  output     [2:0]   vo_ldo_dmro_trim_0;
  output     [2:0]   vo_ldo_dmro_trim_1;
  output     [2:0]   vo_ldo_fce_trim;
  output     [2:0]   vo_ldo_osc_trim;
  output             o_ldo_pfd_byp_0;
  output             o_ldo_pfd_byp_1;
  output     [2:0]   vo_ldo_pfd_trim_0;
  output     [2:0]   vo_ldo_pfd_trim_1;
  output             o_ldo_rfdc_dis_0;
  output             o_ldo_rfdc_dis_1;
  output     [2:0]   vo_ldo_rfdiv_trim;
  output             o_ldo_sync_byp_0;
  output             o_ldo_sync_byp_1;
  output     [2:0]   vo_ldo_sync_trim_0;
  output     [2:0]   vo_ldo_sync_trim_1;
  output             o_ldo_xo_byp;
  output             o_ldo_xo_dis;
  output     [2:0]   vo_ldo_xo_trim;
  output     [1:0]   vo_num_ref_periods;
  output     [1:0]   vo_pfd_mode_0;
  output     [1:0]   vo_pfd_mode_1;
  output             o_pfd_pol_0;
  output             o_pfd_pol_1;
  output     [3:0]   vo_pi_ki;
  output     [1:0]   vo_pi_km;
  output     [3:0]   vo_pi_kp;
  output     [5:0]   vo_pswap_count_0;
  output     [5:0]   vo_pswap_count_1;
  output             o_pswap_ena_0;
  output             o_pswap_ena_1;
  output     [1:0]   vo_ref_clk_sel;
  output             o_ref_pol_0;
  output             o_ref_pol_1;
  output             o_rfdc_0_hold;
  output             o_rfdc_1_hold;
  output     [1:0]   vo_rfdc_path_sel;
  output             o_rfdc_pol_0;
  output             o_rfdc_pol_1;
  output             o_rfdc_state_0_hold;
  output             o_rfdc_state_1_hold;
  output             o_rng_ena_dco_req;
  output             o_rng_ena_dlc_req;
  output             o_rng_ena_iir_req_0;
  output             o_rng_ena_iir_req_1;
  output             o_rng_ena_iir_req_2;
  output             o_rng_ena_iir_req_3;
  output             o_rng_ena_pi_req;
  output             o_rng_ena_pswap_0;
  output             o_rng_ena_pswap_1;
  output     [3:0]   vo_sync_del_0;
  output     [3:0]   vo_sync_del_1;
  output             o_sync_mode_0;
  output             o_sync_mode_1;
  output             o_sync_pol_0;
  output             o_sync_pol_1;
  // Digital register interface
  inout        io_success;
  inout [36:0] vio_tbus;
  // Register read signals
  output            o_dco_rd;
  output            o_dcycle_cal_0_rd;
  output            o_dcycle_cal_1_rd;
  output            o_dlc_rd;
  output            o_dlc_f2p_rd;
  output            o_dlc_iir_0_rd;
  output            o_dlc_iir_1_rd;
  output            o_dlc_iir_2_rd;
  output            o_dlc_iir_3_rd;
  output            o_dlc_pi_rd;
  output            o_dlc_qnc_rd;
  output            o_fdelta_cal_0_rd;
  output            o_fdelta_cal_1_rd;
  output            o_fdelta_lms_0_rd;
  output            o_fdelta_lms_1_rd;
  output            o_rfdc_0_rd;
  output            o_rfdc_1_rd;
  output            o_rfdc_state_0_rd;
  output            o_rfdc_state_1_rd;
  // Register write signals
  output            o_dco_wr;
  output            o_dcycle_cal_0_wr;
  output            o_dcycle_cal_1_wr;
  output            o_dlc_wr;
  output            o_dlc_f2p_wr;
  output            o_dlc_iir_0_wr;
  output            o_dlc_iir_1_wr;
  output            o_dlc_iir_2_wr;
  output            o_dlc_iir_3_wr;
  output            o_dlc_pi_wr;
  output            o_dlc_qnc_wr;
  output            o_fdelta_cal_0_wr;
  output            o_fdelta_cal_1_wr;
  output            o_fdelta_lms_0_wr;
  output            o_fdelta_lms_1_wr;
  output            o_rfdc_0_wr;
  output            o_rfdc_1_wr;
  output            o_rfdc_state_0_wr;
  output            o_rfdc_state_1_wr;
  /*
   * SPI core
   */
  wire 	       spi_active;
  wire 	       rx_valid;
  wire [2:0]   v_byte_num;
  wire [7:0]   v_data_rx;
  wire [7:0]   v_data_tx;
  assign o_ena_spi_clk = spi_active;
  spi_core #(.BITS_IN_BYTE(8), .NBIT_BIT_CTR(3), .NBIT_BYTE_CTR(3))
    spi_interface(
		  // Outputs
		  .o_sdo		(o_sdo),
		  .o_drvb		(o_drvb),
		  .o_spi_active		(spi_active),
		  .o_rx_valid		(rx_valid),
		  .vo_byte_num		(v_byte_num),
		  .vo_data_rx		(v_data_rx),
		  // Inputs
		  .i_rstb		(i_rstb),
		  .i_cs			(i_cs),
		  .i_sck		(i_sck),
		  .i_sdi		(i_sdi),
		  .i_epol		(1'b1),
		  .i_cpol		(1'b1),
		  .i_cpha		(1'b0),
		  .vi_data_tx		(v_data_tx));
  /*
   * SPI map
   */
  spi_map
    spi_map_interface(
		      // Read-only inputs
                      .vi_adc_data       (vi_adc_data),
                      .vi_dco_freq_count (vi_dco_freq_count),
                      .i_dtst_value_a    (i_dtst_value_a),
                      .i_dtst_value_b    (i_dtst_value_b),
		      // Write-only inputs
                      .o_adc_samp              (o_adc_samp),
                      .o_clk_step              (o_clk_step),
                      .o_dco_step_clk          (o_dco_step_clk),
                      .o_dcycle_cal_0_step_clk (o_dcycle_cal_0_step_clk),
                      .o_dcycle_cal_1_step_clk (o_dcycle_cal_1_step_clk),
                      .o_dlc_f2p_step_clk      (o_dlc_f2p_step_clk),
                      .o_dlc_iir_0_step_clk    (o_dlc_iir_0_step_clk),
                      .o_dlc_iir_1_step_clk    (o_dlc_iir_1_step_clk),
                      .o_dlc_iir_2_step_clk    (o_dlc_iir_2_step_clk),
                      .o_dlc_iir_3_step_clk    (o_dlc_iir_3_step_clk),
                      .o_dlc_pi_step_clk       (o_dlc_pi_step_clk),
                      .o_dlc_qnc_step_clk      (o_dlc_qnc_step_clk),
                      .o_dlc_step_clk          (o_dlc_step_clk),
                      .o_fdelta_cal_0_step_clk (o_fdelta_cal_0_step_clk),
                      .o_fdelta_cal_1_step_clk (o_fdelta_cal_1_step_clk),
                      .o_fdelta_lms_0_step_clk (o_fdelta_lms_0_step_clk),
                      .o_fdelta_lms_1_step_clk (o_fdelta_lms_1_step_clk),
                      .o_get_dco_freq          (o_get_dco_freq),
                      .o_rfdc_0_step_clk       (o_rfdc_0_step_clk),
                      .o_rfdc_1_step_clk       (o_rfdc_1_step_clk),
                      .o_rfdc_state_0_step_clk (o_rfdc_state_0_step_clk),
                      .o_rfdc_state_1_step_clk (o_rfdc_state_1_step_clk),
                      .o_srst_dco              (o_srst_dco),
                      .o_srst_dlc              (o_srst_dlc),
                      .o_srst_rfdc_0           (o_srst_rfdc_0),
                      .o_srst_rfdc_1           (o_srst_rfdc_1),
                      .o_sync_samp_0           (o_sync_samp_0),
                      .o_sync_samp_1           (o_sync_samp_1),
		      // Registered outputs
                      .o_adc_pol             (o_adc_pol),
                      .vo_atst_dac_sel       (vo_atst_dac_sel),
                      .o_atst_drive_pad      (o_atst_drive_pad),
                      .o_atst_pwm_dac_sel    (o_atst_pwm_dac_sel),
                      .vo_atst_sel           (vo_atst_sel),
                      .o_clkgen_clk_dco_pol  (o_clkgen_clk_dco_pol),
                      .vo_clkgen_clkf_sel    (vo_clkgen_clkf_sel),
                      .vo_clkgen_clkr_sel    (vo_clkgen_clkr_sel),
                      .vo_clkgen_clkv_sel    (vo_clkgen_clkv_sel),
                      .vo_clkgen_dco_del     (vo_clkgen_dco_del),
                      .vo_clkgen_dlc_del     (vo_clkgen_dlc_del),
                      .vo_clkgen_rfdc_del_0  (vo_clkgen_rfdc_del_0),
                      .vo_clkgen_rfdc_del_1  (vo_clkgen_rfdc_del_1),
                      .vo_dac_data           (vo_dac_data),
                      .vo_dco_amp_trim       (vo_dco_amp_trim),
                      .vo_dco_buffer_bias    (vo_dco_buffer_bias),
                      .o_dco_byp_agc         (o_dco_byp_agc),
                      .o_dco_ena_div_buffer  (o_dco_ena_div_buffer),
                      .o_dco_ena_fce_dem     (o_dco_ena_fce_dem),
                      .o_dco_hold            (o_dco_hold),
                      .o_dco_pol             (o_dco_pol),
                      .vo_dco_requant_type   (vo_dco_requant_type),
                      .vo_dco_tune           (vo_dco_tune),
                      .vo_dcycle_byp_m_0     (vo_dcycle_byp_m_0),
                      .vo_dcycle_byp_m_1     (vo_dcycle_byp_m_1),
                      .o_dcycle_cal_0_hold   (o_dcycle_cal_0_hold),
                      .o_dcycle_cal_1_hold   (o_dcycle_cal_1_hold),
                      .o_dcycle_ena_cal_0    (o_dcycle_ena_cal_0),
                      .o_dcycle_ena_cal_1    (o_dcycle_ena_cal_1),
                      .vo_dcycle_hyst_0      (vo_dcycle_hyst_0),
                      .vo_dcycle_hyst_1      (vo_dcycle_hyst_1),
                      .vo_dcycle_target_0    (vo_dcycle_target_0),
                      .vo_dcycle_target_1    (vo_dcycle_target_1),
                      .o_dlc_ena_qnc         (o_dlc_ena_qnc),
                      .o_dlc_ena_type2       (o_dlc_ena_type2),
                      .o_dlc_f2p_hold        (o_dlc_f2p_hold),
                      .o_dlc_hold            (o_dlc_hold),
                      .o_dlc_iir_0_hold      (o_dlc_iir_0_hold),
                      .o_dlc_iir_1_hold      (o_dlc_iir_1_hold),
                      .o_dlc_iir_2_hold      (o_dlc_iir_2_hold),
                      .o_dlc_iir_3_hold      (o_dlc_iir_3_hold),
                      .o_dlc_iir_ena_0       (o_dlc_iir_ena_0),
                      .o_dlc_iir_ena_1       (o_dlc_iir_ena_1),
                      .o_dlc_iir_ena_2       (o_dlc_iir_ena_2),
                      .o_dlc_iir_ena_3       (o_dlc_iir_ena_3),
                      .o_dlc_pi_hold         (o_dlc_pi_hold),
                      .o_dlc_qnc_hold        (o_dlc_qnc_hold),
                      .vo_dmro_ifast_0       (vo_dmro_ifast_0),
                      .vo_dmro_ifast_1       (vo_dmro_ifast_1),
                      .vo_dmro_islow_0       (vo_dmro_islow_0),
                      .vo_dmro_islow_1       (vo_dmro_islow_1),
                      .vo_dmro_reg_frc_0     (vo_dmro_reg_frc_0),
                      .vo_dmro_reg_frc_1     (vo_dmro_reg_frc_1),
                      .o_dmro_rst_0          (o_dmro_rst_0),
                      .o_dmro_rst_1          (o_dmro_rst_1),
                      .o_dmro_sel_manual_0   (o_dmro_sel_manual_0),
                      .o_dmro_sel_manual_1   (o_dmro_sel_manual_1),
                      .vo_dmro_tune_0        (vo_dmro_tune_0),
                      .vo_dmro_tune_1        (vo_dmro_tune_1),
                      .o_dtst_drive_pad_a    (o_dtst_drive_pad_a),
                      .o_dtst_drive_pad_b    (o_dtst_drive_pad_b),
                      .vo_dtst_sel_a         (vo_dtst_sel_a),
                      .vo_dtst_sel_b         (vo_dtst_sel_b),
                      .o_ena_dco             (o_ena_dco),
                      .o_ena_dlc             (o_ena_dlc),
                      .o_ena_ext_osc         (o_ena_ext_osc),
                      .o_ena_fbdiv_0         (o_ena_fbdiv_0),
                      .o_ena_fbdiv_1         (o_ena_fbdiv_1),
                      .o_ena_rfdc_0          (o_ena_rfdc_0),
                      .o_ena_rfdc_1          (o_ena_rfdc_1),
                      .vo_fdelta_byp_knorm_0 (vo_fdelta_byp_knorm_0),
                      .vo_fdelta_byp_knorm_1 (vo_fdelta_byp_knorm_1),
                      .o_fdelta_cal_0_hold   (o_fdelta_cal_0_hold),
                      .o_fdelta_cal_1_hold   (o_fdelta_cal_1_hold),
                      .vo_fdelta_cal_gain_0  (vo_fdelta_cal_gain_0),
                      .vo_fdelta_cal_gain_1  (vo_fdelta_cal_gain_1),
                      .o_fdelta_del_v_0      (o_fdelta_del_v_0),
                      .o_fdelta_del_v_1      (o_fdelta_del_v_1),
                      .o_fdelta_ena_cal_0    (o_fdelta_ena_cal_0),
                      .o_fdelta_ena_cal_1    (o_fdelta_ena_cal_1),
                      .o_fdelta_lms_0_hold   (o_fdelta_lms_0_hold),
                      .o_fdelta_lms_1_hold   (o_fdelta_lms_1_hold),
                      .vo_frac_alpha         (vo_frac_alpha),
                      .vo_freq_meas_sel      (vo_freq_meas_sel),
                      .o_gate_dco_fast       (o_gate_dco_fast),
                      .o_gate_dco_slow       (o_gate_dco_slow),
                      .o_gate_dlc            (o_gate_dlc),
                      .o_gate_rfdc_0         (o_gate_rfdc_0),
                      .o_gate_rfdc_1         (o_gate_rfdc_1),
                      .o_ibias_sel_ext       (o_ibias_sel_ext),
                      .vo_iir_pole_0         (vo_iir_pole_0),
                      .vo_iir_pole_1         (vo_iir_pole_1),
                      .vo_iir_pole_2         (vo_iir_pole_2),
                      .vo_iir_pole_3         (vo_iir_pole_3),
                      .vo_integer_n          (vo_integer_n),
                      .o_ldo_dco_byp         (o_ldo_dco_byp),
                      .o_ldo_dco_dis         (o_ldo_dco_dis),
                      .o_ldo_dig_byp         (o_ldo_dig_byp),
                      .vo_ldo_dig_trim       (vo_ldo_dig_trim),
                      .o_ldo_dmro_byp_0      (o_ldo_dmro_byp_0),
                      .o_ldo_dmro_byp_1      (o_ldo_dmro_byp_1),
                      .vo_ldo_dmro_trim_0    (vo_ldo_dmro_trim_0),
                      .vo_ldo_dmro_trim_1    (vo_ldo_dmro_trim_1),
                      .vo_ldo_fce_trim       (vo_ldo_fce_trim),
                      .vo_ldo_osc_trim       (vo_ldo_osc_trim),
                      .o_ldo_pfd_byp_0       (o_ldo_pfd_byp_0),
                      .o_ldo_pfd_byp_1       (o_ldo_pfd_byp_1),
                      .vo_ldo_pfd_trim_0     (vo_ldo_pfd_trim_0),
                      .vo_ldo_pfd_trim_1     (vo_ldo_pfd_trim_1),
                      .o_ldo_rfdc_dis_0      (o_ldo_rfdc_dis_0),
                      .o_ldo_rfdc_dis_1      (o_ldo_rfdc_dis_1),
                      .vo_ldo_rfdiv_trim     (vo_ldo_rfdiv_trim),
                      .o_ldo_sync_byp_0      (o_ldo_sync_byp_0),
                      .o_ldo_sync_byp_1      (o_ldo_sync_byp_1),
                      .vo_ldo_sync_trim_0    (vo_ldo_sync_trim_0),
                      .vo_ldo_sync_trim_1    (vo_ldo_sync_trim_1),
                      .o_ldo_xo_byp          (o_ldo_xo_byp),
                      .o_ldo_xo_dis          (o_ldo_xo_dis),
                      .vo_ldo_xo_trim        (vo_ldo_xo_trim),
                      .vo_num_ref_periods    (vo_num_ref_periods),
                      .vo_pfd_mode_0         (vo_pfd_mode_0),
                      .vo_pfd_mode_1         (vo_pfd_mode_1),
                      .o_pfd_pol_0           (o_pfd_pol_0),
                      .o_pfd_pol_1           (o_pfd_pol_1),
                      .vo_pi_ki              (vo_pi_ki),
                      .vo_pi_km              (vo_pi_km),
                      .vo_pi_kp              (vo_pi_kp),
                      .vo_pswap_count_0      (vo_pswap_count_0),
                      .vo_pswap_count_1      (vo_pswap_count_1),
                      .o_pswap_ena_0         (o_pswap_ena_0),
                      .o_pswap_ena_1         (o_pswap_ena_1),
                      .vo_ref_clk_sel        (vo_ref_clk_sel),
                      .o_ref_pol_0           (o_ref_pol_0),
                      .o_ref_pol_1           (o_ref_pol_1),
                      .o_rfdc_0_hold         (o_rfdc_0_hold),
                      .o_rfdc_1_hold         (o_rfdc_1_hold),
                      .vo_rfdc_path_sel      (vo_rfdc_path_sel),
                      .o_rfdc_pol_0          (o_rfdc_pol_0),
                      .o_rfdc_pol_1          (o_rfdc_pol_1),
                      .o_rfdc_state_0_hold   (o_rfdc_state_0_hold),
                      .o_rfdc_state_1_hold   (o_rfdc_state_1_hold),
                      .o_rng_ena_dco_req     (o_rng_ena_dco_req),
                      .o_rng_ena_dlc_req     (o_rng_ena_dlc_req),
                      .o_rng_ena_iir_req_0   (o_rng_ena_iir_req_0),
                      .o_rng_ena_iir_req_1   (o_rng_ena_iir_req_1),
                      .o_rng_ena_iir_req_2   (o_rng_ena_iir_req_2),
                      .o_rng_ena_iir_req_3   (o_rng_ena_iir_req_3),
                      .o_rng_ena_pi_req      (o_rng_ena_pi_req),
                      .o_rng_ena_pswap_0     (o_rng_ena_pswap_0),
                      .o_rng_ena_pswap_1     (o_rng_ena_pswap_1),
                      .vo_sync_del_0         (vo_sync_del_0),
                      .vo_sync_del_1         (vo_sync_del_1),
                      .o_sync_mode_0         (o_sync_mode_0),
                      .o_sync_mode_1         (o_sync_mode_1),
                      .o_sync_pol_0          (o_sync_pol_0),
                      .o_sync_pol_1          (o_sync_pol_1),
		      // Inouts
		      .vio_data_spi	(v_data_tx),
		      // Inputs
		      .i_rstb		(i_rstb),
		      .i_spi_active	(spi_active),
		      .vi_data_rx	(v_data_rx),
		      .i_rx_valid	(rx_valid),
		      .vi_byte_num	(v_byte_num));
  /*
   * SPI reg
   */
  spi_reg
    spi_reg_interface(// Register read signals
                      .o_dco_rd          (o_dco_rd),
                      .o_dcycle_cal_0_rd (o_dcycle_cal_0_rd),
                      .o_dcycle_cal_1_rd (o_dcycle_cal_1_rd),
                      .o_dlc_rd          (o_dlc_rd),
                      .o_dlc_f2p_rd      (o_dlc_f2p_rd),
                      .o_dlc_iir_0_rd    (o_dlc_iir_0_rd),
                      .o_dlc_iir_1_rd    (o_dlc_iir_1_rd),
                      .o_dlc_iir_2_rd    (o_dlc_iir_2_rd),
                      .o_dlc_iir_3_rd    (o_dlc_iir_3_rd),
                      .o_dlc_pi_rd       (o_dlc_pi_rd),
                      .o_dlc_qnc_rd      (o_dlc_qnc_rd),
                      .o_fdelta_cal_0_rd (o_fdelta_cal_0_rd),
                      .o_fdelta_cal_1_rd (o_fdelta_cal_1_rd),
                      .o_fdelta_lms_0_rd (o_fdelta_lms_0_rd),
                      .o_fdelta_lms_1_rd (o_fdelta_lms_1_rd),
                      .o_rfdc_0_rd       (o_rfdc_0_rd),
                      .o_rfdc_1_rd       (o_rfdc_1_rd),
                      .o_rfdc_state_0_rd (o_rfdc_state_0_rd),
                      .o_rfdc_state_1_rd (o_rfdc_state_1_rd),
		      // Register write signals
                      .o_dco_wr          (o_dco_wr),
                      .o_dcycle_cal_0_wr (o_dcycle_cal_0_wr),
                      .o_dcycle_cal_1_wr (o_dcycle_cal_1_wr),
                      .o_dlc_wr          (o_dlc_wr),
                      .o_dlc_f2p_wr      (o_dlc_f2p_wr),
                      .o_dlc_iir_0_wr    (o_dlc_iir_0_wr),
                      .o_dlc_iir_1_wr    (o_dlc_iir_1_wr),
                      .o_dlc_iir_2_wr    (o_dlc_iir_2_wr),
                      .o_dlc_iir_3_wr    (o_dlc_iir_3_wr),
                      .o_dlc_pi_wr       (o_dlc_pi_wr),
                      .o_dlc_qnc_wr      (o_dlc_qnc_wr),
                      .o_fdelta_cal_0_wr (o_fdelta_cal_0_wr),
                      .o_fdelta_cal_1_wr (o_fdelta_cal_1_wr),
                      .o_fdelta_lms_0_wr (o_fdelta_lms_0_wr),
                      .o_fdelta_lms_1_wr (o_fdelta_lms_1_wr),
                      .o_rfdc_0_wr       (o_rfdc_0_wr),
                      .o_rfdc_1_wr       (o_rfdc_1_wr),
                      .o_rfdc_state_0_wr (o_rfdc_state_0_wr),
                      .o_rfdc_state_1_wr (o_rfdc_state_1_wr),
		      // Inouts
		      .vio_data_regs	(v_data_tx),
		      .io_success	(io_success),
		      .vio_tbus		(vio_tbus),
		      // Inputs
		      .i_rstb		(i_rstb),
		      .i_clk_ext_osc	(i_clk_ext_osc),
		      .i_spi_active	(spi_active),
		      .vi_data_rx	(v_data_rx),
		      .i_rx_valid	(rx_valid),
		      .vi_byte_num	(v_byte_num));
  
// Local Variables:
// verilog-library-directories:("/home/colinww/git/doormat_dig/scripts/spi_generation/" "/home/colinww/git/doormat_dig/source/spi_dig/")
// verilog-library-extensions:(".v" ".h")
// End:
endmodule // spi_top
