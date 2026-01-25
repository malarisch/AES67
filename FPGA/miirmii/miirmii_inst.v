	miirmii u0 (
		.ena_10      (<connected-to-ena_10>),      //                    MACSPEED.ena_10
		.RefClk      (<connected-to-RefClk>),      //                  clock_sink.clk
		.m_tx_en     (<connected-to-m_tx_en>),     //               mii_interface.mii_tx_en
		.m_tx_d      (<connected-to-m_tx_d>),      //                            .mii_tx_d
		.m_tx_err    (<connected-to-m_tx_err>),    //                            .mii_tx_err
		.m_rx_en     (<connected-to-m_rx_en>),     //                            .mii_rx_dv
		.m_rx_d      (<connected-to-m_rx_d>),      //                            .mii_rx_d
		.m_rx_err    (<connected-to-m_rx_err>),    //                            .mii_rx_err
		.m_rx_crs    (<connected-to-m_rx_crs>),    //                            .mii_crs
		.m_rx_col    (<connected-to-m_rx_col>),    //                            .mii_col
		.rx_clk      (<connected-to-rx_clk>),      // pcs_mac_rx_clock_connection.clk
		.tx_clk      (<connected-to-tx_clk>),      // pcs_mac_tx_clock_connection.clk
		.Rstn        (<connected-to-Rstn>),        //                  reset_sink.reset_n
		.rmii_crs_dv (<connected-to-rmii_crs_dv>), //              rmii_interface.crs
		.rmii_rx_d   (<connected-to-rmii_rx_d>),   //                            .rxdata
		.rmii_rx_err (<connected-to-rmii_rx_err>), //                            .rxerror
		.rmii_tx_en  (<connected-to-rmii_tx_en>),  //                            .txenable
		.rmii_tx_d   (<connected-to-rmii_tx_d>)    //                            .txdata
	);

