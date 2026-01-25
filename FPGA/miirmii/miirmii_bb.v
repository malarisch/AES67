
module miirmii (
	ena_10,
	RefClk,
	m_tx_en,
	m_tx_d,
	m_tx_err,
	m_rx_en,
	m_rx_d,
	m_rx_err,
	m_rx_crs,
	m_rx_col,
	rx_clk,
	tx_clk,
	Rstn,
	rmii_crs_dv,
	rmii_rx_d,
	rmii_rx_err,
	rmii_tx_en,
	rmii_tx_d);	

	input		ena_10;
	input		RefClk;
	input		m_tx_en;
	input	[3:0]	m_tx_d;
	input		m_tx_err;
	output		m_rx_en;
	output	[3:0]	m_rx_d;
	output		m_rx_err;
	output		m_rx_crs;
	output		m_rx_col;
	output		rx_clk;
	output		tx_clk;
	input		Rstn;
	input		rmii_crs_dv;
	input	[1:0]	rmii_rx_d;
	input		rmii_rx_err;
	output		rmii_tx_en;
	output	[1:0]	rmii_tx_d;
endmodule
