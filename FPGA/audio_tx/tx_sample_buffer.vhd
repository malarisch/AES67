library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity tx_sample_buffer is
    generic
    (
        samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3 -- number of bytes per sample (e.g., 3 for 24-bit audio)
    );
	port
	(
        sys_clk                    : in std_logic;
        reset_n                    : in std_logic;
		audio_ch0_in			: in std_logic_vector(23 downto 0);
		audio_ch1_in			: in std_logic_vector(23 downto 0);
		audio_ch2_in			: in std_logic_vector(23 downto 0);
		audio_ch3_in			: in std_logic_vector(23 downto 0);
		audio_ch4_in			: in std_logic_vector(23 downto 0);
		audio_ch5_in			: in std_logic_vector(23 downto 0);
		audio_ch6_in			: in std_logic_vector(23 downto 0);
		audio_ch7_in			: in std_logic_vector(23 downto 0);
		audio_ch8_in			: in std_logic_vector(23 downto 0);
		audio_ch9_in			: in std_logic_vector(23 downto 0);
		audio_ch10_in			: in std_logic_vector(23 downto 0);
		audio_ch11_in			: in std_logic_vector(23 downto 0);
		audio_ch12_in			: in std_logic_vector(23 downto 0);
		audio_ch13_in			: in std_logic_vector(23 downto 0);
		audio_ch14_in			: in std_logic_vector(23 downto 0);
		audio_ch15_in			: in std_logic_vector(23 downto 0);
		fs_clk_i      				: in std_logic;
		
		wr_ptr_o					: out std_logic_vector(15 downto 0) := (others => '0'); -- data-octet
        wr_ready_o					: out std_logic := '0';

        read0Addr		: in unsigned(15 downto 0);
		data0_out		: out std_logic_vector(7 downto 0); -- 8 bit

        metering_signal_o : out std_logic_vector(global_channel_count - 1 downto 0);
        metering_clip_o : out std_logic_vector(global_channel_count - 1 downto 0);
        metering_clear_i : in std_logic
	);
end entity;

architecture Behavioral of tx_sample_buffer is
    constant AUDIO_BUFFER_LENGTH : integer := samples_per_channel_depth * global_channel_count * bytes_per_sample * 2; -- total number of bytes in the buffer
    type t_sample_ram is array (0 to AUDIO_BUFFER_LENGTH - 1) of std_logic_vector(7 downto 0);
   	signal sample_ram		: t_sample_ram := (others => (others => '0'));
    signal sample_wr_ptr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;

    signal fs_clk_i_sync1 : std_logic := '0';
    signal fs_clk_i_sync2 : std_logic := '0';
    signal zaudio_sync : std_logic := '0';
    signal current_channel_id : integer range 0 to global_channel_count - 1 := 0;
    signal write_active : std_logic := '0';
    signal byte_count : integer range 0 to bytes_per_sample - 1 := 0;
    
    -- Delayed wr_ready to ensure wr_ptr_o is stable when consumer reads it
    signal wr_ready_pending : std_logic := '0';

    -- Metering CDC
    signal metering_clear_i_sync1 : std_logic := '0';
    signal metering_clear_i_sync2 : std_logic := '0';
    signal metering_clear_last : std_logic := '0';

    -- Metering thresholds (same as rx_ringbuffer)
    constant SAMPLE_BITS : integer := bytes_per_sample * 8;
    -- Clip:   0x640000 ≈ -2 dBFS
    -- Signal: 0x004000 ≈ -42 dBFS
    constant CLIP_THRESHOLD   : unsigned(SAMPLE_BITS - 2 downto 0) := to_unsigned(16#640000#, SAMPLE_BITS - 1);
    constant SIGNAL_THRESHOLD : unsigned(SAMPLE_BITS - 2 downto 0) := to_unsigned(16#004000#, SAMPLE_BITS - 1);
begin
    
    process(sys_clk, reset_n)
    variable data_latch : std_logic_vector(23 downto 0);
    variable v_sample_abs : unsigned(SAMPLE_BITS - 2 downto 0);
    begin
        if reset_n = '0' then
            wr_ptr_o <= (others => '0');
            sample_wr_ptr <= 0;
            fs_clk_i_sync1 <= '0';
            fs_clk_i_sync2 <= '0';
            zaudio_sync <= '0';
            current_channel_id <= 0;
            write_active <= '0';
            byte_count <= 0;
            wr_ready_o <= '0';
            wr_ready_pending <= '0';
            metering_clear_i_sync1 <= '0';
            metering_clear_i_sync2 <= '0';
            metering_clear_last <= '0';
        elsif rising_edge(sys_clk) then
            
            -- Delay wr_ready by 1 clock: wr_ready_pending -> wr_ready_o
            -- This ensures wr_ptr_o is stable when downstream logic reads it
            wr_ready_o <= wr_ready_pending;
            wr_ready_pending <= '0';

            -- Metering clear CDC (toggle detect)
            metering_clear_i_sync1 <= metering_clear_i;
            metering_clear_i_sync2 <= metering_clear_i_sync1;
            if (metering_clear_i_sync2 /= metering_clear_last) then
                metering_clear_last <= metering_clear_i_sync2;
                metering_clip_o <= (others => '0');
                metering_signal_o <= (others => '0');
            end if;

            -- cdc for fs_clk_i to sys_clk domain
            fs_clk_i_sync1 <= fs_clk_i;
            fs_clk_i_sync2 <= fs_clk_i_sync1;
            zaudio_sync <= fs_clk_i_sync2;
            if (zaudio_sync = '0' and fs_clk_i_sync2 = '1') then
                write_active <= '1';
            end if;
            if write_active = '1' then


                case current_channel_id is
                    when 0 => data_latch := audio_ch0_in;
                    when 1 => data_latch := audio_ch1_in;
                    when 2 => data_latch := audio_ch2_in;
                    when 3 => data_latch := audio_ch3_in;
                    when 4 => data_latch := audio_ch4_in;
                    when 5 => data_latch := audio_ch5_in;
                    when 6 => data_latch := audio_ch6_in;
                    when 7 => data_latch := audio_ch7_in;
                    when 8 => data_latch := audio_ch8_in;
                    when 9 => data_latch := audio_ch9_in;
                    when 10 => data_latch := audio_ch10_in;
                    when 11 => data_latch := audio_ch11_in;
                    when 12 => data_latch := audio_ch12_in;
                    when 13 => data_latch := audio_ch13_in;
                    when 14 => data_latch := audio_ch14_in;
                    when 15 => data_latch := audio_ch15_in;
                    when others => data_latch := (others => '0');
                end case;

                -- Metering: clip & signal detection on first byte cycle
                if byte_count = 0 then
                    v_sample_abs := unsigned(data_latch(SAMPLE_BITS - 2 downto 0));
                    if data_latch(SAMPLE_BITS - 1) = '1' then
                        v_sample_abs := not v_sample_abs;
                    end if;
                    if v_sample_abs >= CLIP_THRESHOLD then
                        metering_clip_o(current_channel_id) <= '1';
                    end if;
                    if v_sample_abs >= SIGNAL_THRESHOLD then
                        metering_signal_o(current_channel_id) <= '1';
                    end if;
                end if;

                sample_ram(sample_wr_ptr + current_channel_id * bytes_per_sample + byte_count) <= data_latch(byte_count*8 + 7 downto byte_count*8);
                byte_count <= byte_count + 1;
                if byte_count = bytes_per_sample - 1 then
                    byte_count <= 0;
                    if current_channel_id = global_channel_count - 1 then
                        -- reset to first channel and move write pointer forward by one sample (all channels)
                        current_channel_id <= 0;
                        if sample_wr_ptr + global_channel_count * bytes_per_sample >= AUDIO_BUFFER_LENGTH then
                            sample_wr_ptr <= sample_wr_ptr + global_channel_count * bytes_per_sample - AUDIO_BUFFER_LENGTH;
                            wr_ptr_o <= std_logic_vector(to_unsigned(sample_wr_ptr + global_channel_count * bytes_per_sample - AUDIO_BUFFER_LENGTH, 16));
                        else
                            sample_wr_ptr <= sample_wr_ptr + global_channel_count * bytes_per_sample;
                            wr_ptr_o <= std_logic_vector(to_unsigned(sample_wr_ptr + global_channel_count * bytes_per_sample, 16));
                        end if;
                        write_active <= '0';
                        -- Set pending flag - actual wr_ready_o will be set next clock
                        -- This ensures wr_ptr_o is stable when downstream logic samples it
                        wr_ready_pending <= '1';
                    else
                        current_channel_id <= current_channel_id + 1;
                    end if;    
                end if;
            end if;
        end if;
    end process;

    -- Registered read required for block RAM inference
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            data0_out <= sample_ram(to_integer(read0Addr));
        end if;
    end process;

end Behavioral;