library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity tx_sample_buffer is
    generic
    (
        samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3; -- number of bytes per sample (e.g., 3 for 24-bit audio)
        ENABLE_METERING: boolean := true
    );
	port
	(
        sys_clk                    : in std_logic;
        reset_n                    : in std_logic;


        audio_in                 : in STD_LOGIC_VECTOR((bytes_per_sample * 8) * global_channel_count - 1 downto 0);
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
    
    signal ram_wr_en : std_logic := '0';
    signal ram_wr_data : std_logic_vector(7 downto 0);
    signal ram_wr_addr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    -- Delayed wr_ready to ensure wr_ptr_o is stable when consumer reads it
    signal wr_ready_pending : std_logic := '0';

    -- Metering CDC
    signal metering_clear_i_sync1 : std_logic := '0';
    signal metering_clear_i_sync2 : std_logic := '0';
    signal metering_clear_last : std_logic := '0';

    constant SAMPLE_BITS : integer := bytes_per_sample * 8;
    -- Metering thresholds (same as rx_ringbuffer)
    -- Clip:   0x640000 ≈ -2 dBFS
    -- Signal: 0x004000 ≈ -42 dBFS
    -- Only the top CMP_BITS of the magnitude are needed — both thresholds have zero
    -- in the lower (SAMPLE_BITS-1 - CMP_BITS) bits, so truncating does not change the compare.
    constant CMP_BITS : integer := 9;
    constant CLIP_THRESHOLD   : unsigned(CMP_BITS - 1 downto 0) := to_unsigned(16#064#, CMP_BITS); -- 0x640000 >> 14
    constant SIGNAL_THRESHOLD : unsigned(CMP_BITS - 1 downto 0) := to_unsigned(16#001#, CMP_BITS); -- 0x004000 >> 14
    signal metering_ch_id  : integer range 0 to global_channel_count - 1;

    -- Pipeline registers for metering path: break the 16-to-1 mux over audio_in
    -- from the compare logic by registering the selected sample first.
    signal metering_sample_reg : std_logic_vector(SAMPLE_BITS - 1 downto 0) := (others => '0');
    signal metering_ch_id_reg  : integer range 0 to global_channel_count - 1 := 0;
    signal metering_valid_reg  : std_logic := '0';
begin
    
    metering_proc_gen: if (ENABLE_METERING = true) generate
    -- metering process
    -- Pipeline:
    --   Stage 1: select channel from audio_in (large mux) -> metering_sample_reg
    --   Stage 2: abs on top CMP_BITS only, compare against reduced thresholds
    process (sys_clk, reset_n)
        variable metering_active : std_logic := '0';
        variable v_sample_top : unsigned(CMP_BITS - 1 downto 0);
    begin
        if (reset_n = '0') then
            metering_active := '0';
            metering_ch_id <= 0;
            metering_sample_reg <= (others => '0');
            metering_ch_id_reg <= 0;
            metering_valid_reg <= '0';
            metering_clear_last <= '0';
        elsif (rising_edge(sys_clk)) then
            if (zaudio_sync = '0' and fs_clk_i_sync2 = '1') then
                metering_active := '1';
            end if;

            if (metering_clear_i_sync2 /= metering_clear_last) then
                metering_clear_last <= metering_clear_i_sync2;
                metering_clip_o <= (others => '0');
                metering_signal_o <= (others => '0');
            end if;

            -- Stage 1: channel select (the big 16-to-1 mux gets its own clock)
            metering_valid_reg <= '0';
            if (metering_active = '1') then
                if (metering_ch_id = global_channel_count - 1) then
                    metering_active := '0';
                end if;
                metering_sample_reg <=
                    audio_in((SAMPLE_BITS * (metering_ch_id + 1) - 1) downto (SAMPLE_BITS * metering_ch_id));
                metering_ch_id_reg <= metering_ch_id;
                if byte_count = 0 then
                    metering_valid_reg <= '1';
                end if;
                metering_ch_id <= metering_ch_id + 1;
            end if;

            -- Stage 2: abs on the top bits only, compare against reduced thresholds
            if (metering_valid_reg = '1') then
                v_sample_top := unsigned(metering_sample_reg(SAMPLE_BITS - 2 downto SAMPLE_BITS - 1 - CMP_BITS));
                if metering_sample_reg(SAMPLE_BITS - 1) = '1' then
                    v_sample_top := not v_sample_top;
                end if;
                if v_sample_top >= CLIP_THRESHOLD then
                    metering_clip_o(metering_ch_id_reg) <= '1';
                end if;
                if v_sample_top >= SIGNAL_THRESHOLD then
                    metering_signal_o(metering_ch_id_reg) <= '1';
                end if;
            end if;
        end if;

    end process;
    end generate;


    metering_proc_disable_gen: if (ENABLE_METERING = false) generate
        metering_signal_o <= (others => '0');
        metering_clip_o <= (others=> '0');
    end generate;
        
    -- cdc process
    process (sys_clk, reset_n)
    begin
        if (reset_n = '0') then


            metering_clear_i_sync1 <= '0';
            metering_clear_i_sync2 <= '0';
            fs_clk_i_sync1 <= '0';
            fs_clk_i_sync2 <= '0';
            zaudio_sync <= '0';
        elsif (rising_edge(sys_clk)) then
            metering_clear_i_sync1 <= metering_clear_i;
            metering_clear_i_sync2 <= metering_clear_i_sync1;
            fs_clk_i_sync1 <= fs_clk_i;
            fs_clk_i_sync2 <= fs_clk_i_sync1;
            zaudio_sync <= fs_clk_i_sync2;
        end if;

    end process;

    process(sys_clk, reset_n)
    begin
        if reset_n = '0' then
            wr_ptr_o <= (others => '0');
            sample_wr_ptr <= 0;

            current_channel_id <= 0;
            write_active <= '0';
            byte_count <= 0;
            wr_ready_o <= '0';
            wr_ready_pending <= '0';
            ram_wr_en <= '0';
            ram_wr_data <= (others => '0');
            ram_wr_addr <= 0;
        elsif rising_edge(sys_clk) then
            
            -- Delay wr_ready by 1 clock: wr_ready_pending -> wr_ready_o
            -- This ensures wr_ptr_o is stable when downstream logic reads it
            wr_ready_o <= wr_ready_pending;
            wr_ready_pending <= '0';
            ram_wr_en <= '0';



            if (zaudio_sync = '0' and fs_clk_i_sync2 = '1') then
                write_active <= '1';
            end if;
            if write_active = '1' then

                
                ram_wr_data <= audio_in((SAMPLE_BITS * (current_channel_id) + byte_count * 8 + 7)  downto (SAMPLE_BITS * current_channel_id ) + byte_count *8 );
                ram_wr_addr <= sample_wr_ptr + current_channel_id * bytes_per_sample + byte_count;
                ram_wr_en <= '1';
                -- sample_ram(sample_wr_ptr + current_channel_id * bytes_per_sample + byte_count) <= data_latch(byte_count*8 + 7 downto byte_count*8);
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
            -- sample_ram(to_integer(read0Addr)) <= (others => '0');
        end if;
    end process;

    process (sys_clk)
    begin
        if (rising_edge(sys_clk)) then
            if (ram_wr_en = '1') then
                sample_ram(ram_wr_addr) <= ram_wr_data;
            end if;
        end if;

    end process;
    
end Behavioral;