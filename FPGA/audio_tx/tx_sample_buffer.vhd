library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity tx_sample_buffer is
    generic
    (
        samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3; -- wire bytes per sample (RTP L24 = 3). Internal RAM slot is 4.
        ENABLE_METERING: boolean := true;

        -- TDM serial input frontend (symmetric to rx_ringbuffer's TDM output).
        -- TDM_INPUT=false: legacy parallel audio_in bus.
        -- TDM_INPUT=true : serial TDM demux integrated here; audio_in unused.
        TDM_INPUT  : boolean := false;
        TDM_INPUTS : integer := 2;  -- number of serial TDM data pins
        TDM_CHANNELS : integer := 8; -- channels per TDM frame (32 bclk slots each)

        -- Simulation-only backdoor into sample_ram (see rx_ringbuffer). Disabled
        -- / optimized away in synthesis.
        SIM_SAMPLE_RAM_BACKDOOR : boolean := false
    );
	port
	(
        sys_clk                    : in std_logic;
        reset_n                    : in std_logic;


        audio_in                 : in STD_LOGIC_VECTOR((bytes_per_sample * 8) * global_channel_count - 1 downto 0) := (others => '0');
		fs_clk_i      				: in std_logic;
        bclk_sync_i                 : in std_logic := '0'; -- TDM bit clock (only used when TDM_INPUT=true)
        tdm_in                      : in std_logic_vector(TDM_INPUTS - 1 downto 0) := (others => '0');

		wr_ptr_o					: out std_logic_vector(15 downto 0) := (others => '0'); -- slot-octet
        wr_ready_o					: out std_logic := '0';

        read0Addr		: in unsigned(15 downto 0);
		data0_out		: out std_logic_vector(7 downto 0); -- 8 bit

        metering_signal_o : out std_logic_vector(global_channel_count - 1 downto 0);
        metering_clip_o : out std_logic_vector(global_channel_count - 1 downto 0);
        metering_clear_i : in std_logic;

        -- ===== Simulation backdoor into sample_ram (see SIM_SAMPLE_RAM_BACKDOOR) =====
        dbg_wr_en_i   : in  std_logic := '0';
        dbg_wr_addr_i : in  unsigned(15 downto 0) := (others => '0');
        dbg_wr_data_i : in  std_logic_vector(7 downto 0) := (others => '0');
        dbg_rd_addr_i : in  unsigned(15 downto 0) := (others => '0');
        dbg_rd_data_o : out std_logic_vector(7 downto 0)
	);
end entity;

architecture Behavioral of tx_sample_buffer is
    -- ceil(log2(n)) (same helper as rx_ringbuffer)
    function clog2(n : positive) return natural is
        variable result : natural := 0;
        variable val    : natural := n - 1;
    begin
        while val > 0 loop
            result := result + 1;
            val := val / 2;
        end loop;
        return result;
    end function;

    -- Internal RAM slot = 4 bytes per sample (pad 24-bit -> 32-bit). Channel and
    -- sample strides become powers of two, so the *3 address multiplies are gone.
    constant SLOT_BYTES   : integer := 4;
    constant CHANNEL_STRIDE : integer := SLOT_BYTES;                       -- 4
    constant SAMPLE_STRIDE  : integer := global_channel_count * SLOT_BYTES; -- ch * 4

    -- Total RAM = double-buffered (x2), in slot bytes.
    constant AUDIO_BUFFER_LENGTH : integer := samples_per_channel_depth * global_channel_count * SLOT_BYTES * 2;

    type t_sample_ram is array (0 to AUDIO_BUFFER_LENGTH - 1) of std_logic_vector(7 downto 0);
   	signal sample_ram		: t_sample_ram := (others => (others => '0'));
    signal sample_wr_ptr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;

    signal zaudio_sync : std_logic := '0';
    signal zbclk : std_logic := '0';
    signal current_channel_id : integer range 0 to global_channel_count - 1 := 0;
    signal write_active : std_logic := '0';
    signal byte_count : integer range 0 to bytes_per_sample - 1 := 0;

    -- Snapshot of audio_in taken atomically on the fs edge (parallel path only).
    signal audio_in_latched : std_logic_vector(audio_in'range) := (others => '0');

    -- Common RAM write port (driven by whichever input frontend is active)
    signal ram_wr_en : std_logic := '0';
    signal ram_wr_data : std_logic_vector(7 downto 0) := (others => '0');
    signal ram_wr_addr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal wr_ready_pending : std_logic := '0';

    -- Metering CDC
    signal metering_clear_i_sync1 : std_logic := '0';
    signal metering_clear_i_sync2 : std_logic := '0';
    signal metering_clear_last : std_logic := '0';

    constant SAMPLE_BITS : integer := bytes_per_sample * 8;
    constant CMP_BITS : integer := 9;
    constant CLIP_THRESHOLD   : unsigned(CMP_BITS - 1 downto 0) := to_unsigned(16#064#, CMP_BITS);
    constant SIGNAL_THRESHOLD : unsigned(CMP_BITS - 1 downto 0) := to_unsigned(16#001#, CMP_BITS);
    signal metering_ch_id  : integer range 0 to global_channel_count - 1;
    signal metering_sample_reg : std_logic_vector(SAMPLE_BITS - 1 downto 0) := (others => '0');
    signal metering_ch_id_reg  : integer range 0 to global_channel_count - 1 := 0;
    signal metering_valid_reg  : std_logic := '0';

    -- ===== TDM demux frontend signals (TDM_INPUT=true) =====
    -- bit/slot counters, clocked on bclk edges, reset on fs edge.
    signal tdm_bit_counter     : unsigned(4 downto 0) := (others => '0'); -- 0..31 within a slot
    signal tdm_channel_counter : unsigned(clog2(TDM_CHANNELS) - 1 downto 0) := (others => '0');
    -- one shift register per serial pin, capturing the top SAMPLE_BITS of each slot
    type t_tdm_shift is array (0 to TDM_INPUTS - 1) of std_logic_vector(SAMPLE_BITS - 1 downto 0);
    signal tdm_shift : t_tdm_shift := (others => (others => '0'));
    -- pulse: a full 32-bit slot just finished -> commit shift regs to RAM
    signal tdm_slot_done   : std_logic := '0';
    signal tdm_frame_start : std_logic := '0';
    -- demux write FSM
    type t_demux_state is (ds_idle, ds_write);
    signal demux_state   : t_demux_state := ds_idle;
    signal demux_pin     : integer range 0 to TDM_INPUTS := 0;
    signal demux_byte    : integer range 0 to bytes_per_sample - 1 := 0;
    signal demux_channel : integer range 0 to TDM_CHANNELS - 1 := 0;
    signal demux_slot_data : t_tdm_shift := (others => (others => '0'));
    signal demux_wr_ptr  : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal demux_metering_sample : std_logic_vector(SAMPLE_BITS - 1 downto 0) := (others => '0');
    signal demux_metering_ch     : integer range 0 to global_channel_count - 1 := 0;
    signal demux_metering_valid  : std_logic := '0';
begin

    -- =========================================================================
    -- fs / bclk edge detection.
    -- fs_clk_i and bclk_sync_i are already sys_clk-synchronous (generated in the
    -- sys_clk domain by audioclock_generator_sysclk), so NO multi-FF CDC is
    -- needed here -- a single delay register for edge detection is enough. This
    -- matches the rx_ringbuffer timing exactly (1 sys_clk fs/bclk latency), so
    -- in a serial loopback both sides start their frame on the same cycle.
    -- (metering_clear_i does cross from the ctrl-plane domain, so it keeps its
    -- 2-FF synchronizer.)
    -- zaudio_sync = fs delayed by 1, zbclk = bclk delayed by 1.
    process (sys_clk, reset_n)
    begin
        if (reset_n = '0') then
            metering_clear_i_sync1 <= '0';
            metering_clear_i_sync2 <= '0';
            zaudio_sync <= '0';
            zbclk <= '0';
        elsif (rising_edge(sys_clk)) then
            metering_clear_i_sync1 <= metering_clear_i;
            metering_clear_i_sync2 <= metering_clear_i_sync1;
            zaudio_sync <= fs_clk_i;
            zbclk <= bclk_sync_i;
        end if;
    end process;

    -- =========================================================================
    -- PARALLEL INPUT FRONTEND (TDM_INPUT = false)
    -- =========================================================================
    parallel_in_gen: if (TDM_INPUT = false) generate

        -- metering process (operates on audio_in)
        metering_proc_gen: if (ENABLE_METERING = true) generate
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
                if (zaudio_sync = '0' and fs_clk_i = '1') then
                    metering_active := '1';
                end if;
                if (metering_clear_i_sync2 /= metering_clear_last) then
                    metering_clear_last <= metering_clear_i_sync2;
                    metering_clip_o <= (others => '0');
                    metering_signal_o <= (others => '0');
                end if;
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

        -- Capture FSM: parallel audio_in -> RAM with 4-byte slots.
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
                audio_in_latched <= (others => '0');
            elsif rising_edge(sys_clk) then
                wr_ready_o <= wr_ready_pending;
                wr_ready_pending <= '0';
                ram_wr_en <= '0';

                if (zaudio_sync = '0' and fs_clk_i = '1') then
                    audio_in_latched <= audio_in;
                    write_active <= '1';
                end if;
                if write_active = '1' then
                    -- Linear slot layout: MSB @ slot offset 0, mid @ +1, LSB @ +2,
                    -- pad @ +3 (left 0). audio_in's per-channel slice has the MSB in
                    -- its top 8 bits, so byte_count=0 selects the top 8 bits and the
                    -- address increments -> MSB lands at offset 0. This matches the
                    -- TDM frontend and the transmitter's linear read order.
                    ram_wr_data <= audio_in_latched((SAMPLE_BITS * current_channel_id) + (SAMPLE_BITS - 1 - byte_count*8)
                                                    downto (SAMPLE_BITS * current_channel_id) + (SAMPLE_BITS - 8 - byte_count*8));
                    ram_wr_addr <= sample_wr_ptr + current_channel_id * CHANNEL_STRIDE + byte_count;
                    ram_wr_en <= '1';
                    if byte_count = bytes_per_sample - 1 then
                        byte_count <= 0;
                        if current_channel_id = global_channel_count - 1 then
                            current_channel_id <= 0;
                            if sample_wr_ptr + SAMPLE_STRIDE >= AUDIO_BUFFER_LENGTH then
                                sample_wr_ptr <= sample_wr_ptr + SAMPLE_STRIDE - AUDIO_BUFFER_LENGTH;
                                wr_ptr_o <= std_logic_vector(to_unsigned(sample_wr_ptr + SAMPLE_STRIDE - AUDIO_BUFFER_LENGTH, 16));
                            else
                                sample_wr_ptr <= sample_wr_ptr + SAMPLE_STRIDE;
                                wr_ptr_o <= std_logic_vector(to_unsigned(sample_wr_ptr + SAMPLE_STRIDE, 16));
                            end if;
                            write_active <= '0';
                            wr_ready_pending <= '1';
                        else
                            current_channel_id <= current_channel_id + 1;
                        end if;
                    else
                        byte_count <= byte_count + 1;
                    end if;
                end if;
            end if;
        end process;

    end generate;

    -- =========================================================================
    -- TDM SERIAL INPUT FRONTEND (TDM_INPUT = true)
    -- Inverse of rx_ringbuffer's serial TDM output: shift bits in MSB-first,
    -- commit one channel slot every 32 bclk, write the demuxed bytes into RAM
    -- in linear order MSB@+0, mid@+1, LSB@+2, pad@+3.
    -- =========================================================================
    tdm_in_gen: if (TDM_INPUT = true) generate

        -- ----- bclk-domain (sampled into sys_clk via edge detect) bit/slot counters -----
        tdm_ctrl_proc: process(sys_clk, reset_n)
        begin
            if reset_n = '0' then
                tdm_bit_counter <= (others => '0');
                tdm_channel_counter <= (others => '0');
                tdm_slot_done <= '0';
                tdm_frame_start <= '0';
                tdm_shift <= (others => (others => '0'));
            elsif rising_edge(sys_clk) then
                tdm_slot_done   <= '0';
                tdm_frame_start <= '0';

                -- bclk rising edge: sample one bit per pin
                if (bclk_sync_i = '1' and zbclk = '0') then
                    -- shift MSB-first; only the top SAMPLE_BITS of the 32-bit slot
                    -- are kept (the rest are padding/ignored).
                    for p in 0 to TDM_INPUTS - 1 loop
                        if tdm_bit_counter < to_unsigned(SAMPLE_BITS, 5) then
                            tdm_shift(p) <= tdm_shift(p)(SAMPLE_BITS - 2 downto 0) & tdm_in(p);
                        end if;
                    end loop;

                    if tdm_bit_counter = to_unsigned(31, 5) then
                        -- slot complete: commit and advance channel
                        tdm_slot_done <= '1';
                        tdm_bit_counter <= (others => '0');
                        if tdm_channel_counter = to_unsigned(TDM_CHANNELS - 1, tdm_channel_counter'length) then
                            tdm_channel_counter <= (others => '0');
                        else
                            tdm_channel_counter <= tdm_channel_counter + 1;
                        end if;
                    else
                        tdm_bit_counter <= tdm_bit_counter + 1;
                    end if;
                end if;

                -- frame sync: restart at channel 0 / bit 0
                if (fs_clk_i = '1' and zaudio_sync = '0') then
                    tdm_bit_counter <= (others => '0');
                    tdm_channel_counter <= (others => '0');
                    tdm_frame_start <= '1';
                end if;
            end if;
        end process;

        -- ----- write FSM: on each slot_done, push the TDM_INPUTS captured slots
        -- into RAM. Pin p carries channels [p*TDM_CHANNELS .. p*TDM_CHANNELS+TC-1].
        -- The slot just finished belongs to channel (tdm_channel_counter - 1) since
        -- the counter already advanced; we latch the finishing channel index. -----
        tdm_write_proc: process(sys_clk, reset_n)
            variable v_ch_global : integer range 0 to global_channel_count - 1;
            variable v_finished_ch : integer range 0 to TDM_CHANNELS - 1;
        begin
            if reset_n = '0' then
                demux_state <= ds_idle;
                demux_pin <= 0;
                demux_byte <= 0;
                demux_channel <= 0;
                demux_slot_data <= (others => (others => '0'));
                demux_wr_ptr <= 0;
                ram_wr_en <= '0';
                ram_wr_data <= (others => '0');
                ram_wr_addr <= 0;
                wr_ptr_o <= (others => '0');
                wr_ready_o <= '0';
                wr_ready_pending <= '0';
                demux_metering_valid <= '0';
            elsif rising_edge(sys_clk) then
                ram_wr_en <= '0';
                wr_ready_o <= wr_ready_pending;
                wr_ready_pending <= '0';
                demux_metering_valid <= '0';

                -- A new frame begins: reset write pointer to the start of the
                -- inactive half is NOT done here; we keep a free-running ring
                -- pointer just like the parallel path. Frame start only matters
                -- for channel alignment (handled by the ctrl counters).

                case demux_state is
                    when ds_idle =>
                        if tdm_slot_done = '1' then
                            -- channel that just finished (counter already advanced)
                            if tdm_channel_counter = to_unsigned(0, tdm_channel_counter'length) then
                                v_finished_ch := TDM_CHANNELS - 1;
                            else
                                v_finished_ch := to_integer(tdm_channel_counter) - 1;
                            end if;
                            demux_channel <= v_finished_ch;
                            demux_slot_data <= tdm_shift;
                            demux_pin <= 0;
                            demux_byte <= 0;
                            -- present metering sample for pin 0's channel
                            demux_metering_sample <= tdm_shift(0);
                            demux_metering_ch <= 0 * TDM_CHANNELS + v_finished_ch;
                            demux_metering_valid <= '1';
                            demux_state <= ds_write;
                        end if;

                    when ds_write =>
                        -- write byte demux_byte of pin demux_pin's channel.
                        -- linear layout: MSB@+0, mid@+1, LSB@+2.
                        v_ch_global := demux_pin * TDM_CHANNELS + demux_channel;
                        ram_wr_addr <= demux_wr_ptr + v_ch_global * CHANNEL_STRIDE + demux_byte;
                        -- byte 0 = MSB = top 8 bits of the captured word
                        ram_wr_data <= demux_slot_data(demux_pin)(SAMPLE_BITS - 1 - demux_byte*8
                                                                   downto SAMPLE_BITS - 8 - demux_byte*8);
                        ram_wr_en <= '1';

                        if demux_byte = bytes_per_sample - 1 then
                            demux_byte <= 0;
                            if demux_pin = TDM_INPUTS - 1 then
                                -- all pins for this channel written
                                demux_state <= ds_idle;
                                -- when the last channel of the frame is committed,
                                -- advance the ring write pointer by one sample and
                                -- signal wr_ready (one full multichannel sample done).
                                if demux_channel = TDM_CHANNELS - 1 then
                                    if demux_wr_ptr + SAMPLE_STRIDE >= AUDIO_BUFFER_LENGTH then
                                        demux_wr_ptr <= demux_wr_ptr + SAMPLE_STRIDE - AUDIO_BUFFER_LENGTH;
                                        wr_ptr_o <= std_logic_vector(to_unsigned(demux_wr_ptr + SAMPLE_STRIDE - AUDIO_BUFFER_LENGTH, 16));
                                    else
                                        demux_wr_ptr <= demux_wr_ptr + SAMPLE_STRIDE;
                                        wr_ptr_o <= std_logic_vector(to_unsigned(demux_wr_ptr + SAMPLE_STRIDE, 16));
                                    end if;
                                    wr_ready_pending <= '1';
                                end if;
                            else
                                demux_pin <= demux_pin + 1;
                                -- present metering sample for next pin's channel
                                demux_metering_sample <= demux_slot_data(demux_pin + 1);
                                demux_metering_ch <= (demux_pin + 1) * TDM_CHANNELS + demux_channel;
                                demux_metering_valid <= '1';
                            end if;
                        else
                            demux_byte <= demux_byte + 1;
                        end if;
                end case;
            end if;
        end process;

        -- metering for TDM path: consume demux_metering_* one-shot samples
        metering_tdm_gen: if (ENABLE_METERING = true) generate
        process(sys_clk, reset_n)
            variable v_sample_top : unsigned(CMP_BITS - 1 downto 0);
        begin
            if reset_n = '0' then
                metering_clear_last <= '0';
            elsif rising_edge(sys_clk) then
                if (metering_clear_i_sync2 /= metering_clear_last) then
                    metering_clear_last <= metering_clear_i_sync2;
                    metering_clip_o <= (others => '0');
                    metering_signal_o <= (others => '0');
                end if;
                if demux_metering_valid = '1' then
                    v_sample_top := unsigned(demux_metering_sample(SAMPLE_BITS - 2 downto SAMPLE_BITS - 1 - CMP_BITS));
                    if demux_metering_sample(SAMPLE_BITS - 1) = '1' then
                        v_sample_top := not v_sample_top;
                    end if;
                    if v_sample_top >= CLIP_THRESHOLD then
                        metering_clip_o(demux_metering_ch) <= '1';
                    end if;
                    if v_sample_top >= SIGNAL_THRESHOLD then
                        metering_signal_o(demux_metering_ch) <= '1';
                    end if;
                end if;
            end if;
        end process;
        end generate;

        metering_tdm_disable_gen: if (ENABLE_METERING = false) generate
            metering_signal_o <= (others => '0');
            metering_clip_o <= (others => '0');
        end generate;

    end generate;

    -- =========================================================================
    -- Sample RAM: registered read (block RAM), registered write + sim backdoor
    -- =========================================================================
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            data0_out <= sample_ram(to_integer(read0Addr));
        end if;
    end process;

    process (sys_clk)
    begin
        if (rising_edge(sys_clk)) then
            if (ram_wr_en = '1') then
                sample_ram(ram_wr_addr) <= ram_wr_data;
            end if;
            -- Simulation backdoor write (testbench only); takes priority.
            if SIM_SAMPLE_RAM_BACKDOOR and dbg_wr_en_i = '1' then
                sample_ram(to_integer(dbg_wr_addr_i)) <= dbg_wr_data_i;
            end if;
        end if;
    end process;

    -- Simulation backdoor read port (testbench only).
    sim_backdoor_rd_gen : if SIM_SAMPLE_RAM_BACKDOOR generate
        process(sys_clk)
        begin
            if rising_edge(sys_clk) then
                dbg_rd_data_o <= sample_ram(to_integer(dbg_rd_addr_i));
            end if;
        end process;
    end generate;
    sim_backdoor_rd_off_gen : if not SIM_SAMPLE_RAM_BACKDOOR generate
        dbg_rd_data_o <= (others => '0');
    end generate;

end Behavioral;
