library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
 
entity urt_dbg is
    port(
        clk                 : in  std_logic;
        reset_n             : in  std_logic;
 
        urt_tx_data_o       : out std_logic_vector(7 downto 0);
        urt_tx_valid_o      : out std_logic;
        urt_tx_active_i      : in  std_logic;
        urt_tx_done_i       : in  std_logic;

        start_i          : in  std_logic;
        dbg_data1_i          : in  std_logic_vector(63 downto 0);
        dbg_data2_i          : in  std_logic_vector(63 downto 0)
    );
end entity;


architecture Behavioral of urt_dbg is

    type t_state is (s_Idle, s_Send_Data1, s_Wait1, s_Send_Data2, s_Wait2);
    signal state         : t_state := s_Idle;
    signal byte_counter  : unsigned(3 downto 0) := (others => '0');

    function get_byte(v : std_logic_vector(63 downto 0);
                      idx : unsigned(3 downto 0)) return std_logic_vector is
    begin
        case to_integer(idx) is
            when 0 => return v(63 downto 56);
            when 1 => return v(55 downto 48);
            when 2 => return v(47 downto 40);
            when 3 => return v(39 downto 32);
            when 4 => return v(31 downto 24);
            when 5 => return v(23 downto 16);
            when 6 => return v(15 downto 8);
            when 7 => return v(7 downto 0);
            when others => return (others => '0');
        end case;
    end function;
begin
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            state           <= s_Idle;
            byte_counter    <= (others => '0');
            urt_tx_data_o   <= (others => '0');
            urt_tx_valid_o  <= '0';

        elsif rising_edge(clk) then
            case state is
                when s_Idle =>
                    urt_tx_valid_o <= '0';
                    byte_counter   <= (others => '0');
                    if start_i = '1' then
                        state <= s_Send_Data1;
                    end if;

                when s_Send_Data1 =>
                    urt_tx_data_o  <= get_byte(dbg_data1_i, byte_counter);
                    urt_tx_valid_o <= '1';
                    if urt_tx_active_i = '1' then
                        state <= s_Wait1;
                    end if;

                when s_Wait1 =>
                    if urt_tx_active_i = '0' then
                        if byte_counter = "0111" then
                            byte_counter <= (others => '0');
                            state <= s_Send_Data2;
                        else
                            byte_counter <= byte_counter + 1;
                            state <= s_Send_Data1;
                        end if;
                    end if;

                when s_Send_Data2 =>
                    urt_tx_data_o  <= get_byte(dbg_data2_i, byte_counter);
                    urt_tx_valid_o <= '1';
                    if urt_tx_active_i = '1' then
                        state <= s_Wait2;
                    end if;

                when s_Wait2 =>
                    if urt_tx_active_i = '0' then
                        if byte_counter = "0111" then
                            byte_counter <= (others => '0');
                            state <= s_Idle;
                        else
                            byte_counter <= byte_counter + 1;
                            state <= s_Send_Data2;
                        end if;
                    end if;

                when others =>
                    state <= s_Idle;
            end case;
        end if;
    end process;

end architecture;
