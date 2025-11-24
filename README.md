CAN NODE DESIGN CODE


library IEEE;
use IEEE.STD_LOGIC_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
library work;
use work.xyz.all;


ENTITY CAN_Node IS

PORT(
    clk			:in std_logic;		 --system clock (10Mhz)
    data_clk		:in std_logic;		 --1 MHz data clock
    reset			:in std_logic;		 --active low reset
    tx_start		:in std_logic;
    tx_enable1		:in std_logic:='0'; 	--indicates node is transmitting
    tx_enable2		:in std_logic:='0'; 
    tx_enable3		:in std_logic:='0'; 
    tx_enable4		:in std_logic:='0'; 
    rx_enable		:in std_logic;
    trigger		:inout std_logic:='1';	 --trigger signal to start transmission
    trigger_rx 		:inout std_logic:='0'; 	-- trigger to start receiving data
    trigger_proc		:inout std_logic:='0'; 	-- trigger to start data processing
    flag            		:out std_logic:='0';
    out_port        	:out std_logic_vector(63 downto 0):=(others=>'1')
);
end CAN_Node;


ARCHITECTURE Behavioural of CAN_Node IS
   
    type state_type1 is(IDLE,START_OF_FRAME,ARBITRATION,ID_FIELD,RTR,RECEIVER,DE_CRC,CONTROL,DATA,CRC,ACK,END_OF_FRAME); --needed states
    type state_type2 is(IDLE,TRANSMITTER,END_OF_TX);
    type state_type3 is(IDLE,RECEIVER,END_OF_RX);
    type state_type4 is(IDLE,CHECK,ACCEPT_DATA,REJECT_DATA);
    signal state1       :state_type1:=IDLE;
    signal state2       :state_type2:=IDLE;
    signal state3       :state_type3:=IDLE;
    signal state4       :state_type4:=IDLE;
    signal bit_counter  :integer:=0;
    signal data_reg     :std_logic_vector(63 downto 0):="1010110110101101101011011010110110101101101011011010110110101101";   
 --stores data of node
    signal crc_reg      :std_logic_vector(15 downto 0):=(others=>'0');
    signal tx_bit       :std_logic :='0';
    signal rx_bit       :std_logic :='0';
    signal tx_enable_reg:std_logic :='1';
    signal rx_enable_reg:std_logic :='1';
    signal dlc_reg      :std_logic_vector(3 downto 0) :="1000";
    signal data_in      :std_logic_vector(81 downto 0);
    signal crc_in       :std_logic_vector(15 downto 0);
    signal can_bus      :std_logic_vector(23 downto 0);
    signal id1		    :STD_LOGIC_VECTOR(10 DOWNTO 0):="00000000010"; 
--identifier for nodes
    signal id2		    :STD_LOGIC_VECTOR(10 DOWNTO 0):="11111111111";
    signal id3		    :STD_LOGIC_VECTOR(10 DOWNTO 0):="11111111111";
    signal id4		    :STD_LOGIC_VECTOR(10 DOWNTO 0):="11111111111";
    signal id_reg       :std_logic_vector(10 downto 0):=id1;  --stores id value of node
    signal rx_id        :std_logic_vector(10 downto 0):=(others=>'1'); 
    signal rx_rtr       :std_logic:='1';
    signal rx_dlc       :std_logic_vector(3 downto 0):=(others=>'1');
    signal rx_data      :std_logic_vector(63 downto 0):=(others=>'1');
        
    signal tx_busy      :std_logic:='0';
    signal rx_busy      :std_logic:='0';
    signal tx_reg       :std_logic_vector(113 downto 0):=(others=>'0'); --transmitter register
    signal rx_reg       :std_logic_vector(116 downto 0):=(others=>'0'); --receiver register
    signal cbus_bit     :std_logic:='0'; --data stored in can bus

    signal count1:integer:=113;
    signal count2:integer:=116;

--CRC calculation fxn
    function calculate_crc15(
        data_in :std_logic_vector(81 downto 0);
        crc_in  :std_logic_vector(15 downto 0) )
    return std_logic_vector is
        variable crc   :std_logic_vector(15 downto 0);
        --variable data_var :std_logic_vector(81 downto 0);
    begin
        crc:=crc_in;
        for i in 0 to data_in'length-1 loop
            if ((crc(15) xor data_in(i)) ='1')then
                crc:=crc(14 downto 0)&'0';
                crc(15 downto 0):=crc(15 downto 0) xor "1100010110011001"; --0xC599
            else
                crc:=crc(14 downto 0)&'0';
            end if;
        end loop;
    return crc;
    end function;

------Arbitration function---------- 
   --function arbitration(
        --id1:std_logic_vector(10 downto 0);
        --id2:std_logic_vector(10 downto 0);
        --id3:std_logic_vector(10 downto 0);
        --id4:std_logic_vector(10 downto 0)
    --)
    --return std_logic_vector is
        --variable id_out1:std_logic_vector(10 downto 0);
        --variable id_out2:std_logic_vector(10 downto 0);
        --variable id_out3:std_logic_vector(10 downto 0);
        --variable id_out4:std_logic_vector(10 downto 0);
        --variable cbus_id:std_logic_vector(10 downto 0);
        ----register temp::std_logic_vector(10 downto 0);
        --variable i,j:integer:=0;
        ----signal node_filter_id   :std_logic_vector(10 downto 0);
--
   --begin
        --
        --id_out1:=id1;
        --id_out2:=id2;
        --id_out3:=id3;
        --id_out4:=id4;
--
   --for i in 0 to 10 loop
            --cbus_id(i) := id_out1(i) AND id_out2(i) AND id_out3(i) AND id_out4(i);  
            --if cbus_id(i) /= id_out1(i) then
                --id_out1(10 downto i+1):=(others=>'1');
                ----FOR j IN i+1 TO 10 LOOP
					----id_out1(j):='1';
				----END LOOP;
            --end if;
			--IF cbus_id(i)/=id_out2(i) THEN
                --id_out2(10 downto i+1):=(others=>'1');
				----FOR j IN i+1 TO 10 LOOP
					----id_out2(j):='1';
				----END LOOP;
            --end if;
			--IF cbus_id(i)/=id_out3(i) THEN
                --id_out3(10 downto i+1):=(others=>'1');
				----FOR j IN i+1 TO 10 LOOP
					----id_out3(j):='1';
				----END LOOP;
            --end if;
			--IF cbus_id(i)/=id_out4(i) THEN
                --id_out4(10 downto i+1):=(others=>'1');
				----FOR j IN i+1 TO 10 LOOP
					----id_out4(j):='1';
				----END LOOP;
            --end if;
            ----temp=cbus_id(1);
   --end loop;
        ----return cbus_id;
        --if cbus_id = id1 then   
            --return id1;
        --elsif cbus_id = id2 then
            --return id2;
        --elsif cbus_id = id3 then
            --return id3;
        --elsif cbus_id = id4 then
            --return id4;
        --else
            --return "11111111111"; --indicate error
        --end if;
    --end arbitration ;


begin
----------Data frame block---------------
    process(clk,reset,tx_start)
    begin
        if reset ='1' then
            state1<=IDLE;
            bit_counter<=0;
        elsif rising_edge(clk) then
            case state1 is
                when IDLE =>
                    tx_enable_reg<='1';
                    if tx_start='1'then
                        state1<=START_OF_FRAME;
                    else
                        state1<=IDLE;
                    end if;

                when START_OF_FRAME=>                   
                    state1<=ID_FIELD;

                when ARBITRATION=>
                    id_reg<=id1;
               
                when ID_FIELD =>                      
                    tx_reg(113 downto 103)<=id_reg;
                    state1<=RTR;

                when RTR =>     --Detection of remote frame and data frames.
                    tx_reg(102)<='0';
                    state1<= CONTROL;
   
                when CONTROL =>
                    tx_reg(101)<='0'; --IDE bit
                    tx_reg(100)<='0'; --r0 bit
                    tx_reg(99 downto 96)<="1000"; --DLC=64 bits=8 bytes
                    state1<= DATA;

                when DATA =>    --data reg contains data received from top module                               
                    tx_reg(95 downto 32)<=data_reg;     
                    state1<=CRC;
                    
               when CRC =>      --error detection function. crc15 polynomial used
                    crc_reg<=calculate_crc15(tx_reg(81 downto 0),"1111111111111111");
                    state1<=ACK;

                when ACK =>     --if ack is error free any node can indicate to the transmitter.
                    tx_reg(31 downto 16)<=crc_reg;
                    tx_reg(15 downto 14)<="11";
                    state1<= END_OF_FRAME;

                when END_OF_FRAME =>
                    tx_reg(13 downto 7)<="1111111"; --eof
                    tx_reg(6 downto 0)<="1111111"; --ifs
                    tx_enable_reg <='0';
                    rx_enable_reg<='0';


                when others =>
                    state1<=IDLE;
            end case;
        end if;
    end process;


--------Transmitter block----------
process(data_clk)
begin
    trigger<='1'; --user input
    if rising_edge(data_clk)then
        case state2 is
            when IDLE=>
                if(trigger='1' and tx_enable_reg='0')then --trigger = 1 will start the actual transmission bit by bit
                    tx_bit<='0';    --sof bit = 0 transmitted
                    tx_busy<='1';
                    state2<=TRANSMITTER;
                else 
                    tx_busy<='0';
                    state2<=IDLE;
                end if;
        
            when TRANSMITTER=>
                tx_bit<=tx_reg(count1);
                cbus_bit<=tx_reg(count1);
                if(count1=0)then
                    state2<=END_OF_TX;  --end of tx
                else 
                    count1<=count1-1;
                    state2<=TRANSMITTER;
                end if;

            when END_OF_TX=>
                tx_busy<='0';
                tx_bit<='1';

            when others=>
                state2<=IDLE;
        end case;
    end if;
end process;


-------Receiver block--------
process(data_clk)
begin
    trigger_rx<='1'; --user input
    if rising_edge(data_clk) then
        case state3 is
            when IDLE=>
                if(trigger_rx='1') and (rx_enable_reg='0') then
                    rx_busy<='1';
                    rx_reg(116)<='0';  --sof bit
                    count2<=115;
                    state3<=RECEIVER;
                else
                    state3<=IDLE;
                end if;                                                                                                                                                                                                                                                                                                                  
    
            when RECEIVER=>
                rx_bit<=cbus_bit;
                rx_reg(count2)<=rx_bit;
                if(count2=0)then
                    state3<=END_OF_RX;  -- end of rx
                else
                    count2<=count2-1;
                    state3<=RECEIVER;
                end if;
        
            when END_OF_RX=>
                rx_busy<='0';
                trigger_proc<='1';

            when others=>
                state3<=IDLE;
        end case;
    end if;
end process;
                

-------Data Processing block---------
process(clk,reset,trigger_proc)
begin
    if reset='1'then
        state4<=IDLE;
        
    elsif rising_edge(clk) and rx_enable='1' then
        case state4 is
            when IDLE=>
                if (rx_bit<='1' and trigger_proc='1') then
                    state4<=CHECK;
                else
                    state4<=IDLE;    
                end if;
            when CHECK=>
                if(tx_reg(113 downto 103)=rx_reg(113 downto 103))--id value
                  and(tx_reg(99 downto 96)=rx_reg(99 downto 96))--dlc
                  and(tx_reg(102)=rx_reg(102))then--rtr
                        state4<=ACCEPT_DATA;
                else
                        state4<=REJECT_DATA;
                end if;
                        
            when ACCEPT_DATA=>
                flag<='1';
                rx_id  <= rx_reg(113 downto 103);
                rx_rtr <= rx_reg(102);
                rx_dlc <= rx_reg(99 downto 96);
                rx_data <= rx_reg(95 downto 32);
                out_port <= rx_data;
            
            when REJECT_DATA=>
                 flag<='0';
                 out_port <= (others => '1');
        end case;            
    end if;
end process;

--- end of data processing ---
    
end Behavioural;


— The above code consists of CAN node data frame states, transmitter logic and receiver logic.CAN NODE. TESTBENCH CODE

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


entity CAN_Node_tb is
end CAN_Node_tb;

architecture behavioral of CAN_Node_tb is

    constant clk_period : time := 100 ns; -- 10MHZ
    constant data_clk_period : time := 1000 ns; --1MgHz

    signal clk : std_logic := '0';
    signal data_clk : std_logic := '0';
    signal reset : std_logic := '0';

-------------------------------------------------------------
    type state_type1 is(IDLE,START_OF_FRAME,ARBITRATION,ID_FIELD,RTR,RECEIVER,DE_CRC,CONTROL,DATA,CRC,ACK,END_OF_FRAME); --needed states
    type state_type2 is(IDLE,TRANSMITTER,END_OF_TX);
    type state_type3 is(IDLE,RECEIVER,END_OF_RX);
    type state_type4 is(IDLE,CHECK,ACCEPT_DATA,REJECT_DATA);
    signal state1       :state_type1:=IDLE;
    signal state2       :state_type2:=IDLE;
    signal state3       :state_type3:=IDLE;
    signal state4       :state_type4:=IDLE;
    signal id_reg       :std_logic_vector(10 downto 0);   --stores id value of node
    signal data_reg     :std_logic_vector(63 downto 0):="1010110110101101101011011010110110101101101011011010110110101101";    --stores data of node
    signal crc_reg      :std_logic_vector(15 downto 0):=(others=>'0');
    signal tx_bit       :std_logic :='0';
    signal rx_bit       :std_logic :='0';
    signal rx_id        :std_logic_vector(10 downto 0):=(others=>'1'); 
    signal rx_rtr       :std_logic:='1';
    signal rx_dlc       :std_logic_vector(3 downto 0):=(others=>'1');
    signal rx_data      :std_logic_vector(63 downto 0):=(others=>'1');
    signal tx_reg       :std_logic_vector(113 downto 0):=(others=>'0'); --transmitter register
    signal rx_reg       :std_logic_vector(116 downto 0):=(others=>'0'); --receiver register
    signal cbus_bit     :std_logic:='0'; --data stored in can bus
    signal count1       :integer:=113;
    signal count2       :integer:=114;
    signal tx_start     :std_logic;
    signal tx_enable1   :std_logic;
    signal tx_enable2   :std_logic;
    signal tx_enable3   :std_logic;
    signal tx_enable4   :std_logic;
    signal rx_enable    :std_logic;
    signal flag         :std_logic;
    signal out_port     :std_logic_vector(63 downto 0);
    signal trigger      :std_logic;
    signal trigger_rx   :std_logic;
    signal trigger_proc :std_logic;
    signal id1		    :std_logic_vector(10 downto 0):="00000000010"; --identifier for nodes
    signal id2		    :std_logic_vector(10 downto 0):="11111111111";
    signal id3		    :std_logic_vector(10 downto 0):="11111111111";
    signal id4		    :std_logic_vector(10 downto 0):="11111111111";


-------------------------------------------------------------

    component CAN_Node
        -- ports
        port( 
            -- Inputs
            clk : in std_logic;
            data_clk : in std_logic;
            reset : in std_logic;
            tx_start : in std_logic;
            tx_enable1 : in std_logic;
            tx_enable2 : in std_logic;
            tx_enable3 : in std_logic;
            tx_enable4 : in std_logic;
            rx_enable : in std_logic;

            -- Outputs
            flag : out std_logic;
            out_port : out std_logic_vector(63 downto 0);

            -- Inouts
            trigger : inout std_logic;
            trigger_rx : inout std_logic;
            trigger_proc : inout std_logic

        );

        
    end component;

begin
    uut:CAN_Node
        port map( 
            clk          => clk,
            data_clk     => data_clk ,
            reset        => reset ,
            tx_start     => tx_start ,
            tx_enable1   => tx_enable1,
            tx_enable2   => tx_enable2 ,
            tx_enable3   => tx_enable3 ,
            tx_enable4   => tx_enable4  ,
            rx_enable    => rx_enable ,
            flag         => flag ,
            out_port     => out_port ,
            trigger      => trigger ,
            trigger_rx   => trigger_rx ,
            trigger_proc => trigger_proc 
    );
    process
        variable vhdl_initial : BOOLEAN := TRUE;

    begin
        if ( vhdl_initial ) then
            -- Assert Reset
            reset <= '1';
            wait for 100 ns;
            reset <= '0';
            wait;
        end if;

        
    end process;

    -- Clock Driver
    clk <= not clk after (clk_period / 2.0 );
    data_clk <= not data_clk after (data_clk_period / 2.0 );

    --stimulus
stim_proc:process
    begin
    --reset<='0';
    tx_start    <= '1';
    tx_enable1  <='1';
    tx_enable2  <='0';
    tx_enable3  <='0';
    tx_enable4  <='0';
    rx_enable   <='1';
    trigger     <='1';
    trigger_rx  <='1';
--    trigger_proc<='1';
    --id_reg<="00000000010";
    wait;
end process;

end behavioral;
