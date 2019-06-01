library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity Robot is
  port ( main_clk				: in std_logic;				 
		Robot_ID				: in std_logic_vector(3 downto 0);
		
		LF_HALL_I           	: in std_logic_vector(2 downto 0);  --HALL(C) > HALL(B) > HALL(A)
		LF_MOSFET_O  			: out std_logic_vector(5 downto 0); --HIGH(C) > HIGH(B) > HIGH(A) > LOW(C) > LOW(B) > LOW(A)
		
		LB_HALL_I           	: in std_logic_vector(2 downto 0);  --HALL(C) > HALL(B) > HALL(A)
		LB_MOSFET_O  			: out std_logic_vector(5 downto 0); --HIGH(C) > HIGH(B) > HIGH(A) > LOW(C) > LOW(B) > LOW(A)
		
		RB_HALL_I           	: in std_logic_vector(2 downto 0);  --HALL(C) > HALL(B) > HALL(A)
		RB_MOSFET_O  			: out std_logic_vector(5 downto 0); --HIGH(C) > HIGH(B) > HIGH(A) > LOW(C) > LOW(B) > LOW(A)
		
		RF_HALL_I           	: in std_logic_vector(2 downto 0);  --HALL(C) > HALL(B) > HALL(A)
		RF_MOSFET_O  			: out std_logic_vector(5 downto 0); --HIGH(C) > HIGH(B) > HIGH(A) > LOW(C) > LOW(B) > LOW(A)
		
		a_clk0,b_clk0			: in std_logic;--LF
		a_clk1,b_clk1 			: in std_logic;--LB
		a_clk2,b_clk2 			: in std_logic;--RB
		a_clk3,b_clk3 			: in std_logic;--RF

		---------------------------------------------------SHOOT-----
		switch_pulse			: out std_logic;
		discharge_key0			: in std_logic:='0';
		discharge_key1			: in std_logic:='0';
		shoot_zart				: out std_logic;
		chip_zart				: out std_logic;
		adc						: IN std_logic;
		--------------------------------------------------SpinBack---
		sb,sb_fw,sb_break 		: out std_logic;
		------------------------------------------------------IR-----
		TX						: out std_logic:='0';
		RX						: in std_logic:='0';
		--------------------------------------------------LED/Buzzer----
		LED1						: out std_logic:='1';
		LED2						: out std_logic:='1';
		Buzzer					: out std_logic:='0';
		------------------------------------------------------SPI----
		mosi					: out std_logic;
		miso					: in std_logic;
		CE  					: out std_logic;
		CS						: out std_logic:='1';
		sck 					: out std_logic:='1';
		---------------------------------------------------Battery---
		BatteryIsFull			: in std_logic
		);
end entity;
architecture Behavior of Robot is

--    COMPONENT adc
--    port(
--	 clk     : in     std_logic;
--	 sda     : INOUT  std_logic;
--     scl     : INOUT  std_logic;
--     CH1 	 : out std_logic_vector(11 downto 0);
--     CH2 	 : out std_logic_vector(11 downto 0);
--     CH3 	 : out std_logic_vector(11 downto 0);
--     CH4 	 : out std_logic_vector(11 downto 0)
--		);
--	end component;

---------------------------------------------Delay's----------------
--[Delay Between Each Commutation Phase]--	[td=YourValuex33 ns]
constant Delay_LF		:integer:=2;
constant Delay_LB		:integer:=2;
constant Delay_RB		:integer:=2;
constant Delay_RF		:integer:=2;
--------------------------------------------Reverse Delay's---------
--[The Time Duration Between Each Motor Reverse Situation]--[tdR=YourValuex33 ns]
constant ReverseDelay0 	:integer:=666666;
constant ReverseDelay1 	:integer:=666666;
constant ReverseDelay2 	:integer:=666666;
constant ReverseDelay3 	:integer:=666666;
---------------------------------------------ADC Voltage------------
--[ADC Voltage Refrence's]--
constant V_16 : std_logic_vector(11 downto 0):=x"1F4";
constant V_230: std_logic_vector(11 downto 0):=x"8B6";
constant V_250: std_logic_vector(11 downto 0):=x"B54";
--------------------------------------------------------------------
signal LF_aa			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_bb			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_cc			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_dd			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_ee			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_ff			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_aaa			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_bbb			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_ccc			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_ddd			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_eee			:integer range 0 to Delay_LF+1:=Delay_LF+1;
signal LF_fff			:integer range 0 to Delay_LF+1:=Delay_LF+1;

signal LB_aa			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_bb			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_cc			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_dd			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_ee			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_ff			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_aaa			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_bbb			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_ccc			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_ddd			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_eee			:integer range 0 to Delay_LB+1:=Delay_LB+1;
signal LB_fff			:integer range 0 to Delay_LB+1:=Delay_LB+1;

signal RB_aa			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_bb			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_cc			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_dd			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_ee			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_ff			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_aaa			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_bbb			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_ccc			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_ddd			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_eee			:integer range 0 to Delay_RB+1:=Delay_RB+1;
signal RB_fff			:integer range 0 to Delay_RB+1:=Delay_RB+1;

signal RF_aa			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_bb			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_cc			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_dd			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_ee			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_ff			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_aaa			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_bbb			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_ccc			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_ddd			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_eee			:integer range 0 to Delay_RF+1:=Delay_RF+1;
signal RF_fff			:integer range 0 to Delay_RF+1:=Delay_RF+1;
-------------------------------------------------------------Direction
signal LF_direction 	:std_logic:='0';
signal LB_direction 	:std_logic:='0';
signal RB_direction 	:std_logic:='0';
signal RF_direction 	:std_logic:='0';
-------------------------------------------------------------PI Controller
signal kp0,kp1,kp2,kp3 	:integer:=0;
signal ki0,ki1,ki2,ki3 	:integer:=0;

signal setpoint0 		:integer range -128 to 127:=0;
signal setpoint1		:integer range -128 to 127:=0;
signal setpoint2 		:integer range -128 to 127:=0;
signal setpoint3 		:integer range -128 to 127:=0;

signal speed0 			:integer range -128 to 127:=0;
signal speed1 			:integer range -128 to 127:=0;
signal speed2 			:integer range -128 to 127:=0;
signal speed3 			:integer range -128 to 127:=0;

signal clkf01			:std_logic_vector(9 downto 0);
signal clkf02			:std_logic_vector(9 downto 0);
signal clkf11			:std_logic_vector(9 downto 0);
signal clkf12			:std_logic_vector(9 downto 0);
signal clkf21			:std_logic_vector(9 downto 0);
signal clkf22			:std_logic_vector(9 downto 0);
signal clkf31			:std_logic_vector(9 downto 0);
signal clkf32			:std_logic_vector(9 downto 0);

signal mp02				:std_logic;
signal mp01				:std_logic;
signal mp12				:std_logic;
signal mp11				:std_logic;
signal mp22				:std_logic;
signal mp21				:std_logic;
signal mp32				:std_logic;
signal mp31				:std_logic;

signal data01,data02 	:integer:=0;
signal data11,data12 	:integer:=0;
signal data21,data22 	:integer:=0;
signal data31,data32 	:integer:=0;

signal count0 			:integer:=0;
signal count1 			:integer:=0;
signal count2 			:integer:=0;
signal count3 			:integer:=0;

signal error0 			:integer range -128 to 127:=0;
signal error1 			:integer range -128 to 127:=0;
signal error2 			:integer range -128 to 127:=0;
signal error3 			:integer range -128 to 127:=0;

signal p_data_out0 		:integer range -45000 to 45000:=0;
signal p_data_out1 		:integer range -45000 to 45000:=0;
signal p_data_out2 		:integer range -45000 to 45000:=0;
signal p_data_out3 		:integer range -45000 to 45000:=0;

signal idata0 			:integer range -45000 to 45000:=0;
signal idata1 			:integer range -45000 to 45000:=0;
signal idata2 			:integer range -45000 to 45000:=0;
signal idata3 			:integer range -45000 to 45000:=0;

signal pi_data_out0		:integer range -45000 to 45000:=0;
signal pi_data_out1		:integer range -45000 to 45000:=0;
signal pi_data_out2		:integer range -45000 to 45000:=0;
signal pi_data_out3		:integer range -45000 to 45000:=0;

signal pwm0 			:std_logic;
signal pwm1 			:std_logic;
signal pwm2				:std_logic;
signal pwm3 			:std_logic;
-----------------------------------------------------Period
signal h: std_logic_vector(0 to 15);
signal k: integer:=0;
signal period: std_logic;
signal pwm_clk:std_logic;
signal pwm_data:integer range -45000 to 45000:=0;
-----------------------------------------------------nRF
signal m :std_logic:='0';
signal step:std_logic_vector(1 downto 0):="00";
signal nRFConectionErr :std_logic:='0';
signal SetAllRegisterIsDone:std_logic:='0';
signal IS_PRX:std_logic:='1';
signal ErrCount:integer:=0;
signal ChipEN:std_logic:='0';
signal enable,receive_enable,rx_full,Tx_Empty : std_logic:='0';
signal sck_int:std_logic:='1';
signal send_buffer1,send_buffer2,send_buffer2_2,send_buffer3,send_buffer4,send_buffer5,send_buffer6,send_buffer7,send_buffer8,send_buffer8_2,send_buffer9,send_buffer10,send_buffer11,send_buffer12,send_buffer13,send_buffer14,send_buffer15,send_buffer16,send_buffer17,send_buffer18,send_buffer19,send_buffer20,send_buffer21,send_buffer22,send_buffer23,send_buffer24,send_buffer25,send_buffer26,send_status,send_fifo_status,send_read_comm,send_buffer30,send_buffer31,send_buffer32,ConfigRegVal,send_W_TX_payload:std_logic_vector(7 downto 0);

signal send_TX_Data :std_logic_vector(239 downto 0);
signal RX_DATA :std_logic_vector(239 downto 0);

signal send_data1,send_data2,send_data3,send_data4,send_data5:std_logic_vector(7 downto 0);
signal Send_Delay_count :integer:=0;
signal is_After_Delay:std_logic:='0';

--signal Timeout_count :integer:=0;
--signal IS_Timeout :std_logic:='0';

TYPE IDType IS ARRAY (5 DOWNTO 0) OF  std_logic_vector(7 downto 0);
signal RX_ID : IDType:=("00000000","00000001","00000010","00000011","00000100","00000101");
signal c1,c2,c3,c4,c5 : std_logic_vector(7 downto 0);

signal packet_index:integer range 0 to 5:=0;

signal RX_SET_Format :std_logic:='0';
signal Temp:std_logic_vector(31 downto 0);

signal TXCount:integer:=0;
signal DataSent:std_logic:='0';
signal kk:integer range -1 to 500:=0;
signal l:integer range -1 to 500:=0;

signal receive_buffer: std_logic_vector(7 downto 0);
signal fifo_status: std_logic_vector(7 downto 0);
signal comm_id : std_logic_vector( 7 downto 0 );
signal LF_MOTOR1,RF_MOTOR1,RB_MOTOR1,LB_MOTOR1:integer range -128 to 127:=0;
signal clk_counter:integer range 0 to 35:=0;

signal read_count0 			:integer :=0;
signal read_count1 			:integer :=0;
signal read_count2 			:integer :=0;
signal read_count3			:integer :=0;

signal read_data01,read_data02 	:integer :=0;
signal read_data11,read_data12 	:integer :=0;
signal read_data21,read_data22 	:integer :=0;
signal read_data31,read_data32 	:integer :=0;

-----------------------------------------------------ADC
signal ch_1 : std_logic_vector(11 downto 0):="000000000000";
signal ch_2 : std_logic_vector(11 downto 0):="000000000000";
signal ch_3 : std_logic_vector(11 downto 0):="000000000000";
signal ch_4 : std_logic_vector(11 downto 0):="000000000000";

signal sb_pwm_data:integer range 0 to 255;
signal sb_pwm : integer range 0 to 255;
------------------------------------------------------Battery Monitor
signal Battery_Level : std_logic:='0';
signal Battery_bf	 : std_logic_vector(9 downto 0);
signal Battery_Hister : std_logic_vector(9 downto 0):="1111111111";
signal battery_f : std_logic:='1';
signal battery_index_sample :integer:=0;
signal Buzzer_PWM_4Hz,Buzzer_PWM_1Hz,check_battery,Battery_Monitor,Battery_Buzzer : std_logic:='0';
signal Duty_1Hz,Duty_4Hz,Duty_battery : integer :=0;
signal first_count		: integer range 0 to 14745600:=0;
signal first_buzz: std_logic:='0';

SIGNAL RX_COUNTER 	:INTEGER RANGE 0 TO 200:=0;
signal cc         	:integer range 0 to 29499:=0;
signal ccA       	:integer RANGE 0 TO 2949120:=0;
SIGNAL T          	:STD_LOGIC:='0';
SIGNAL flag       	:STD_LOGIC:='1';
signal Ball		  	:std_logic:='0';
--------------------------------------------------------Reverse Solution
signal zero_count0 	:integer:=0;
signal zero_count1 	:integer:=0;
signal zero_count2 	:integer:=0;
signal zero_count3 	:integer:=0;
--------------------------------------------------------current limiting
signal RB_overCounter 	: integer :=0;
---------------------------
signal RF_overCounter	: integer :=0;
---------------------------
signal LB_overCounter 	: integer :=0;
---------------------------
signal LF_overCounter 	: integer :=0;

-------------------------------------------------------------------Shoot
--TYPE machine IS(Idle, Discharge,Charge,Shoot); --needed states
--SIGNAL  state		          : machine; 
signal switching_counter      : integer range 0 to 60000;
signal kick_counter_t         : integer range 0 to 29491200 :=0;
signal kick_counter_p         : integer range 0 to 884736000 :=0;
signal shoot_zart_time		  : std_logic;
signal shoot_zart_pwm		  : std_logic;
signal chip_zart_time		  : std_logic;
signal chip_zart_pwm		  : std_logic;
signal chip_counter_t         : integer range 0 to 884736000 :=0;
signal chip_counter_p         : integer range 0 to 884736000 :=0;
signal dis_counter_t          : integer range 0 to 884736000 :=0;
signal dis_counter_p          : integer range 0 to 884736000 :=0;
signal discharge_time		  : std_logic;
signal discharge_PWM		  : std_logic;
signal charge_ok			  : std_logic:='0';
signal kick_pwm				  : integer range 0 to 10000000:=0;
signal kick_power			  : std_logic_vector(2 downto 0):="000";
signal DirectORchip 			  : std_logic:='0';
signal chip_pwm				  : integer range 0 to 10000000:=0;
signal shooting,charging,chiping: std_logic:='0';

signal delay_shoot_cnt		  : integer:=0;
signal safe_ball			  : std_logic:='0';
signal ball_f				  : std_logic:='0';
signal ball_noise			  : std_logic:='0';

signal penalty_mode			  : std_logic_vector(1 downto 0):="00";
signal after_turn_penalty	  : std_logic:='0';
signal delay_turn_penalty	  : integer:=0;
signal lastball				  : std_logic:='0';
--signal kick_enable			  : std_logic:='0';
signal ie	  : integer:=0;
signal pause_ch				  : std_logic:='0';
signal pr_count	  : integer:=0;
begin



--	A2D: adc PORT MAP (
--		clk => main_clk,
--		sda => SDA_ADC,
--		scl => SCL_ADC,
--		CH1 => ch_1,
--		CH2 => ch_2,
--		CH3 => ch_3,
--		CH4 => ch_4 );



	sb_fw <= '0';
	sb_pwm<=50;
	
cs <= m;
	ce <= ChipEN;
	sck <= sck_int;
	---------------------------------1,2
	send_buffer1  	 <= "00100000";--0x20
	send_buffer2   	 <= "00001111";--PRX
	Send_buffer2_2	 <= "00001110";--PTX
	---------------------------------3,4
	send_buffer3  	 <= "00100001";--0x21
	send_buffer4  	 <= "00000000";
	---------------------------------5,6
	send_buffer5  	 <= "00100110";--0x26
	send_buffer6  	 <= "00000110";
	--------------------------------- Set Frequency
	send_buffer7  	 <= "00100101";--0x25
	--send_buffer8  	 <= "01110011";--		Rx   Example:[115=>Freq=2400+115=2515]
	send_buffer8  	 <= "01110011"; --rx         115
	send_buffer8_2 	 <= "00110010";--		Tx 80
	---------------------------------9.10
	send_buffer9  	 <= "00110001";--0x31
	send_buffer10 	 <= "00011110";
	---------------------------------11,12,13,14,15,16
	send_buffer11 	 <= "00101010";--0x2A
	send_buffer12 	 <= "11100111";
	send_buffer13 	 <= "11100111";
	send_buffer14 	 <= "11100111";
	send_buffer15 	 <= "11100111";
	send_buffer16 	 <= "11100111";
	---------------------------------17,18,19,20,21,22
	send_buffer17 	 <= "00110000";--0x30
	send_buffer18  	 <= "11100111";
	send_buffer19 	 <= "11100111";
	send_buffer20 	 <= "11100111";
	send_buffer21 	 <= "11100111";
	send_buffer22 	 <= "11100111";
	---------------------------------23,24
	send_buffer23 	 <= "00111100";--0x3C
	send_buffer24 	 <= "00000000";
	---------------------------------25,26
	send_buffer25 	 <= "00111101";--0x3D
	send_buffer26 	 <= "00000001";
	--------------------------------------interupt register
	send_status      <= "00000111";--Read Status 0x07
	send_fifo_status <= "00010111";--Read FifoStatus 0x17
	send_read_comm   <= "01100001";--0x61
	send_W_TX_payload<= "10110000";--0xB0
	send_buffer30    <= "00100111";--Write status 0x27
	send_buffer31    <= "11111111";--Clear Status 0xFF
	send_buffer32    <= "00000000";--Read Config register 0x00

	send_data1		 <= ("0001" & Robot_ID(3 downto 0));
----	send_data2		 <= "1010" & "1011";
----	send_data3		 <= "1100" & "1101";
----	send_data4		 <= "1110" & "1111";
----	send_data5		 <= "1010" & "1101";
	--Temp <= conv_std_logic_vector(-127,8);
	send_data2		 <= conv_std_logic_vector(read_count0,8);
	send_data3		 <= conv_std_logic_vector(read_count1,8);--"1100" & "1101";
	send_data4		 <= conv_std_logic_vector(read_count2,8);--"1110" & "1111";
	send_data5		 <= conv_std_logic_vector(read_count3,8);--"1010" & "1101";
	--------------------------------------------------
	
	--------------------------------------------------
	

process(main_clk)
begin
	if(rising_edge(main_clk))then
		if(IS_PRX='1')then
			if(l>283)then
				if(RX_SET_Format='0')then
					for i in 0 to 239 loop
						send_TX_Data(i)<= RX_DATA(i);
						if(i=239)then
							RX_SET_Format<='1';
						end if;
					end loop;
				end if;
			end if;
		end if;
		if(RX_SET_Format='1')then
			for i in 0 to 7 loop
				send_TX_Data(packet_index*40+0 +i)<= send_data5(i);
				send_TX_Data(packet_index*40+8 +i)<= send_data4(i);
				send_TX_Data(packet_index*40+16+i)<= send_data3(i);
				send_TX_Data(packet_index*40+24+i)<= send_data2(i);
				send_TX_Data(packet_index*40+32+i)<= send_data1(i);
				if(i=7)then
					RX_SET_Format<='0';
				end if;
			end loop;
		end if;
	end if;
end process;
-----===========================================TEST===========================
--process(main_clk)
--begin
--if(rising_edge(main_clk))then
--
--	if(robot_id(3)='1')then
--		setpoint0<=10;
--	else
--		setpoint0<=0;
--	end if;
--	-------------------------
--	if(robot_id(2)='1')then
--		setpoint1<=10;
--	else
--		setpoint1<=0;
--	end if;
--	-------------------------
--	if(robot_id(1)='1')then
--		setpoint2<=10;
--	else
--		setpoint2<=0;
--	end if;
--	-------------------------
--	if(robot_id(0)='1')then
--		DirectORchip <='1';
--	else
--		DirectORchip <='0';
--	end if;
--
--	if(robot_id(3)='1')then
--		kick_power<="100";
--	else
--		kick_power<="000";
--	end if;
--
--end if;
--end process;

--kick_power(0)<=robot_id(2);
--kick_power(1)<=robot_id(1);
--kick_power(2)<=robot_id(0);
--
--kick_power <= "111";
--
--
--DirectORchip <='0';
--penalty_mode(0) <= robot_id(0);
--
--penalty_mode(1) <='0';

---=========================================================================
--------------------------------------------------------------------------------------Penalty-------------------
process(main_clk)
begin
	if(rising_edge(main_clk)) then
		if(penalty_mode="00" )then
			setpoint0 <=speed0;
			setpoint1 <=speed1;
			setpoint2 <=speed2;
			setpoint3 <=speed3;
		else
			if(ball='1' or lastball='1')then
				if(penalty_mode="01")then
					setpoint0 <=-15;
					setpoint1 <=-15;
					setpoint2 <=0;
					setpoint3 <=0;
				elsif(penalty_mode="10")then
					setpoint0 <=0;
					setpoint1 <=0;
					setpoint2 <=15;
					setpoint3 <=15;
				end if;
			else
				setpoint0 <=-1;
				setpoint1 <=-1;
				setpoint2 <=1;
				setpoint3 <=1;
			end if;--if(ball='1' or lastball='1')then
		end if;--if(penalty_mode="00")then
	end if;--rising
end process;

process(IS_PRX,mp21,mp31,mp01,mp11)
begin						-- counter process
-------------------------------counter0
	if (IS_PRX='0' and l>283) then                            			-- period = ON ?
		read_data01  <=0;
		read_data02  <=0;
	elsif (rising_edge(mp01)) then             				-- Clk1 in ?
		if mp02='1' then                      				-- Yes. Up count ?
			read_data01 <= read_data01 + 1;              				-- Yes. Count-up
		else                                				-- Not count-up
			read_data01 <= read_data01 - 1;              				-- Count-down
		end if;
				
	elsif(falling_edge(mp01)) then						
		if mp02='0' then									-- yes. up count?
			read_data02 <= read_data02 + 1;							-- yes. count-up
		else 												-- not count up
			read_data02 <= read_data02 - 1;							-- count-down
		end if;
	end if;	
-------------------------------counter1
	if (IS_PRX='0' and l>283) then                            			-- period = ON ?
		read_data11  <=0;
		read_data12  <=0;
	elsif (rising_edge(mp11)) then             				-- Clk1 in ?
		if mp12='1' then                      				-- Yes. Up count ?
			read_data11 <= read_data11 + 1;              				-- Yes. Count-up
		else                                				-- Not count-up
			read_data11 <= read_data11 - 1;              				-- Count-down
		end if;
				
	elsif(falling_edge(mp11)) then						
		if mp12='0' then									-- yes. up count?
			read_data12 <= read_data12 + 1;							-- yes. count-up
		else 												-- not count up
			read_data12 <= read_data12 - 1;							-- count-down
		end if;
	end if;	

-------------------------------counter2
	if (IS_PRX='0' and l>238) then                            			-- period = ON ?
		read_data21  <=0;
		read_data22  <=0;
	elsif (rising_edge(mp21)) then             				-- Clk1 in ?
		if mp22='1' then                      				-- Yes. Up count ?
			read_data21 <= read_data21 + 1;              				-- Yes. Count-up
		else                                				-- Not count-up
			read_data21 <= read_data21 - 1;              				-- Count-down
		end if;
				
	elsif(falling_edge(mp21)) then						
		if mp22='0' then									-- yes. up count?
			read_data22 <= read_data22 + 1;							-- yes. count-up
		else 												-- not count up
			read_data22 <= read_data22 - 1;							-- count-down
		end if;
	end if;	
	
-------------------------------counter3
	if (IS_PRX='0' and l>283) then                            			-- period = ON ?
		read_data31  <=0;
		read_data32  <=0;
	elsif (rising_edge(mp31)) then             				-- Clk1 in ?
		if mp32='1' then                      				-- Yes. Up count ?
			read_data31 <= read_data31 + 1;              				-- Yes. Count-up
		else                                				-- Not count-up
			read_data31 <= read_data31 - 1;              				-- Count-down
		end if;
				
	elsif(falling_edge(mp31)) then						
		if mp32='0' then									-- yes. up count?
			read_data32 <= read_data32 + 1;							-- yes. count-up
		else 												-- not count up
			read_data32 <= read_data32- 1;							-- count-down
		end if;
	end if;	
end process;

	read_count0 <= 127 when read_data01/4>=127 else read_data01/4;
	read_count1 <= 127 when read_data11/4>=127 else read_data11/4;
	read_count2 <= 127 when read_data21/4>=127 else read_data21/4;
	read_count3 <= 127 when read_data31/4>=127 else read_data31/4;

--------------------------PI Error Calculation
	count0 <= data01 + data02;
	count1 <= data11 + data12;
	count2 <= data21 + data22;
	count3 <= data31 + data32;
	
	error0 <= setpoint0 - count0;	
	error1 <= setpoint1 - count1;	
	error2 <= setpoint2 - count2;	
	error3 <= setpoint3 - count3;
-----------------------------------------------

process(main_clk)--FOR TWICE BUZZ!!
begin
if(rising_edge(main_clk))then
	
	if(first_count < 14744600)then
		first_count <= first_count +1;
	else
		first_count <= 14744699;
		first_buzz <= '1';
	end if;
end if;
end process;

-----------PWM Generator [4 Hz] ---------
process(main_clk)
variable Freq :integer :=4;--Hz
begin
if(rising_edge(main_clk))then
	if(Duty_4Hz < 29491200/Freq)then
		Duty_4Hz <= Duty_4Hz+1;
		if(Duty_4Hz < 14745600/Freq)then
			Buzzer_PWM_4Hz<='1';
		else
			Buzzer_PWM_4Hz<='0';
		end if;	
	else
		Duty_4Hz <= 0;
	end if;
end if;		
end process;

process(Buzzer_PWM_1Hz)
begin
if(rising_edge(Buzzer_PWM_1Hz))then
	if(battery_index_sample<10)then
		Battery_Hister(battery_index_sample) <= Battery_f;
		battery_index_sample <= battery_index_sample +1;
	else
		battery_index_sample <= 0;
	end if;
end if;		
end process;

process(main_clk)
begin
if(rising_edge(main_clk))then
	if(Battery_Hister(0)='0' and Battery_Hister(1)='0' and Battery_Hister(2)='0' and Battery_Hister(3)='0' and Battery_Hister(4)='0' and Battery_Hister(5)='0' and Battery_Hister(6)='0' and Battery_Hister(7)='0' and Battery_Hister(8)='0' and Battery_Hister(9)='0' )then
		Battery_Buzzer<='1';
	else
		Battery_Buzzer<='0';
	end if;
end if;
end process;

-----------PWM Generator [1 Hz] ---------
process(main_clk)
variable Freq :integer :=1;--Hz
begin
if(rising_edge(main_clk))then
	if(Duty_1Hz < 29491200/Freq)then
		Duty_1Hz <= Duty_1Hz+1;
		if(Duty_1Hz < 14745600/Freq)then
			Buzzer_PWM_1Hz<='1';
		else
			Buzzer_PWM_1Hz<='0';
		end if;	
	else
		Duty_1Hz <= 0;
	end if;
end if;		
end process;

------------------------------------------------------------------------------------Battery Monitor-------------
--process(main_clk)
--begin
--if(rising_edge(main_clk))then
--	if(  ch_1 < "011110000000") then
--		Battery_Level <='1';
--	else
--		Battery_Level <='0';
--	end if;
--end if;
--end process;
--Battery_Monitor <= Battery_Buzzer and Buzzer_PWM_4Hz;
--------------------------------------------------------------------------------------Buzzer-------------------
process(main_clk)
begin
	if(first_buzz ='0')then
		Buzzer <= Buzzer_PWM_4Hz;
	else	
		if(Robot_ID="0000")then
			Buzzer <= ball_f;
		else
			--if( Battery_Buzzer ='1')then
				--Buzzer <= Battery_Monitor;
			--else
				if(Ball='1' OR nRFConectionErr='1') then
					Buzzer <= '1';
				else
					Buzzer <= '0';
				
			end if;
		end if;
	end if;
end process;
--LED1 <= not(charge_ok);
LED2 <= not(charge_ok);

process(main_clk)--Shoot Delay in Penalty
begin
	if(rising_edge(main_clk))then
		if(penalty_mode="00")then
			lastball<='0';
		else
			if(ball='1')then
				if(shooting='0' and charging='0')then
					lastball<='1';
				end if;
			end if;
			if(shooting='1')then
				lastball<='0';
			end if;
	
		end if;
	end if;
end process;



process(main_clk)
variable cnt_noise : integer:=0;
begin
if(rising_edge(main_clk)) then
	if(charge_ok='1') then
		cnt_noise:=cnt_noise+1;
	else
		cnt_noise:=0;
		ball_noise<='0';
	end if;
	
	if(cnt_noise > 7500000) then
		ball_noise<='1';
		cnt_noise:=7500009;
	else
		ball_noise<='0';
	end if;
end if;
end process;

process(main_clk)----------------------------------------------------------------------CHARGE-----------------------
begin
if(rising_edge(main_clk)) then
	if(discharge_key1='1' and discharge_key0='1') then
			if (adc='0' and charge_ok='0' ) then 
				if(shooting='0' and chiping='0') then
					charging<='1';
					switching_counter<=switching_counter+1;
					if (switching_counter>240) then
						if(switching_counter=368) then
							switching_counter<=0;
						end if;	
						switch_pulse<='0';
					else
						switch_pulse<='1';
					end if;
				else
					switch_pulse<='0';
				end if;	
			else
				switch_pulse<='0';
				charging<='0';
				charge_ok<='1';
			end if;
			
			if (adc='0' and charge_ok='1' and penalty_mode="00") then 
				charge_ok<='0';
			end if;	
	else
		switch_pulse<='0';
		charging<='0';
	end if;
end if;
end process;

process(main_clk)
begin
if(rising_edge(main_clk)) then
		pr_count<=pr_count+1;
		if(pr_count<90000000) then
			if(pr_count<60000000) then
				pause_ch<='1';
			else
				pause_ch<='0';
			end if;
		else
			pr_count<=0;
		end if;
	end if;
end process;
-------------------------------------------------------------------------------------DISCHARGE-------------------
process(main_clk)
begin
----------------- time
	if(rising_edge(main_clk)) then
			dis_counter_t<=dis_counter_t+1;
				if (dis_counter_t>36500) then
					if(dis_counter_t=20000000) then
						dis_counter_t<=0;
					end if;
					discharge_time<='0';
				else
					discharge_time<='1';
				end if;
		
------------------pulse PWM
			dis_counter_p<=dis_counter_p+1;
				if (dis_counter_p>21000) then
					if(dis_counter_p=30000) then
						dis_counter_p<=0;
					end if;
					discharge_PWM<='0';
				else
					discharge_PWM<='1';
				end if;
end if;
end process;

--------------------------------------------------------------------------------Direct Kick----------------
--------------------------------------Shoot time
process(main_clk)
variable kick_enable : std_logic:='0';
begin
if(rising_edge(main_clk)) then
	if(penalty_mode="00")then
		if (DirectORchip='0' and kick_power>"000" and ball='1' and charging='0') then 
			kick_enable:='1';
		end if;
	else
		if(after_turn_penalty='1' and charging='0')then
			kick_enable:='1';
			after_turn_penalty<='0';
		end if;
		
		if(lastball='1')then
			if(delay_turn_penalty<3375000)then--DelayPenalty
				delay_turn_penalty <= delay_turn_penalty+1;
				after_turn_penalty<='0';
			else
				after_turn_penalty<='1';
				delay_turn_penalty<=3375009;
			end if;
		else
			after_turn_penalty<='0';
			delay_turn_penalty<=0;
		end if;
		
	end if;
	
	if(kick_enable ='1') then
		kick_counter_t<=kick_counter_t+1;
	else
		kick_counter_t<=0;
	end if;

	if (kick_counter_t<3000000 and kick_counter_t>0) then
		shoot_zart_time<='1';
		shooting<='1';
	else
		shoot_zart_time<='0';
		shooting<='0';
		kick_enable:='0';
	end if;

end if;
end process;
--------------------------------------Kick POWER
process(main_clk)
begin
if(rising_edge(main_clk)) then
	--if (DirectORchip='0' and kick_power>"000" and ball='1' and charging='0') then
	if(penalty_mode="00")then
		if ( kick_power="001") then
			kick_pwm<=1500;
		elsif (kick_power="010") then
			kick_pwm<=3750;
		elsif (kick_power="011") then
			kick_pwm<=6000;
		elsif (kick_power="100") then
			kick_pwm<=8250;
		elsif (kick_power="101") then
			kick_pwm<=10500;
		elsif (kick_power="110") then
			kick_pwm<=12600;
		elsif (kick_power="111") then
			kick_pwm<=12750;
		end if;
	else
		kick_pwm<=12750;
	end if;
		
		kick_counter_p<=kick_counter_p+1;
		if (kick_counter_p>kick_pwm) then
			if(kick_counter_p=15005) then
				kick_counter_p<=0;
			end if;
			shoot_zart_PWM<='0';
		else
			shoot_zart_PWM<='1';
		end if;
	--end if;
end if;
end process;
----------------------------------------------------------------- Decide To Kick OR Discharge-----------------
process(main_clk)
begin
if(rising_edge(main_clk)) then								---------To KICK 
		if(discharge_key1='1' and discharge_key0='1') then
			if (DirectORchip='0' and charging='0') then
				shoot_zart<=shoot_zart_time AND shoot_zart_PWM;
			else
				shoot_zart<='0';
			end if;

															---------To Chip
			if (DirectORchip='1' and kick_power>"000" and charging='0') then
				chip_zart<=chip_zart_time AND chip_zart_PWM;
			else
				chip_zart<='0';
			end if;
			
		elsif(discharge_key0='0' and discharge_key1='0') then-------To Discharge
			shoot_zart<=discharge_time AND discharge_PWM;
		else
			shoot_zart<='0';
		end if;
end if;
end process;
-----------------------------------------------------------------------chip kick
-------------------------------------------Chip time
process(main_clk)
variable chip_enable : std_logic:='0';
begin
if(rising_edge(main_clk)) then
	if (DirectORchip='1' and kick_power>"000" and ball='1' and charging='0') then 
		chip_enable:='1';
	end if;
	
	if(chip_enable ='1') then
		chip_counter_t<=chip_counter_t+1;
	else
		chip_counter_t<=0;
	end if;
	
	if (chip_counter_t<3000000 and chip_counter_t>0) then
		chip_zart_time<='1';
		chiping<='1';
	else
		chip_zart_time<='0';
		chiping<='0';
		chip_enable:='0';
	end if;

end if;
end process;
--------------------------------------------chip PWM
process(main_clk)
begin
if(rising_edge(main_clk)) then
--	if (DirectORchip='1' and kick_power>"000" and ball='0') then
		if (kick_power="001") then
			chip_pwm<=1500;
		elsif (kick_power="010") then
			chip_pwm<=3750;
		elsif (kick_power="011") then
			chip_pwm<=6000;
		elsif (kick_power="100") then
			chip_pwm<=8250;
		elsif (kick_power="101") then
			chip_pwm<=10500;
		elsif (kick_power="110") then
			chip_pwm<=12600;
		elsif (kick_power="111") then
			chip_pwm<=12750;
		end if;
		
		chip_counter_p<=chip_counter_p+1;
		if (chip_counter_p>chip_pwm) then
			if(chip_counter_p=15005) then
				chip_counter_p<=0;
			end if;
			chip_zart_PWM<='0';
		else
			chip_zart_PWM<='1';
		end if;
--	end if;
end if;
end process;
--------------------------------------------chip  zart
--process(main_clk)
--begin
--if(rising_edge(main_clk)) then
--		if(discharge_key1='1' and discharge_key0='1') then
--			if (DirectORchip='1' and kick_power>"000" and charging='0') then
--				chip_zart<=chip_zart_time AND chip_zart_PWM;
--			else
--				chip_zart<='0';
--			end if;
--		end if;
--end if;
--end process;
------------------------------------------------------------------------Reverse Solution-----------------------------------
--process(main_clk)
--begin
--	if(rising_edge(main_clk)) then
--		if(setpoint0<speed0)then
--			if(setpoint0=0 and zero_count0<ReverseDelay0) then
--				zero_count0<=zero_count0+1;
--			else
--				zero_count0<=0;
--				setpoint0<=setpoint0+1;
--			end if;
--		elsif(setpoint0>speed0) then
--			if(setpoint0=0 and zero_count0<ReverseDelay0) then
--				zero_count0<=zero_count0+1;
--			else
--				zero_count0<=0;
--				setpoint0<=setpoint0-1;
--			end if;
--		end if;
--	end if;
--end process;
--------------------------------------------------------------
--process(main_clk)
--begin
--	if(rising_edge(main_clk)) then
--		if(setpoint1<speed1)then
--			if(setpoint1=0 and zero_count1<ReverseDelay1) then
--				zero_count1<=zero_count1+1;
--			else
--				zero_count1<=0;
--				setpoint1<=setpoint1+1;
--			end if;
--		elsif(setpoint1>speed1) then
--			if(setpoint1=0 and zero_count1<ReverseDelay1) then
--				zero_count1<=zero_count1+1;
--			else
--				zero_count1<=0;
--				setpoint1<=setpoint1-1;
--			end if;
--		end if;
--	end if;
--end process;----------------------------------------------------------------	
--process(main_clk)
--begin
--	if(rising_edge(main_clk)) then	
--		if(setpoint2<speed2)then
--			if(setpoint2=0 and zero_count2<ReverseDelay2) then
--				zero_count2<=zero_count2+1;
--			else
--				zero_count2<=0;
--				setpoint2<=setpoint2+1;
--			end if;
--		elsif(setpoint2>speed2) then
--			if(setpoint2=0 and zero_count2<ReverseDelay2) then
--				zero_count2<=zero_count2+1;
--			else
--				zero_count2<=0;
--				setpoint2<=setpoint2-1;
--			end if;
--		end if;
--	end if;
--end process;
-----------------------------------------------------------------		
--process(main_clk)
--begin
--	if(rising_edge(main_clk)) then
--		if(setpoint3<speed3)then
--			if(setpoint3=0 and zero_count3<ReverseDelay3) then
--				zero_count3<=zero_count3+1;
--			else
--				zero_count3<=0;
--				setpoint3<=setpoint3+1;
--			end if;
--		elsif(setpoint3>speed3) then
--			if(setpoint3=0 and zero_count3<ReverseDelay3) then
--				zero_count3<=zero_count3+1;
--			else
--				zero_count3<=0;
--				setpoint3<=setpoint3-1;
--			end if;
--		end if;		
--	end if;
--end process;


	--LF
	kp0<=1500;
	ki0<=50;
	--LB
	kp1<=1500;
	ki1<=50;
	--RB
	kp2<=1500;
	ki2<=50;
	--RF
	kp3<=1500;
	ki3<=50;

	
---------------------------------------------------------------------------Variable PI By Speed-----------------------------
--process(main_clk)-----------------------------------------LF
--begin
--	if(abs(setpoint0)=1) then
--		kp0<=2000;
--		ki0<=12;
--	elsif(abs(setpoint0)=2) then
--		kp0<=2000;
--		ki0<=9;
--	elsif(abs(setpoint0)<5 and abs(setpoint0)>2) then
--		kp0<=2000;
--		ki0<=8;
--	elsif(abs(setpoint0)<9 and abs(setpoint0)>4) then
--		kp0<=2000;
--		ki0<=6;
--	else
--		kp0<=2000;
--		ki0<=2;
--	end if;
------------------------------------------------------------LB
--	if(abs(setpoint1)=1) then
--		kp1<=2000;
--		ki1<=12;
--	elsif(abs(setpoint1)=2) then
--		kp1<=2000;
--		ki1<=9;
--	elsif(abs(setpoint1)<5 and abs(setpoint1)>2) then
--		kp1<=2000;
--		ki1<=8;
--	elsif(abs(setpoint1)<9 and abs(setpoint1)>4) then
--		kp1<=2000;
--		ki1<=6;
--	else
--		kp1<=2000;
--		ki1<=2;
--	end if;
-----------------------------------------------------------RB
--	if(abs(setpoint2)=1) then
--		kp2<=2000;
--		ki2<=12;
--	elsif(abs(setpoint2)=2) then
--		kp2<=2000;
--		ki2<=9;
--	elsif(abs(setpoint2)<5 and abs(setpoint2)>2) then
--		kp2<=2000;
--		ki2<=8;
--	elsif(abs(setpoint2)<9 and abs(setpoint2)>4) then
--		kp2<=2000;
--		ki2<=6;
--	else
--		kp2<=2000;
--		ki2<=2;
--	end if;
-----------------------------------------------------------RF
--	if(abs(setpoint3)=1) then
--		kp3<=2000;
--		ki3<=12;
--	elsif(abs(setpoint3)=2) then
--		kp3<=2000;
--		ki3<=9;
--	elsif(abs(setpoint3)<5 and abs(setpoint3)>2) then
--		kp3<=2000;
--		ki3<=8;
--	elsif(abs(setpoint3)<9 and abs(setpoint3)>4) then
--		kp3<=2000;
--		ki3<=6;
--	else
--		kp3<=2000;
--		ki3<=2;
--	end if;	
--end process;


------------------------------------------------------------------------Ball Detection---------------
PROCESS(main_clk)
BEGIN
if(rising_edge(main_clk))then
	if(rx='0')then
		ball_f<='1';
	else
		ball_f<='0';
	end if;								
END IF;
end process;

ball<= ball_f and ball_noise;--To Remove Charge noise :)

tx<='1';

----------------------------------------------------------------------------nRF---------------------------

process(sck_int)
begin
if(falling_edge(sck_int))then
	if(kk=-1 or kk=16 or kk=17 or kk=34 or kk=35 or kk=52 or kk=53 or kk=70 or kk=71 or kk=88 or kk=89 or kk=130 or kk=131 or kk=180 or kk=181 or kk=198 or kk=199 or kk=216 )then
		m<='1';
	else
		m<='0';
	end if;
		
	if(SetAllRegisterIsDone='0')then
		-----------------------------------1,2
		if(kk<8 )then
			mosi <= send_buffer1(7-kk);
		end if;
		if(kk<16 and kk>7 and IS_PRX='1')then--PRX=0x0F
			mosi <= send_buffer2(15-kk);
		end if;
		
		if(kk<16 and kk>7 and IS_PRX='0')then--PTX=0x0E
			mosi <= send_buffer2_2(15-kk);
		end if;		
	
		----------------------------------3,4
		if(kk>17 and kk<26 )then
			mosi <= send_buffer3(25-kk);
		end if;
		if(kk<34 and kk>25 )then
			mosi <= send_buffer4(33-kk);
		end if;
	
		--------------------------------5,6
		if(kk>35 and kk<44)then
			mosi <= send_buffer5(43-kk);
		end if;
		if(kk<52 and kk>43 )then
			mosi <= send_buffer6(51-kk);
		end if;
	
		---------------------------------7,8
		if(kk<62 and kk>53)then
			mosi <= send_buffer7(61-kk);
		end if;
		if(kk>61 and kk<70 and IS_PRX='1')then--Receiver...
			mosi <= send_buffer8(69-kk);
		end if;
		
		if(kk>61 and kk<70 and IS_PRX='0')then--Sender...
			mosi <= send_buffer8_2(69-kk);
		end if;

		---------------------------------9,10
		if(kk<80 and kk>71)then
			mosi <= send_buffer9(79-kk);
		end if;
		if(kk<88 and kk>79 )then
			mosi <= send_buffer10(87-kk);
		end if;
		---------------------------------11,12,13,14,15,(16)!
		if(kk<98 and kk>89)then
			mosi <= send_buffer11(97-kk);
		end if;
		if(kk<106 and kk>97 )then
			mosi <= send_buffer12(105-kk);
		end if;
		if(kk<114 and kk>105)then
			mosi <= send_buffer13(113-kk);
		end if;
		if(kk<122 and kk>113 )then
			mosi <= send_buffer14(121-kk);
		end if;
		if(kk<130 and kk>121)then
			mosi <= send_buffer15(129-kk);
		end if;
		---------------------------------17,18,19,20,21,22
		if(kk<140 and kk>131)then
			mosi <= send_buffer17(139-kk);
		end if;
		if(kk<148 and kk>139 )then
			mosi <= send_buffer18(147-kk);
		end if;
		if(kk<156 and kk>147)then
			mosi <= send_buffer19(155-kk);
		end if;
		if(kk<164 and kk>155)then
			mosi <= send_buffer20(163-kk);
		end if;
		if(kk<172 and kk>163)then
			mosi <= send_buffer21(171-kk);
		end if;
		if(kk<180 and kk>171)then
			mosi <= send_buffer22(179-kk);
		end if;
		---------------------------------23,24
		if(kk<190 and kk>181 )then
			mosi <= send_buffer23(189-kk);
		end if;
		if(kk<198 and kk>189 )then
			mosi <= send_buffer24(197-kk);
		end if;
	
		---------------------------------25,26
		if(kk<208 and kk>199)then
			mosi <= send_buffer25(207-kk);
		end if;
		if(kk<216 and kk>207 )then
			mosi <= send_buffer26(215-kk);
		end if;
	else
		if(  l = 0 or l = 17 or l=34 OR L = 283 or l=300) then
			m <= '1';
		else 
			m <= '0';
		end if;
		
		if(l > 0 and l < 9 )then
			mosi <= send_buffer32(8-l);--Command R_Config
		end if;

		if( l>17 and l<26 )then
			mosi <= send_status(25-l);--Command R_Status
		end if;

		
		if( l>34 and l<43 and IS_PRX='1')then
			mosi <= send_read_comm(42-l);--Command Receive
		end if;
		
		if( l>34 and l<43 and IS_PRX='0')then
			mosi <= send_W_TX_payload(42-l);--Command SEND
		end if;
		
		if(l>42 and l<283 and IS_PRX='0') then
			mosi <= send_TX_Data(282-l);--Sending data...
		end if;
		
		if( l>283 and l<292 )then
			mosi <= send_buffer30(291-l);--Comand W_Status
		end if;
		
		if(l>291 and l<300 )then
			mosi <= send_buffer31(299-l);--Clear Rx_DR
		end if;
		
		if(l>300 and l<309 )then
			mosi <= send_fifo_status(308-l);--Command R_FiFo_Status
		end if;

	end if;	
end if;
end process;

IE<=0;
process(sck_int)
begin
if(rising_edge(sck_int))then				--READ MISO
		if( l>8 and l< 17 )then
			ConfigRegVal(16-l) <= miso;		--Read Config Register value
		end if;
		
		if( l>25 and l< 34 )then
			receive_buffer(33-l) <= miso;	--Read Status Register value
		end if;
		
		if(IS_PRX='1') then
			if(l>42 and l<283) then
				RX_DATA(282-l) <=miso;
			end if;
			
			
--			for i in 0 to 5 loop 				--Read Data Packets
--				if(l>42 + i*40 and l<51 + i*40 )then
--					comm_id(i*40 + 50-l) <= miso;
--				end if;
--				------------------------------------------RF
--				IF(L=51+i*40 AND MISO='1')THEN
--					RF_MOTOR1 <= RF_MOTOR1 - 32;
--				ELSIF(L=52+i*40 AND MISO='1')THEN
--					RF_MOTOR1 <= RF_MOTOR1 + 16;
--				ELSIF(L=53+i*40 AND MISO='1')THEN
--					RF_MOTOR1 <= RF_MOTOR1 + 8;
--				ELSIF(L=54+i*40 AND MISO='1')THEN
--					RF_MOTOR1 <= RF_MOTOR1 + 4;
--				ELSIF(L=55+i*40 AND MISO='1')THEN
--					RF_MOTOR1 <= RF_MOTOR1 + 2;
--				ELSIF(L=56+i*40 AND MISO='1')THEN
--					RF_MOTOR1 <= RF_MOTOR1 + 1;
--				------------------------------------------RB	
--				ELSIF(L=57+i*40 AND MISO='1')THEN
--					RB_MOTOR1 <= RB_MOTOR1 - 32;
--				ELSIF(L=58+i*40 AND MISO='1')THEN
--					RB_MOTOR1 <= RB_MOTOR1 + 16;
--				ELSIF(L=59+i*40 AND MISO='1')THEN
--					RB_MOTOR1 <= RB_MOTOR1 + 8;
--				ELSIF(L=60+i*40 AND MISO='1')THEN
--					RB_MOTOR1 <= RB_MOTOR1 + 4;
--				ELSIF(L=61+i*40 AND MISO='1')THEN
--					RB_MOTOR1 <= RB_MOTOR1 + 2;
--				ELSIF(L=62+i*40 AND MISO='1')THEN
--					RB_MOTOR1 <= RB_MOTOR1 + 1;
--				------------------------------------------LB	
--				ELSIF(L=63+i*40 AND MISO='1')THEN
--					LB_MOTOR1 <= LB_MOTOR1 - 32;
--				ELSIF(L=64+i*40 AND MISO='1')THEN
--					LB_MOTOR1 <= LB_MOTOR1 + 16;
--				ELSIF(L=65+i*40 AND MISO='1')THEN
--					LB_MOTOR1 <= LB_MOTOR1 + 8;
--				ELSIF(L=66+i*40 AND MISO='1')THEN
--					LB_MOTOR1 <= LB_MOTOR1 + 4;
--				ELSIF(L=67+i*40 AND MISO='1')THEN
--					LB_MOTOR1 <= LB_MOTOR1 + 2;
--				ELSIF(L=68+i*40 AND MISO='1')THEN
--					LB_MOTOR1 <= LB_MOTOR1 + 1;
--				------------------------------------------LF	
--				ELSIF(L=69+i*40 AND MISO='1')THEN
--					LF_MOTOR1 <= LF_MOTOR1 - 32;
--				ELSIF(L=70+i*40 AND MISO='1')THEN
--					LF_MOTOR1 <= LF_MOTOR1 + 16;
--				ELSIF(L=71+i*40 AND MISO='1')THEN
--					LF_MOTOR1 <= LF_MOTOR1 + 8;
--				ELSIF(L=72+i*40 AND MISO='1')THEN
--					LF_MOTOR1 <= LF_MOTOR1 + 4;
--				ELSIF(L=73+i*40 AND MISO='1')THEN
--					LF_MOTOR1 <= LF_MOTOR1 + 2;
--				ELSIF(L=74+i*40 AND MISO='1')THEN
--					LF_MOTOR1 <= LF_MOTOR1 + 1;
--				END IF;	
--					
--					
--				if( l>74+i*40 and l<83+i*40 ) then
--					if(comm_id(3 downto 0) = Robot_ID(3 downto 0) and comm_id(7 downto 4)="0000")then
--					
--						if( l = 75+i*40 )then
--							speed3 <= -RF_MOTOR1;
--							speed2 <= -RB_MOTOR1;
--							speed1 <= -LB_MOTOR1;
--							speed0 <= -LF_MOTOR1;
--						end if;
--					
--						if(l>74+i*40 and l<78+i*40)then
--							kick_power(i*40 + 77-l) <= miso;
--						elsif(l>77+i*40 and l<79+i*40)then
--							DirectORchip <= miso;
------					elsif(l>78+i*40 and l<80+i*40)then
------						sb_break <= miso;
--						elsif(l>79+i*40 and l<82+i*40)then
--							penalty_mode(i*40 + 81-l) <= miso;
------					elsif(l>81+i*40 and l<83+i*40)then--281>282<283
----							force_kick <= miso;
--						end if;
--					end if;
--					RF_MOTOR1 <= 0;
--					RB_MOTOR1 <= 0;
--					LB_MOTOR1 <= 0;
--					LF_MOTOR1 <= 0;
--				end if;
--
--			end loop;
--			
--			
				if(l>42 + ie*40 and l<51 + ie*40 )then
					comm_id(ie*40 + 50-l) <= miso;
				end if;
				if(comm_id(3 downto 0) = Robot_ID(3 downto 0) and comm_id(7 downto 4)="0000" and l>50 and l<90)then
				------------------------------------------RF
				IF(L=51+ie*40 AND MISO='1')THEN
					RF_MOTOR1 <= RF_MOTOR1 - 32;
				ELSIF(L=52+ie*40 AND MISO='1')THEN
					RF_MOTOR1 <= RF_MOTOR1 + 16;
				ELSIF(L=53+ie*40 AND MISO='1')THEN
					RF_MOTOR1 <= RF_MOTOR1 + 8;
				ELSIF(L=54+ie*40 AND MISO='1')THEN
					RF_MOTOR1 <= RF_MOTOR1 + 4;
				ELSIF(L=55+ie*40 AND MISO='1')THEN
					RF_MOTOR1 <= RF_MOTOR1 + 2;
				ELSIF(L=56+ie*40 AND MISO='1')THEN
					RF_MOTOR1 <= RF_MOTOR1 + 1;
				------------------------------------------RB	
				ELSIF(L=57+ie*40 AND MISO='1')THEN
					RB_MOTOR1 <= RB_MOTOR1 - 32;
				ELSIF(L=58+ie*40 AND MISO='1')THEN
					RB_MOTOR1 <= RB_MOTOR1 + 16;
				ELSIF(L=59+ie*40 AND MISO='1')THEN
					RB_MOTOR1 <= RB_MOTOR1 + 8;
				ELSIF(L=60+ie*40 AND MISO='1')THEN
					RB_MOTOR1 <= RB_MOTOR1 + 4;
				ELSIF(L=61+ie*40 AND MISO='1')THEN
					RB_MOTOR1 <= RB_MOTOR1 + 2;
				ELSIF(L=62+ie*40 AND MISO='1')THEN
					RB_MOTOR1 <= RB_MOTOR1 + 1;
				------------------------------------------LB	
				ELSIF(L=63+ie*40 AND MISO='1')THEN
					LB_MOTOR1 <= LB_MOTOR1 - 32;
				ELSIF(L=64+ie*40 AND MISO='1')THEN
					LB_MOTOR1 <= LB_MOTOR1 + 16;
				ELSIF(L=65+ie*40 AND MISO='1')THEN
					LB_MOTOR1 <= LB_MOTOR1 + 8;
				ELSIF(L=66+ie*40 AND MISO='1')THEN
					LB_MOTOR1 <= LB_MOTOR1 + 4;
				ELSIF(L=67+ie*40 AND MISO='1')THEN
					LB_MOTOR1 <= LB_MOTOR1 + 2;
				ELSIF(L=68+ie*40 AND MISO='1')THEN
					LB_MOTOR1 <= LB_MOTOR1 + 1;
				------------------------------------------LF	
				ELSIF(L=69+ie*40 AND MISO='1')THEN
					LF_MOTOR1 <= LF_MOTOR1 - 32;
				ELSIF(L=70+ie*40 AND MISO='1')THEN
					LF_MOTOR1 <= LF_MOTOR1 + 16;
				ELSIF(L=71+ie*40 AND MISO='1')THEN
					LF_MOTOR1 <= LF_MOTOR1 + 8;
				ELSIF(L=72+ie*40 AND MISO='1')THEN
					LF_MOTOR1 <= LF_MOTOR1 + 4;
				ELSIF(L=73+ie*40 AND MISO='1')THEN
					LF_MOTOR1 <= LF_MOTOR1 + 2;
				ELSIF(L=74+ie*40 AND MISO='1')THEN
					LF_MOTOR1 <= LF_MOTOR1 + 1;
				END IF;	
					
					
				if( l>74+ie*40 and l<83+ie*40 ) then
				
					
						if( l = 75+ie*40 )then
							speed3 <= -RF_MOTOR1;
							speed2 <= -RB_MOTOR1;
							speed1 <= -LB_MOTOR1;
							speed0 <= -LF_MOTOR1;
							if(-LF_MOTOR1=-LB_MOTOR1 and LB_MOTOR1= -RB_MOTOR1 and RF_MOTOR1=RB_MOTOR1 )then
								LED1<='0';
							END IF;
						end if;
					
						if(l>74+ie*40 and l<78+ie*40)then
							kick_power(ie*40 + 77-l) <= miso;
						elsif(l>77+ie*40 and l<79+ie*40)then
							DirectORchip <= miso;
----					elsif(l>78+i*40 and l<80+i*40)then
----						sb_break <= miso;
						elsif(l>79+ie*40 and l<82+ie*40)then
							penalty_mode(ie*40 + 81-l) <= miso;
----					elsif(l>81+i*40 and l<83+i*40)then--281>282<283
--							force_kick <= miso;
						end if;
					
					RF_MOTOR1 <= 0;
					RB_MOTOR1 <= 0;
					LB_MOTOR1 <= 0;
					LF_MOTOR1 <= 0;
				end if;
				end if;

		--	end loop;
		end if;
		--L=283
		if(l>300 and l<309 )then
			receive_buffer(308-l) <= miso;	--Read Status Register value
		end if;
		
		if( l>308 and l< 317 )then
			fifo_status(316-l) <= miso; 	--Read Fifo_Status Register value
		end if;
		

end if;
end process;

RX_ID(0) <= "11111111" when RX_DATA(239 downto 232)="00000000" else RX_DATA(239 downto 232);
RX_ID(1) <= "11111111" when RX_DATA(199 downto 192)="00000000" else RX_DATA(199 downto 192);
RX_ID(2) <= "11111111" when RX_DATA(159 downto 152)="00000000" else RX_DATA(159 downto 152);
RX_ID(3) <= "11111111" when RX_DATA(119 downto 112)="00000000" else RX_DATA(119 downto 112);
RX_ID(4) <= "11111111" when RX_DATA(79 downto 72)  ="00000000" else RX_DATA(79 downto 72);
RX_ID(5) <= "11111111" when RX_DATA(39 downto 32)  ="00000000" else RX_DATA(39 downto 32);

Receive_Enable 	<= '1' when receive_buffer(6) = '1' else '0';--status(6)=RX_DR
Rx_Full        	<= '0' when fifo_status(0)    = '1' else '1';--FIFO Status(0)=RX_Empty
Tx_Empty        <= '1' when fifo_status(4)    = '1' else '0';--FIFO Status(4)=TX_Empty
DataSent		<= '1' when receive_buffer(5) = '1' else '0';--status(5)=TX_Ds

--IS_PRX <='1';
--IS_PRX<=robot_id(3);


process(sck_int)
begin
	if(rising_edge(sck_int)) then

		if(l>251)then
			if(step="00")then
				if( RX_ID(0)<RX_ID(1) )then
					c1 <= RX_ID(0);
				else
					c1 <= RX_ID(1);
				end if;
				
				if( RX_ID(2)<RX_ID(3) )then
					c2 <= RX_ID(2);
				else
					c2 <= RX_ID(3);
				end if;
				
				if( RX_ID(4)<RX_ID(5) )then
					c3 <= RX_ID(4);
				else
					c3 <= RX_ID(5);
				end if;
				step<="01";
			end if;
			if(step="01")then
				if( c1 <c2)then
					c4<=c1;
				else
					c4<=c2;
				end if;
				step<="10";
			end if;
			if(step="10")then
				if(c4<c3) then
					c5<=c4;
				else
					c5<=c3;
				end if;
				step<="11";
			end if;
			
			if(step="11")then
				if(c5=Robot_ID)then
					if(l>283) then
						IS_PRX<='0';
					end if;
				else
					IS_PRX<='1';
				end if;
				
				if(Robot_ID=RX_ID(0))then
					packet_index<=5;
				end if;
				if(Robot_ID=RX_ID(1))then
					packet_index<=4;
				end if;
				if(Robot_ID=RX_ID(2))then
					packet_index<=3;
				end if;
				if(Robot_ID=RX_ID(3))then
					packet_index<=2;
				end if;
				if(Robot_ID=RX_ID(4))then
					packet_index<=1;
				end if;
				if(Robot_ID=RX_ID(5))then
					packet_index<=0;
				end if;
				step<="00";
			end if;
		end if;
		
		if(l=318) then
			IS_PRX<='1';--Toggle Mode to receiver IF is sender :)
		end if;
		
	end if;
end process;

process(main_clk)--Send Delay
begin
	if(rising_edge(main_clk))then
		if(IS_PRX='0')then
			if(Send_Delay_count<14745)then
				Send_Delay_count<=Send_Delay_count+1;
				is_After_Delay<='0';
			else
				is_After_Delay<='1';
				Send_Delay_count<=14755;
			end if;
		else
			is_After_Delay<='0';
			Send_Delay_count<=0;
		end if;
	end if;
end process;

--process(main_clk)--receive Timeout
--begin
--	if(rising_edge(main_clk))then
--		if(l<33 and IS_PRX='1')then
--			if(Timeout_count < 29491200)then
--				Timeout_count <= Timeout_count+1;
--				IS_Timeout <= '0';
--			else
--				IS_Timeout <= '1';
--				Timeout_count <= 29491255;
--			end if;
--		else
--			IS_Timeout <= '0';
--			Timeout_count <= 0;
--		end if;
--	end if;
--end process;

process(sck_int)
begin
	if(rising_edge(sck_int))then
		if(nRFConectionErr='1') then
			ErrCount <= ErrCount+1;
		elsif(nRFConectionErr = '0') then
			ErrCount <= 0;
		end if;
		if(ErrCount > 20) then
			nRFConectionErr <= '0';
			ErrCount <= 0;
		end if;
		if(l>18 and SetAllRegisterIsDone='1' and IS_PRX='1') then		
			if(ConfigRegVal = "00001111" ) then
				nRFConectionErr <='0';--nRF is Connect...
			else
				nRFConectionErr <='1';--nRF Disconnected!
			end if;
		end if;
		if(l>18 and SetAllRegisterIsDone='1' and IS_PRX='0') then		
			if(ConfigRegVal = "00001110" ) then
				nRFConectionErr <='0';--nRF is Connect...
			else
				nRFConectionErr <='1';--nRF Disconnected!
			end if;
		end if;
	end if;

	if(l>18 and nRFConectionErr='1') then--To Reset nRF...
		l  <= 0;
		kk <= -1;
		SetAllRegisterIsDone <='0';
		ChipEN<='0';
	else
		if(SetAllRegisterIsDone='0')then
			if(rising_edge(sck_int))then
				if(kk < 217) then
					kk <= kk+1;
				else
					SetAllRegisterIsDone<='1';
				end if;
			end if;
		else
			if(rising_edge(sck_int))then
				if(l < 318)then
					if(IS_PRX='0')then
						if(is_After_Delay='1')then
							l <= l+1;
						else
							l<=0;
						end if;
					end if;
					if(IS_PRX='1')then
						l <= l+1;
					end if;
				elsif(l=318)then
					l <= 0;
				end if;
				
				if(IS_PRX='1') then------------IS RECEIVER
					ChipEN<='1';
					if(receive_enable='0' and Rx_Full ='0' and l=33)then--not Receive
						l <= 0;
					end if;
--					if( rx_full ='1' and l > 316 )then
--						l <= 17;
--					end if;
				end if;
				
				if(IS_PRX='0') then-------------IS SENDER
					if(l<43)then
						ChipEN<='0';
					else
						if(l=317 and DataSent='0' and Tx_Empty='0') then--Not Sent :(
							l <=300;
						end if;
						ChipEN<='1';
					end if;
				end if;
			end if;
		end if;--if(SetAllRegisterIsDone='0')
	end if;--if(l>18 and nRFConectionErr='1')
end process;

process(main_clk)
begin
	if(rising_edge(main_clk))then
		if(clk_counter < 31)then
			clk_counter <= clk_counter +1;
		else
			clk_counter <= 0;
		end if;
	end if;
	if(clk_counter>15)then
		sck_int <= '1';
	else
		sck_int <= '0';
	end if;
end process;
---------------------------------
--process(main_clk)
--begin
--if(rising_edge(main_clk))then
--	enable_count <= enable_count +1;
--	if(nRFConectionErr='1') then
--		enable <= '0';
--		enable_count<=0;
--	elsif(enable_count > 29491200)then
--		enable <= '1';
--	end if;
--end if;
--end process;

----==========================================================================



----------------------------------------filter	
	process(main_clk)
	begin
		--------------------------------filter0
		for b in 0 to 9 loop
			if (rising_edge(main_clk)) then
				Battery_bf(b) <= BatteryIsFull;
			end if;
		end loop;
		Battery_f  <= Battery_bf(0) and Battery_bf(1) and Battery_bf(2) and Battery_bf(3) and Battery_bf(4) and Battery_bf(5) and Battery_bf(6) and Battery_bf(7) and Battery_bf(8) and Battery_bf(9);  
		
		--------------------------------filter0
		for i in 0 to 9 loop
			if (rising_edge(main_clk)) then
				clkf01(i) <= a_clk0;
				clkf02(i) <= b_clk0; 
			end if;
		end loop;
		mp01  <= clkf01(0) and clkf01(1) and clkf01(2) and clkf01(3) and clkf01(4) and clkf01(5) and clkf01(6) and clkf01(7) and clkf01(8) and clkf01(9);   
		mp02  <= clkf02(0) and clkf02(1) and clkf02(2) and clkf02(3) and clkf02(4) and clkf02(5) and clkf02(6) and clkf02(7) and clkf02(8) and clkf02(9); 
		
		---------------------------------filter1
		for j in 0 to 9 loop
			if (rising_edge(main_clk)) then
				clkf11(j) <= a_clk1;
				clkf12(j) <= b_clk1; 
			end if;
		end loop;
		mp11  <= clkf11(0) and clkf11(1) and clkf11(2) and clkf11(3) and clkf11(4) and clkf11(5) and clkf11(6) and clkf11(7) and clkf11(8) and clkf11(9);   
		mp12  <= clkf12(0) and clkf12(1) and clkf12(2) and clkf12(3) and clkf12(4) and clkf12(5) and clkf12(6) and clkf12(7) and clkf12(8) and clkf12(9); 
	
		---------------------------------filter2
		for m in 0 to 9 loop
			if (rising_edge(main_clk)) then
				clkf21(m) <= a_clk2;
				clkf22(m) <= b_clk2; 
			end if;
		end loop;
		mp21  <= clkf21(0) and clkf21(1) and clkf21(2) and clkf21(3) and clkf21(4) and clkf21(5) and clkf21(6) and clkf21(7) and clkf21(8) and clkf21(9);   
		mp22  <= clkf22(0) and clkf22(1) and clkf22(2) and clkf22(3) and clkf22(4) and clkf22(5) and clkf22(6) and clkf22(7) and clkf22(8) and clkf22(9); 
	
--		---------------------------------filter3
		for n in 0 to 9 loop
			if (rising_edge(main_clk)) then
				clkf31(n) <= a_clk3;
				clkf32(n) <= b_clk3; 
			end if;
		end loop;
		mp31  <= clkf31(0) and clkf31(1) and clkf31(2) and clkf31(3) and clkf31(4) and clkf31(5) and clkf31(6) and clkf31(7) and clkf31(8) and clkf31(9);   
		mp32  <= clkf32(0) and clkf32(1) and clkf32(2) and clkf32(3) and clkf32(4) and clkf32(5) and clkf32(6) and clkf32(7) and clkf32(8) and clkf32(9); 
	
		
	end process;

----------------------------------------counter
	process( period,mp21,mp01,mp11,mp31)
	
	 begin						-- counter process
		--------------------------------counter0
		if period='1' then                            			-- period = ON ?			
			data01  <=0;
			data02  <=0;	
		elsif (rising_edge(mp01)) then             				-- Clk1 in ?
			if mp02='1' then                      				-- Yes. Up count ?
				data01 <= data01 + 1;              				-- Yes. Count-up
			else                                				-- Not count-up
				data01 <= data01 - 1;              				-- Count-down
			end if;
				
		elsif(falling_edge(mp01)) then						
			if mp02='0' then									-- yes. up count?
				data02 <= data02 + 1;							-- yes. count-up
			else 												-- not count up
				data02 <= data02 - 1;							-- count-down
			end if;
		end if;
		
--		--------------------------------counter1
		if period='1' then                            			-- period = ON ?			
			data11  <=0;
			data12  <=0;	
		elsif (rising_edge(mp11)) then             				-- Clk1 in ?
			if mp12='1' then                      				-- Yes. Up count ?
				data11 <= data11 + 1;              				-- Yes. Count-up
			else                                				-- Not count-up
				data11 <= data11 - 1;              				-- Count-down
			end if;
				
		elsif(falling_edge(mp11)) then						
			if mp12='0' then									-- yes. up count?
				data12 <= data12 + 1;							-- yes. count-up
			else 												-- not count up
				data12 <= data12 - 1;							-- count-down
			end if;
		end if;
		
		---------------------------------counter2
		if period='1' then                            			-- period = ON ?			
			data21  <=0;
			data22  <=0;	
		elsif (rising_edge(mp21)) then             				-- Clk1 in ?
			if mp22='1' then                      				-- Yes. Up count ?
				data21 <= data21 + 1;              				-- Yes. Count-up
			else                                				-- Not count-up
				data21 <= data21 - 1;              				-- Count-down
			end if;
				
		elsif(falling_edge(mp21)) then						
			if mp22='0' then									-- yes. up count?
				data22 <= data22 + 1;							-- yes. count-up
			else 												-- not count up
				data22 <= data22 - 1;							-- count-down
			end if;
		end if;
		
--		----------------------------------counter3
		if period='1' then                            			-- period = ON ?			
			data31  <=0;
			data32  <=0;	
		elsif (rising_edge(mp31)) then             				-- Clk1 in ?
			if mp32='1' then                      				-- Yes. Up count ?
				data31 <= data31 + 1;              				-- Yes. Count-up
			else                                				-- Not count-up
				data31 <= data31 - 1;              				-- Count-down
			end if;
				
		elsif(falling_edge(mp31)) then						
			if mp32='0' then									-- yes. up count?
				data32 <= data32 + 1;							-- yes. count-up
			else 												-- not count up
				data32 <= data32 - 1;							-- count-down
			end if;
		end if;
			
	end process;
	

----------------------------------------pi	
--	process(period) 
--	begin
----		--------------------------------pi0
--		if (rising_edge(period))then									-- set error
--			if(setpoint0 /= 0)then
--				if((idata0 + error0 * ki0 < 32000) and (idata0 + error0* ki0 > -32000) )then
----					if(idata0 + error0 * ki0 > 10)then
----						idata0 <= idata0 + error0 * ki0 - 10;
----					elsif ( idata0 + error0 * ki0 <  -10)then
----						idata0 <= idata0 + error0 * ki0 + 10;
----					else
--						idata0 <= idata0 + error0 * ki0;
----					end if;
--				end if;
--			else
--				idata0 <= 0;
--			end if;
--				
--			if(error0 * kp0 < 32000 and error0 * kp0 > -32000)then
--				p_data_out0 <= error0 * kp0;
--			elsif( error0 * kp0 > 32000)then
--				p_data_out0 <= 32000;
--			elsif (error0 * kp0 < -32000) then
--				p_data_out0 <= -32000;
--			end if;
----			
----			
--			if(p_data_out0 + idata0 < 32000 and p_data_out0 + idata0 > -32000)then
--				pi_data_out0 <= (p_data_out0 + idata0)/300;
--			elsif(p_data_out0 + idata0 > 32000)then
--				pi_data_out0 <= 32000;
--			elsif (p_data_out0 + idata0 < -32000) then
--				pi_data_out0 <= -32000;
--			end if;
--			if(setpoint0 = 0) then
--				pi_data_out0<=0;
--			end if;
--		end if;	
----		
----		--------------------------------pi1
--		if (rising_edge(period))then									-- set error
--			if(setpoint1 /= 0)then
--				if((idata1 + error1 * ki1 < 32000) and (idata1 + error1* ki1 > -32000) )then
----					if(idata1 + error1 * ki1 > 10)then
----						idata1 <= idata1 + error1 * ki1 - 10;
----					elsif ( idata1 + error1 * ki1 <  -10)then
----						idata1 <= idata1 + error1 * ki1 + 10;
----					else
--						idata1 <= idata1 + error1 * ki1;
----					end if;
--				end if;
--			else
--				idata1 <= 0;
--			end if;
--			if((error1 * kp1 <32000) and (error1 * kp1>-32000))then
--			p_data_out1 <= error1 * kp1;
--			elsif( error1 * kp1 > 32000)then
--				p_data_out1 <= 32000;
--			elsif (error1 * kp1 < -32000) then
--				p_data_out1 <= -32000;
--			end if;
--			if(p_data_out1 + idata1 < 32000 and p_data_out1 + idata1 > -32000)then
--				pi_data_out1 <= (p_data_out1 + idata1)/300;
--				elsif(p_data_out1 + idata1 > 32000)then
--				pi_data_out1 <= 32000;
--			elsif (p_data_out1 + idata1 < -32000) then
--				pi_data_out1 <= -32000;
--			end if;
--			if(setpoint1 = 0) then
--				pi_data_out1<=0;
--			end if;
--		end if;
--	
--		--------------------------------pi2
--		if (rising_edge(period))then									-- set error
--			if(setpoint2 /= 0)then
--				if((idata2 + error2 * ki2 < 32000) and (idata2 + error2* ki2 > -32000) )then
----					if(idata2 + error2 * ki2 > 10)then
----						idata2 <= idata2 + error2 * ki2 - 10;
----					elsif ( idata2 + error2 * ki2 <  -10)then
----						idata2 <= idata2 + error2 * ki2 + 10;
----					else
--						idata2 <= idata2 + error2 * ki2;
----					end if;
--				end if;
--			else
--				idata2 <= 0;
--			end if;
--			if((error2 * kp2 <32000) and (error2 * kp2>-32000))then
--			p_data_out2 <= error2 * kp2;
--			elsif( error2 * kp2 > 32000)then
--				p_data_out2 <= 32000;
--			elsif (error2 * kp2 < -32000) then
--				p_data_out2 <= -32000;
--			end if;
--			if(p_data_out2 + idata2 < 32000 and p_data_out2 + idata2 > -32000)then
--				pi_data_out2 <= (p_data_out2 + idata2)/300;
--			elsif(p_data_out2 + idata2 > 32000)then
--				pi_data_out2 <= 32000;
--			elsif (p_data_out2 + idata2 < -32000) then
--				pi_data_out2 <= -32000;
--			end if;
--			if(setpoint2 = 0) then
--				pi_data_out2<=0;
--			end if;
--		end if;
--		
----		-------------------------------pi3
--		if (rising_edge(period))then									-- set error
--			if(setpoint3 /= 0)then
--				if((idata3 + error3 * ki3 < 32000) and (idata3 + error3* ki3 > -32000) )then
----					if(idata3 + error3 * ki3> 10)then
----						idata3 <= idata3 + error3 * ki3 - 10;
----					elsif ( idata3 + error3 * ki3 <  -10)then
----						idata3 <= idata3 + error3 * ki3 + 10;
----					else
--						idata3 <= idata3 + error3 * ki3;
----					end if;
--				end if;
--			else 
--				idata3 <= 0;
--			end if;
--			
--			if((error3 * kp3 <32000) and (error3 * kp3>-32000))then
--				p_data_out3 <= error3 * kp3;
--			elsif( error3 * kp3 > 32000)then
--				p_data_out3 <= 32000;
--			elsif (error3 * kp3 < -32000) then
--				p_data_out3 <= -32000;
--			end if;
--			if(p_data_out3 + idata3 < 32000 and p_data_out3 + idata3 > -32000)then
--				pi_data_out3 <= (p_data_out3 + idata3)/300;
--			elsif(p_data_out3 + idata3 > 32000)then
--				pi_data_out3 <= 32000;
--			elsif (p_data_out3 + idata3 < -32000) then
--				pi_data_out3 <= -32000;
--			end if;
--			if(setpoint3 = 0) then
--				pi_data_out3<=0;
--			end if;
--		end if;
--	end process;

process(period) 
	begin
--		--------------------------------pi0
		if (rising_edge(period))then									-- set error
			if(setpoint0 /= 0)then
				if((idata0 + error0 * ki0 < 45000) and (idata0 + error0* ki0 > -45000) )then
--					if(idata0 + error0 * ki0 > 10)then
--						idata0 <= idata0 + error0 * ki0 - 10;
--					elsif ( idata0 + error0 * ki0 <  -10)then
--						idata0 <= idata0 + error0 * ki0 + 10;
--					else
						idata0 <= idata0 + error0 * ki0;
--					end if;
				elsif(idata0>45000)then
					idata0<=45000;
				elsif(idata0<-45000)then
					idata0<=-45000;
				end if;
			else
				idata0 <= 0;
			end if;
				
			if(error0 * kp0 < 45000 and error0 * kp0 > -45000)then
				p_data_out0 <= error0 * kp0;
			elsif( error0 * kp0 > 45000)then
				p_data_out0 <= 45000;
			elsif (error0 * kp0 < -45000) then
				p_data_out0 <= -45000;
			end if;
--			
--			
			if(p_data_out0 + idata0 < 45000 and p_data_out0 + idata0 > -45000)then
				pi_data_out0 <= (p_data_out0 + idata0)/300;
			elsif(p_data_out0 + idata0 > 45000)then
				pi_data_out0 <= 150;
			elsif (p_data_out0 + idata0 < -45000) then
				pi_data_out0 <= -150;
			end if;
			if(setpoint0 = 0) then
				pi_data_out0<=0;
			end if;
		end if;	
--		
--		--------------------------------pi1
		if (rising_edge(period))then									-- set error
			if(setpoint1 /= 0)then
				if((idata1 + error1 * ki1 < 45000) and (idata1 + error1* ki1 > -45000) )then
--					if(idata1 + error1 * ki1 > 10)then
--						idata1 <= idata1 + error1 * ki1 - 10;
--					elsif ( idata1 + error1 * ki1 <  -10)then
--						idata1 <= idata1 + error1 * ki1 + 10;
--					else
						idata1 <= idata1 + error1 * ki1;
				elsif(idata1>45000)then
					idata1<=45000;
				elsif(idata1<-45000)then
					idata1<=-45000;
				end if;
--					end if;

			else
				idata1 <= 0;
			end if;
			if((error1 * kp1 <45000) and (error1 * kp1>-45000))then
			p_data_out1 <= error1 * kp1;
			elsif( error1 * kp1 > 45000)then
				p_data_out1 <= 45000;
			elsif (error1 * kp1 < -45000) then
				p_data_out1 <= -45000;
			end if;
			if(p_data_out1 + idata1 < 45000 and p_data_out1 + idata1 > -45000)then
				pi_data_out1 <= (p_data_out1 + idata1)/300;
				elsif(p_data_out1 + idata1 > 45000)then
				pi_data_out1 <=150;
			elsif (p_data_out1 + idata1 < -45000) then
				pi_data_out1 <= -150;
			end if;
			if(setpoint1 = 0) then
				pi_data_out1<=0;
			end if;
		end if;
	
		--------------------------------pi2
		if (rising_edge(period))then									-- set error
			if(setpoint2 /= 0)then
				if((idata2 + error2 * ki2 < 45000) and (idata2 + error2* ki2 > -45000) )then
--					if(idata2 + error2 * ki2 > 10)then
--						idata2 <= idata2 + error2 * ki2 - 10;
--					elsif ( idata2 + error2 * ki2 <  -10)then
--						idata2 <= idata2 + error2 * ki2 + 10;
--					else
						idata2 <= idata2 + error2 * ki2;
				elsif(idata2>45000)then
					idata2<=45000;
				elsif(idata2<-45000)then
					idata2<=-45000;
				end if;
--					end if;
			else
				idata2 <= 0;
			end if;
			if((error2 * kp2 <45000) and (error2 * kp2>-45000))then
			p_data_out2 <= error2 * kp2;
			elsif( error2 * kp2 > 45000)then
				p_data_out2 <= 45000;
			elsif (error2 * kp2 < -45000) then
				p_data_out2 <= -45000;
			end if;
			if(p_data_out2 + idata2 < 45000 and p_data_out2 + idata2 > -45000)then
				pi_data_out2 <= (p_data_out2 + idata2)/300;
			elsif(p_data_out2 + idata2 > 45000)then
				pi_data_out2 <= 150;
			elsif (p_data_out2 + idata2 < -45000) then
				pi_data_out2 <= -150;
			end if;
			if(setpoint2 = 0) then
				pi_data_out2<=0;
			end if;
		end if;
		
--		-------------------------------pi3
		if (rising_edge(period))then									-- set error
			if(setpoint3 /= 0)then
				if((idata3 + error3 * ki3 < 45000) and (idata3 + error3* ki3 > -45000) )then
--					if(idata3 + error3 * ki3> 10)then
--						idata3 <= idata3 + error3 * ki3 - 10;
--					elsif ( idata3 + error3 * ki3 <  -10)then
--						idata3 <= idata3 + error3 * ki3 + 10;
--					else
						idata3 <= idata3 + error3 * ki3;
				elsif(idata3>45000)then
					idata3<=45000;
				elsif(idata3<-45000)then
					idata3<=-45000;
--					end if;
				end if;
			else 
				idata3 <= 0;
			end if;
			
			if((error3 * kp3 <45000) and (error3 * kp3>-45000))then
				p_data_out3 <= error3 * kp3;
			elsif( error3 * kp3 > 45000)then
				p_data_out3 <= 45000;
			elsif (error3 * kp3 < -45000) then
				p_data_out3 <= -45000;
			end if;
			if(p_data_out3 + idata3 < 45000 and p_data_out3 + idata3 > -45000)then
				pi_data_out3 <= (p_data_out3 + idata3)/300;
			elsif(p_data_out3 + idata3 > 45000)then
				pi_data_out3 <= 150;
			elsif (p_data_out3 + idata3 < -45000) then
				pi_data_out3 <= -150;
			end if;
			if(setpoint3 = 0) then
				pi_data_out3<=0;
			end if;
		end if;
	end process;
----------------------------------------pwm clock	
	process(main_clk)
	begin
		if(rising_edge(main_clk)) then
			k<=k+1;
			if k>6 then
				if(k=8) then
					k<=0;
				end if;
				
				pwm_clk<='1';
			else
				pwm_clk<='0';
			end if;
		end if;
	end process;
----------------------------------------pwm	

	
	process(pwm_clk)
	begin
		--------------------------------pwm data
--		if(halt_flag = 0)then	
			if (rising_edge(pwm_clk)) then
				pwm_data<=pwm_data + 1;
					if(pwm_data>240)then
						pwm_data<=0;
					end if;
			end if;
			


------------------------------------------------Direction0
			if (pi_data_out0<0) then
				LF_direction<='1';
			else
				LF_direction<='0';
			end if;
------------------------------------------------PWM0
			if( abs(pi_data_out0) > pwm_data)then
				pwm0<='1';
			else
				pwm0 <='0';
			end if;	
--==============================================PWM0 Limiter
--		if(abs(pi_data_out0)>120) then	--MoreThanLimit
--			LF_overCounter <=LF_overCounter+1;		
--			if(LF_overCounter>2) then
--				if( abs(120) > pwm_data)then
--					pwm0<='1';
--				else
--					pwm0 <='0';
--				end if;
--			else
--				if( abs(pi_data_out0) > pwm_data)then
--					pwm0<='1';
--				else
--					pwm0 <='0';
--				end if;	
--			end if;
--		else							--LessThanLimit
--			LF_overCounter<=0;
--			if( abs(pi_data_out0) > pwm_data)then
--				pwm0<='1';
--			else
--				pwm0 <='0';
--			end if;	
--		end if;
--===============================================
-------------------------------------------------Direction1
			if (pi_data_out1<0) then
				LB_direction<='1';
			else
				LB_direction<='0';
			end if;
--------------------------------------------------PWM1
			if( abs(pi_data_out1) > pwm_data)then
				pwm1<='1';
			else
				pwm1 <='0';
			end if;	
--================================================PWM1 Limiter
--		if(abs(pi_data_out1)>120) then	--MoreThanLimit
--			LB_overCounter <=LB_overCounter+1;		
--			if(LB_overCounter>2) then
--				if( abs(120) > pwm_data)then
--					pwm1<='1';
--				else
--					pwm1 <='0';
--				end if;
--			else
--				if( abs(pi_data_out1) > pwm_data)then
--					pwm1<='1';
--				else
--					pwm1 <='0';
--				end if;	
--			end if;
--		else							--LessThanLimit
--			LB_overCounter<=0;
--			if( abs(pi_data_out1) > pwm_data)then
--				pwm1<='1';
--			else
--				pwm1 <='0';
--			end if;	
--		end if;
--================================================
--------------------------------------------------Direction2
			if (pi_data_out2<0) then
				RB_direction<='1';
			else
				RB_direction<='0';
			end if;
--------------------------------------------------PWM2
			if( abs(pi_data_out2) > pwm_data)then
				pwm2<='1';
			else
				pwm2 <='0';
			end if;
--================================================PWM2 Limiter
--		if(abs(pi_data_out2)>120) then	--MoreThanLimit
--			RB_overCounter <=RB_overCounter+1;		
--			if(RB_overCounter>2) then
--				if( abs(120) > pwm_data)then
--					pwm2<='1';
--				else
--					pwm2 <='0';
--				end if;
--			else
--				if( abs(pi_data_out2) > pwm_data)then
--					pwm2<='1';
--				else
--					pwm2 <='0';
--				end if;	
--			end if;
--		else							--LessThanLimit
--			RB_overCounter<=0;
--			if( abs(pi_data_out2) > pwm_data)then
--				pwm2<='1';
--			else
--				pwm2 <='0';
--			end if;	
--		end if;
--=================================================	
---------------------------------------------------Direction3
			if (pi_data_out3<0) then
				RF_direction<='1';
			else
				RF_direction<='0';
			end if;
----------------------------------------------------PWM3
			if( abs(pi_data_out3) > pwm_data)then
				pwm3<='1';
			else
				pwm3 <='0';
			end if;	
----------------------------------------------------
--==================================================PWM3 Limiter
--		if(abs(pi_data_out3)>120) then	--MoreThanLimit
--			RF_overCounter <=RF_overCounter+1;		
--			if(RF_overCounter>2) then
--				if( abs(120) > pwm_data)then
--					pwm3<='1';
--				else
--					pwm3 <='0';
--				end if;
--			else
--				if( abs(pi_data_out3) > pwm_data)then
--					pwm3<='1';
--				else
--					pwm3 <='0';
--				end if;	
--			end if;
--		else							--LessThanLimit
--			RF_overCounter<=0;
--			if( abs(pi_data_out3) > pwm_data)then
--				pwm3<='1';
--			else
--				pwm3 <='0';
--			end if;	
--		end if;
--==================================================
--	else
--		pwm0<='0';
--		pwm1<='0';
--		pwm2<='0';
--		pwm3<='0';
--	end if;
	end process;
	
process(pwm_clk)
begin
--	if(halt_flag = 0)then	
		if (rising_edge(pwm_clk)) then
			sb_pwm_data <= sb_pwm_data + 1;
		end if;
		if(sb_pwm > sb_pwm_data)then
			sb <= '1';
		else
			sb <= '0';
		end if;
--	end if;
end process;
	
----------------------------------------period	
	process(main_clk)
	begin
		if(rising_edge(main_clk)) then
			h<=h+'1';
			if h>30000 then
				if( h=32535)then
				h<="0000000000000000";
				end if;
				period<='1';
			else
				period<='0';
			end if;
		end if;
	end process;
	
	
--======================================================================================+
--=======================================================================|LF Comutation |
--======================================================================================+
process(main_clk)
begin
if(setpoint0=0) then
	LF_MOSFET_O <= "111000";
	LF_aa <=Delay_LF+1;
	LF_bb <=Delay_LF+1;
	LF_cc <=Delay_LF+1;
	LF_dd <=Delay_LF+1;
	LF_ee <=Delay_LF+1;
	LF_ff <=Delay_LF+1;
	LF_aaa <=Delay_LF+1;
	LF_bbb <=Delay_LF+1;
	LF_ccc <=Delay_LF+1;
	LF_ddd <=Delay_LF+1;
	LF_eee <=Delay_LF+1;
	LF_fff <=Delay_LF+1;
	
elsif(LF_direction='0')THEN--FW
	if(LF_HALL_I="100" )then
		if(LF_aa>Delay_LF) then
			LF_MOSFET_O(3) <= '1' and pwm0;
		else
			LF_aa <=LF_aa+1;
			LF_bb <=0;
			LF_cc <=0;
			LF_dd <=0;
			LF_ee <=0;
			LF_ff <=0;
		end if;
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '1' and pwm0;
		LF_MOSFET_O(0) <= '0';
	elsif(LF_HALL_I="110" ) then
		if(LF_bb>Delay_LF) then
			LF_MOSFET_O(1) <= '1' and pwm0;
		else
			LF_aa <=0;
			LF_bb <=LF_bb+1;
			LF_cc <=0;
			LF_dd <=0;
			LF_ee <=0;
			LF_ff <=0;
		end if;
		LF_MOSFET_O(5) <= '1' and pwm0;
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(0) <= '0';
	elsif(LF_HALL_I="010" ) then
		if(LF_cc>Delay_LF) then
			LF_MOSFET_O(5) <= '1' and pwm0;
		else
			LF_aa <=0;
			LF_bb <=0;
			LF_cc <=LF_cc+1;
			LF_dd <=0;
			LF_ee <=0;
			LF_ff <=0;
		end if;
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '1' and pwm0;
	elsif(LF_HALL_I="011" ) then
		if(LF_dd>Delay_LF) then
			LF_MOSFET_O(0) <= '1' and pwm0;
		else
			LF_aa<=0;
			LF_bb<=0;
			LF_cc<=0;
			LF_dd<=LF_dd+1;
			LF_ee<=0;
			LF_ff<=0;
		end if;
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '1' and pwm0;
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
	elsif(LF_HALL_I="001" ) then
		if(LF_ee>Delay_LF) then
			LF_MOSFET_O(4) <= '1' and pwm0;
		else
			LF_aa <=0;
			LF_bb <=0;
			LF_cc <=0;
			LF_dd <=0;
			LF_ee <=LF_ee+1;
			LF_ff <=0;
		end if;
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '1' and pwm0;
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
	elsif(LF_HALL_I="101" ) then
		if(LF_ff>Delay_LF) then
			LF_MOSFET_O(2) <= '1' and pwm0;
		else
			LF_aa <=0;
			LF_bb <=0;
			LF_cc <=0;
			LF_dd <=0;
			LF_ee <=0;
			LF_ff <=LF_ff+1;
		end if;
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '1' and pwm0;
	elsif(LF_HALL_I="111" ) then
	    LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
	elsif(LF_HALL_I="000" ) then
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
	end if;	
	
ELSIF(LF_DIRECTION='1')THEN
	if(LF_HALL_I="100" )then
		
		if(LF_aaa>Delay_LF) then
			LF_MOSFET_O(3) <= '1' and pwm0;
		else
			LF_aaa <=LF_aaa+1;
			LF_bbb <=0;
			LF_ccc <=0;
			LF_ddd <=0;
			LF_eee <=0;
			LF_fff <=0;
		end if;
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(2) <= '1' and pwm0;
    
	elsif(LF_HALL_I="110" ) then
		if(LF_bbb>Delay_LF) then
			LF_MOSFET_O(1) <= '1' and pwm0;
		else
			LF_aaa <=0;
			LF_bbb <=LF_bbb+1;
			LF_ccc <=0;
			LF_ddd <=0;
			LF_eee <=0;
			LF_fff <=0;
		end if;
		
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(3) <= '1' and pwm0;
		LF_MOSFET_O(0) <= '0';  

	elsif(LF_HALL_I="010" ) then
		if(LF_ccc>Delay_LF) then
			LF_MOSFET_O(5) <= '1' and pwm0;
		else
			LF_aaa <=0;
			LF_bbb <=0;
			LF_ccc <=LF_ccc+1;
			LF_ddd <=0;
			LF_eee <=0;
			LF_fff <=0;
		end if;
		
		LF_MOSFET_O(1) <= '1' and pwm0;
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(0) <= '0';

	elsif(LF_HALL_I="011" ) then
		if(LF_ddd>Delay_LF) then
			LF_MOSFET_O(0) <= '1'  and pwm0;
		else
			LF_aaa <=0;
			LF_bbb <=0;
			LF_ccc <=0;
			LF_ddd <=LF_ddd+1;
			LF_eee <=0;
			LF_fff <=0;
		end if;
		
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0' ;
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(5) <= '1'  and pwm0;
			
	elsif(LF_HALL_I="001" ) then
		if(LF_eee>Delay_LF) then
			LF_MOSFET_O(4) <= '1'  and pwm0;
		else
			LF_aaa <=0;
			LF_bbb <=0;
			LF_ccc <=0;
			LF_ddd <=0;
			LF_eee <=LF_eee+1;
			LF_fff <=0;
		end if;
		
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(0) <= '1'  and pwm0;
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
		
	elsif(LF_HALL_I="101" ) then
		if(LF_fff>Delay_LF) then
			LF_MOSFET_O(2) <= '1' and pwm0;
		else
			LF_aaa <=0;
			LF_bbb <=0;
			LF_ccc <=0;
			LF_ddd <=0;
			LF_eee <=0;
			LF_fff <=LF_fff+1;
		end if;
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(4) <= '1' and pwm0;
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
			
	elsif(LF_HALL_I="111" ) then
			
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
	elsif(LF_HALL_I="000" ) then
	
		LF_MOSFET_O(5) <= '0';
		LF_MOSFET_O(4) <= '0';
		LF_MOSFET_O(3) <= '0';
		LF_MOSFET_O(2) <= '0';
		LF_MOSFET_O(1) <= '0';
		LF_MOSFET_O(0) <= '0';
	end if;	 
END IF;
end process;

--======================================================================================+
--=======================================================================|LB Comutation |
--======================================================================================+
process(main_clk)
begin
if(setpoint1=0) then
	LB_MOSFET_O <= "111000";
	LB_aa <=Delay_LB+1;
	LB_bb <=Delay_LB+1;
	LB_cc <=Delay_LB+1;
	LB_dd <=Delay_LB+1;
	LB_ee <=Delay_LB+1;
	LB_ff <=Delay_LB+1;
	LB_aaa <=Delay_LB+1;
	LB_bbb <=Delay_LB+1;
	LB_ccc <=Delay_LB+1;
	LB_ddd <=Delay_LB+1;
	LB_eee <=Delay_LB+1;
	LB_fff <=Delay_LB+1;
	
elsif(LB_direction='0')THEN--FW
	if(LB_HALL_I="100" )then
		if(LB_aa>Delay_LB) then
			LB_MOSFET_O(3) <= '1' and pwm1;
		else
			LB_aa <=LB_aa+1;
			LB_bb <=0;
			LB_cc <=0;
			LB_dd <=0;
			LB_ee <=0;
			LB_ff <=0;
		end if;
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '1' and pwm1;
		LB_MOSFET_O(0) <= '0';
	elsif(LB_HALL_I="110" ) then
		if(LB_bb>Delay_LB) then
			LB_MOSFET_O(1) <= '1' and pwm1;
		else
			LB_aa <=0;
			LB_bb <=LB_bb+1;
			LB_cc <=0;
			LB_dd <=0;
			LB_ee <=0;
			LB_ff <=0;
		end if;
		LB_MOSFET_O(5) <= '1' and pwm1;
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(0) <= '0';
	elsif(LB_HALL_I="010" ) then
		if(LB_cc>Delay_LB) then
			LB_MOSFET_O(5) <= '1' and pwm1;
		else
			LB_aa <=0;
			LB_bb <=0;
			LB_cc <=LB_cc+1;
			LB_dd <=0;
			LB_ee <=0;
			LB_ff <=0;
		end if;
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '1' and pwm1;
	elsif(LB_HALL_I="011" ) then
		if(LB_dd>Delay_LB) then
			LB_MOSFET_O(0) <= '1' and pwm1;
		else
			LB_aa<=0;
			LB_bb<=0;
			LB_cc<=0;
			LB_dd<=LB_dd+1;
			LB_ee<=0;
			LB_ff<=0;
		end if;
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '1' and pwm1;
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
	elsif(LB_HALL_I="001" ) then
		if(LB_ee>Delay_LB) then
			LB_MOSFET_O(4) <= '1' and pwm1;
		else
			LB_aa <=0;
			LB_bb <=0;
			LB_cc <=0;
			LB_dd <=0;
			LB_ee <=LB_ee+1;
			LB_ff <=0;
		end if;
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '1' and pwm1;
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
	elsif(LB_HALL_I="101" ) then
		if(LB_ff>Delay_LB) then
			LB_MOSFET_O(2) <= '1' and pwm1;
		else
			LB_aa <=0;
			LB_bb <=0;
			LB_cc <=0;
			LB_dd <=0;
			LB_ee <=0;
			LB_ff <=LB_ff+1;
		end if;
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '1' and pwm1;
	elsif(LB_HALL_I="111" ) then
	    LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
	elsif(LB_HALL_I="000" ) then
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
	end if;	
	
ELSIF(LB_DIRECTION='1')THEN
	if(LB_HALL_I="100" )then
		
		if(LB_aaa>Delay_LB) then
			LB_MOSFET_O(3) <= '1' and pwm1;
		else
			LB_aaa <=LB_aaa+1;
			LB_bbb <=0;
			LB_ccc <=0;
			LB_ddd <=0;
			LB_eee <=0;
			LB_fff <=0;
		end if;
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(2) <= '1' and pwm1;
    
	elsif(LB_HALL_I="110" ) then
		if(LB_bbb>Delay_LB) then
			LB_MOSFET_O(1) <= '1' and pwm1;
		else
			LB_aaa <=0;
			LB_bbb <=LB_bbb+1;
			LB_ccc <=0;
			LB_ddd <=0;
			LB_eee <=0;
			LB_fff <=0;
		end if;
		
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(3) <= '1' and pwm1;
		LB_MOSFET_O(0) <= '0';  

	elsif(LB_HALL_I="010" ) then
		if(LB_ccc>Delay_LB) then
			LB_MOSFET_O(5) <= '1' and pwm1;
		else
			LB_aaa <=0;
			LB_bbb <=0;
			LB_ccc <=LB_ccc+1;
			LB_ddd <=0;
			LB_eee <=0;
			LB_fff <=0;
		end if;
		
		LB_MOSFET_O(1) <= '1' and pwm1;
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(0) <= '0';

	elsif(LB_HALL_I="011" ) then
		if(LB_ddd>Delay_LB) then
			LB_MOSFET_O(0) <= '1'  and pwm1;
		else
			LB_aaa <=0;
			LB_bbb <=0;
			LB_ccc <=0;
			LB_ddd <=LB_ddd+1;
			LB_eee <=0;
			LB_fff <=0;
		end if;
		
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0' ;
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(5) <= '1'  and pwm1;
			
	elsif(LB_HALL_I="001" ) then
		if(LB_eee>Delay_LB) then
			LB_MOSFET_O(4) <= '1'  and pwm1;
		else
			LB_aaa <=0;
			LB_bbb <=0;
			LB_ccc <=0;
			LB_ddd <=0;
			LB_eee <=LB_eee+1;
			LB_fff <=0;
		end if;
		
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(0) <= '1'  and pwm1;
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
		
	elsif(LB_HALL_I="101" ) then
		if(LB_fff>Delay_LB) then
			LB_MOSFET_O(2) <= '1' and pwm1;
		else
			LB_aaa <=0;
			LB_bbb <=0;
			LB_ccc <=0;
			LB_ddd <=0;
			LB_eee <=0;
			LB_fff <=LB_fff+1;
		end if;
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(4) <= '1' and pwm1;
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
			
	elsif(LB_HALL_I="111" ) then
			
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
	elsif(LB_HALL_I="000" ) then
	
		LB_MOSFET_O(5) <= '0';
		LB_MOSFET_O(4) <= '0';
		LB_MOSFET_O(3) <= '0';
		LB_MOSFET_O(2) <= '0';
		LB_MOSFET_O(1) <= '0';
		LB_MOSFET_O(0) <= '0';
	end if;	 
END IF;
end process;

--======================================================================================+
--=======================================================================|RB Comutation |
--======================================================================================+
process(main_clk)
begin
if(setpoint2=0) then
	RB_MOSFET_O <= "111000";
	RB_aa <=Delay_RB+1;
	RB_bb <=Delay_RB+1;
	RB_cc <=Delay_RB+1;
	RB_dd <=Delay_RB+1;
	RB_ee <=Delay_RB+1;
	RB_ff <=Delay_RB+1;
	RB_aaa <=Delay_RB+1;
	RB_bbb <=Delay_RB+1;
	RB_ccc <=Delay_RB+1;
	RB_ddd <=Delay_RB+1;
	RB_eee <=Delay_RB+1;
	RB_fff <=Delay_RB+1;
			
elsif(RB_direction='0')THEN--FW
	if(RB_HALL_I="100" )then
		if(RB_aa>Delay_RB) then
			RB_MOSFET_O(3) <= '1' and pwm2;
		else
			RB_aa <=RB_aa+1;
			RB_bb <=0;
			RB_cc <=0;
			RB_dd <=0;
			RB_ee <=0;
			RB_ff <=0;
		end if;
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '1' and pwm2;
		RB_MOSFET_O(0) <= '0';
	elsif(RB_HALL_I="110" ) then
		if(RB_bb>Delay_RB) then
			RB_MOSFET_O(1) <= '1' and pwm2;
		else
			RB_aa <=0;
			RB_bb <=RB_bb+1;
			RB_cc <=0;
			RB_dd <=0;
			RB_ee <=0;
			RB_ff <=0;
		end if;
		RB_MOSFET_O(5) <= '1' and pwm2;
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(0) <= '0';
	elsif(RB_HALL_I="010" ) then
		if(RB_cc>Delay_RB) then
			RB_MOSFET_O(5) <= '1' and pwm2;
		else
			RB_aa <=0;
			RB_bb <=0;
			RB_cc <=RB_cc+1;
			RB_dd <=0;
			RB_ee <=0;
			RB_ff <=0;
		end if;
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '1' and pwm2;
	elsif(RB_HALL_I="011" ) then
		if(RB_dd>Delay_RB) then
			RB_MOSFET_O(0) <= '1' and pwm2;
		else
			RB_aa<=0;
			RB_bb<=0;
			RB_cc<=0;
			RB_dd<=RB_dd+1;
			RB_ee<=0;
			RB_ff<=0;
		end if;
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '1' and pwm2;
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
	elsif(RB_HALL_I="001" ) then
		if(RB_ee>Delay_RB) then
			RB_MOSFET_O(4) <= '1' and pwm2;
		else
			RB_aa <=0;
			RB_bb <=0;
			RB_cc <=0;
			RB_dd <=0;
			RB_ee <=RB_ee+1;
			RB_ff <=0;
		end if;
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '1' and pwm2;
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
	elsif(RB_HALL_I="101" ) then
		if(RB_ff>Delay_RB) then
			RB_MOSFET_O(2) <= '1' and pwm2;
		else
			RB_aa <=0;
			RB_bb <=0;
			RB_cc <=0;
			RB_dd <=0;
			RB_ee <=0;
			RB_ff <=RB_ff+1;
		end if;
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '1' and pwm2;
	elsif(RB_HALL_I="111" ) then
	    RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
	elsif(RB_HALL_I="000" ) then
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
	end if;	
	
ELSIF(RB_DIRECTION='1')THEN
	if(RB_HALL_I="100" )then
		
		if(RB_aaa>Delay_RB) then
			RB_MOSFET_O(3) <= '1' and pwm2;
		else
			RB_aaa <=RB_aaa+1;
			RB_bbb <=0;
			RB_ccc <=0;
			RB_ddd <=0;
			RB_eee <=0;
			RB_fff <=0;
		end if;
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(2) <= '1' and pwm2;
    
	elsif(RB_HALL_I="110" ) then
		if(RB_bbb>Delay_RB) then
			RB_MOSFET_O(1) <= '1' and pwm2;
		else
			RB_aaa <=0;
			RB_bbb <=RB_bbb+1;
			RB_ccc <=0;
			RB_ddd <=0;
			RB_eee <=0;
			RB_fff <=0;
		end if;
		
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(3) <= '1' and pwm2;
		RB_MOSFET_O(0) <= '0';  

	elsif(RB_HALL_I="010" ) then
		if(RB_ccc>Delay_RB) then
			RB_MOSFET_O(5) <= '1' and pwm2;
		else
			RB_aaa <=0;
			RB_bbb <=0;
			RB_ccc <=RB_ccc+1;
			RB_ddd <=0;
			RB_eee <=0;
			RB_fff <=0;
		end if;
		
		RB_MOSFET_O(1) <= '1' and pwm2;
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(0) <= '0';

	elsif(RB_HALL_I="011" ) then
		if(RB_ddd>Delay_RB) then
			RB_MOSFET_O(0) <= '1'  and pwm2;
		else
			RB_aaa <=0;
			RB_bbb <=0;
			RB_ccc <=0;
			RB_ddd <=RB_ddd+1;
			RB_eee <=0;
			RB_fff <=0;
		end if;
		
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0' ;
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(5) <= '1'  and pwm2;
			
	elsif(RB_HALL_I="001" ) then
		if(RB_eee>Delay_RB) then
			RB_MOSFET_O(4) <= '1'  and pwm2;
		else
			RB_aaa <=0;
			RB_bbb <=0;
			RB_ccc <=0;
			RB_ddd <=0;
			RB_eee <=RB_eee+1;
			RB_fff <=0;
		end if;
		
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(0) <= '1'  and pwm2;
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
		
	elsif(RB_HALL_I="101" ) then
		if(RB_fff>Delay_RB) then
			RB_MOSFET_O(2) <= '1' and pwm2;
		else
			RB_aaa <=0;
			RB_bbb <=0;
			RB_ccc <=0;
			RB_ddd <=0;
			RB_eee <=0;
			RB_fff <=RB_fff+1;
		end if;
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(4) <= '1' and pwm2;
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
			
	elsif(RB_HALL_I="111" ) then
			
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
	elsif(RB_HALL_I="000" ) then
	
		RB_MOSFET_O(5) <= '0';
		RB_MOSFET_O(4) <= '0';
		RB_MOSFET_O(3) <= '0';
		RB_MOSFET_O(2) <= '0';
		RB_MOSFET_O(1) <= '0';
		RB_MOSFET_O(0) <= '0';
	end if;	 
END IF;
end process;

--======================================================================================+
--=======================================================================|RF Comutation |
--======================================================================================+
process(main_clk)
begin
if(setpoint3=0) then
	RF_MOSFET_O <= "111000";
	RF_aa <=Delay_RF+1;
	RF_bb <=Delay_RF+1;
	RF_cc <=Delay_RF+1;
	RF_dd <=Delay_RF+1;
	RF_ee <=Delay_RF+1;
	RF_ff <=Delay_RF+1;
	RF_aaa <=Delay_RF+1;
	RF_bbb <=Delay_RF+1;
	RF_ccc <=Delay_RF+1;
	RF_ddd <=Delay_RF+1;
	RF_eee <=Delay_RF+1;
	RF_fff <=Delay_RF+1;
	
elsif(RF_direction='0')THEN--FW
	if(RF_HALL_I="100" )then
		if(RF_aa>Delay_RF) then
			RF_MOSFET_O(3) <= '1' and pwm3;
		else
			RF_aa <=RF_aa+1;
			RF_bb <=0;
			RF_cc <=0;
			RF_dd <=0;
			RF_ee <=0;
			RF_ff <=0;
		end if;
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '1' and pwm3;
		RF_MOSFET_O(0) <= '0';
	elsif(RF_HALL_I="110" ) then
		if(RF_bb>Delay_RF) then
			RF_MOSFET_O(1) <= '1' and pwm3;
		else
			RF_aa <=0;
			RF_bb <=RF_bb+1;
			RF_cc <=0;
			RF_dd <=0;
			RF_ee <=0;
			RF_ff <=0;
		end if;
		RF_MOSFET_O(5) <= '1' and pwm3;
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(0) <= '0';
	elsif(RF_HALL_I="010" ) then
		if(RF_cc>Delay_RF) then
			RF_MOSFET_O(5) <= '1' and pwm3;
		else
			RF_aa <=0;
			RF_bb <=0;
			RF_cc <=RF_cc+1;
			RF_dd <=0;
			RF_ee <=0;
			RF_ff <=0;
		end if;
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '1' and pwm3;
	elsif(RF_HALL_I="011" ) then
		if(RF_dd>Delay_RF) then
			RF_MOSFET_O(0) <= '1' and pwm3;
		else
			RF_aa<=0;
			RF_bb<=0;
			RF_cc<=0;
			RF_dd<=RF_dd+1;
			RF_ee<=0;
			RF_ff<=0;
		end if;
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '1' and pwm3;
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
	elsif(RF_HALL_I="001" ) then
		if(RF_ee>Delay_RF) then
			RF_MOSFET_O(4) <= '1' and pwm3;
		else
			RF_aa <=0;
			RF_bb <=0;
			RF_cc <=0;
			RF_dd <=0;
			RF_ee <=RF_ee+1;
			RF_ff <=0;
		end if;
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '1' and pwm3;
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
	elsif(RF_HALL_I="101" ) then
		if(RF_ff>Delay_RF) then
			RF_MOSFET_O(2) <= '1' and pwm3;
		else
			RF_aa <=0;
			RF_bb <=0;
			RF_cc <=0;
			RF_dd <=0;
			RF_ee <=0;
			RF_ff <=RF_ff+1;
		end if;
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '1' and pwm3;
	elsif(RF_HALL_I="111" ) then
	    RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
	elsif(RF_HALL_I="000" ) then
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
	end if;
ELSIF(RF_DIRECTION='1')THEN
	if(RF_HALL_I="100" )then
		if(RF_aaa>Delay_RF) then
			RF_MOSFET_O(3) <= '1' and pwm3;
		else
			RF_aaa <=RF_aaa+1;
			RF_bbb <=0;
			RF_ccc <=0;
			RF_ddd <=0;
			RF_eee <=0;
			RF_fff <=0;
		end if;
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(2) <= '1' and pwm3;
    
	elsif(RF_HALL_I="110" ) then
		if(RF_bbb>Delay_RF) then
			RF_MOSFET_O(1) <= '1' and pwm3;
		else
			RF_aaa <=0;
			RF_bbb <=RF_bbb+1;
			RF_ccc <=0;
			RF_ddd <=0;
			RF_eee <=0;
			RF_fff <=0;
		end if;
		
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(3) <= '1' and pwm3;
		RF_MOSFET_O(0) <= '0';  

	elsif(RF_HALL_I="010" ) then
		if(RF_ccc>Delay_RF) then
			RF_MOSFET_O(5) <= '1' and pwm3;
		else
			RF_aaa <=0;
			RF_bbb <=0;
			RF_ccc <=RF_ccc+1;
			RF_ddd <=0;
			RF_eee <=0;
			RF_fff <=0;
		end if;
		
		RF_MOSFET_O(1) <= '1' and pwm3;
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(0) <= '0';

	elsif(RF_HALL_I="011" ) then
		if(RF_ddd>Delay_RF) then
			RF_MOSFET_O(0) <= '1'  and pwm3;
		else
			RF_aaa <=0;
			RF_bbb <=0;
			RF_ccc <=0;
			RF_ddd <=RF_ddd+1;
			RF_eee <=0;
			RF_fff <=0;
		end if;
		
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0' ;
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(5) <= '1'  and pwm3;
			
	elsif(RF_HALL_I="001" ) then
		if(RF_eee>Delay_RF) then
			RF_MOSFET_O(4) <= '1'  and pwm3;
		else
			RF_aaa <=0;
			RF_bbb <=0;
			RF_ccc <=0;
			RF_ddd <=0;
			RF_eee <=RF_eee+1;
			RF_fff <=0;
		end if;
		
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(0) <= '1'  and pwm3;
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
		
	elsif(RF_HALL_I="101" ) then
		if(RF_fff>Delay_RF) then
			RF_MOSFET_O(2) <= '1' and pwm3;
		else
			RF_aaa <=0;
			RF_bbb <=0;
			RF_ccc <=0;
			RF_ddd <=0;
			RF_eee <=0;
			RF_fff <=RF_fff+1;
		end if;
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(4) <= '1' and pwm3;
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
			
	elsif(RF_HALL_I="111" ) then
			
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
	elsif(RF_HALL_I="000" ) then
	
		RF_MOSFET_O(5) <= '0';
		RF_MOSFET_O(4) <= '0';
		RF_MOSFET_O(3) <= '0';
		RF_MOSFET_O(2) <= '0';
		RF_MOSFET_O(1) <= '0';
		RF_MOSFET_O(0) <= '0';
	end if;
END IF;
end process;


end Behavior;
		