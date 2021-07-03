#include "../lib/STM32f302x8-HAL/llpd/include/LLPD.hpp"

#include <math.h>

#include "EEPROM_CAT24C64.hpp"
#include "SRAM_23K256.hpp"
#include "SDCard.hpp"
#include "OLED_SH1106.hpp"

// to disassemble -- arm-none-eabi-objdump -S --disassemble main_debug.elf > disassembled.s

#define SYS_CLOCK_FREQUENCY 72000000

// global variables
volatile bool adcSetupComplete = false; // should be set to true after adc has been initialized

// peripheral defines
#define OP_AMP_PORT 		GPIO_PORT::A
#define OP_AMP_INVERT_PIN 	GPIO_PIN::PIN_5
#define OP_AMP_OUTPUT_PIN 	GPIO_PIN::PIN_6
#define OP_AMP_NON_INVERT_PIN 	GPIO_PIN::PIN_7
#define EFFECT1_ADC_PORT 	GPIO_PORT::A
#define EFFECT1_ADC_PIN 	GPIO_PIN::PIN_0
#define EFFECT1_ADC_CHANNEL 	ADC_CHANNEL::CHAN_1
#define EFFECT2_ADC_PORT 	GPIO_PORT::A
#define EFFECT2_ADC_PIN 	GPIO_PIN::PIN_1
#define EFFECT2_ADC_CHANNEL 	ADC_CHANNEL::CHAN_2
#define EFFECT3_ADC_PORT 	GPIO_PORT::A
#define EFFECT3_ADC_PIN 	GPIO_PIN::PIN_2
#define EFFECT3_ADC_CHANNEL 	ADC_CHANNEL::CHAN_3
#define AUDIO_IN_PORT 		GPIO_PORT::A
#define AUDIO_IN_PIN  		GPIO_PIN::PIN_3
#define AUDIO_IN_CHANNEL 	ADC_CHANNEL::CHAN_4
#define EFFECT1_BUTTON_PORT 	GPIO_PORT::B
#define EFFECT1_BUTTON_PIN 	GPIO_PIN::PIN_0
#define EFFECT2_BUTTON_PORT 	GPIO_PORT::B
#define EFFECT2_BUTTON_PIN 	GPIO_PIN::PIN_1
#define SRAM1_CS_PORT 		GPIO_PORT::B
#define SRAM1_CS_PIN 		GPIO_PIN::PIN_12
#define SRAM2_CS_PORT 		GPIO_PORT::B
#define SRAM2_CS_PIN 		GPIO_PIN::PIN_2
#define SRAM3_CS_PORT 		GPIO_PORT::B
#define SRAM3_CS_PIN 		GPIO_PIN::PIN_3
#define SRAM4_CS_PORT 		GPIO_PORT::B
#define SRAM4_CS_PIN 		GPIO_PIN::PIN_4
#define EEPROM1_ADDRESS 	false, false, false
#define EEPROM2_ADDRESS 	true, false, false
#define SDCARD_CS_PORT 		GPIO_PORT::A
#define SDCARD_CS_PIN 		GPIO_PIN::PIN_11
#define OLED_RESET_PORT 	GPIO_PORT::B
#define OLED_RESET_PIN 		GPIO_PIN::PIN_7
#define OLED_DC_PORT 		GPIO_PORT::B
#define OLED_DC_PIN 		GPIO_PIN::PIN_8
#define OLED_CS_PORT 		GPIO_PORT::B
#define OLED_CS_PIN 		GPIO_PIN::PIN_9

// these pins are unconnected on Gen_FX_SYN Rev 2 development board, so we disable them as per the ST recommendations
void disableUnusedPins()
{
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_13, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_14, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_12, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_2, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_3, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_4, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
}

int main(void)
{
	// set system clock to PLL with HSE (16MHz / 2) as input, so 72MHz system clock speed
	LLPD::rcc_clock_setup( RCC_CLOCK_SOURCE::EXTERNAL, true, RCC_PLL_MULTIPLY::BY_8, SYS_CLOCK_FREQUENCY );

	// prescale APB1 by 2, since the maximum clock speed is 36MHz
	LLPD::rcc_set_periph_clock_prescalers( RCC_AHB_PRES::BY_1, RCC_APB1_PRES::AHB_BY_2, RCC_APB2_PRES::AHB_BY_1 );

	// enable all gpio clocks
	LLPD::gpio_enable_clock( GPIO_PORT::A );
	LLPD::gpio_enable_clock( GPIO_PORT::B );
	LLPD::gpio_enable_clock( GPIO_PORT::C );
	LLPD::gpio_enable_clock( GPIO_PORT::F );

	// USART setup
	LLPD::usart_init( USART_NUM::USART_3, USART_WORD_LENGTH::BITS_8, USART_PARITY::EVEN, USART_CONF::TX_AND_RX,
				USART_STOP_BITS::BITS_1, SYS_CLOCK_FREQUENCY, 9600 );
	LLPD::usart_log( USART_NUM::USART_3, "Gen_FX_SYN starting up -----------------------------" );

	// disable the unused pins
	disableUnusedPins();

	// i2c setup (72MHz source 1000KHz clock 0x00A00D26)
	LLPD::i2c_master_setup( I2C_NUM::I2C_2, 0x00A00D26 );
	LLPD::usart_log( USART_NUM::USART_3, "I2C initialized..." );

	// spi init (36MHz SPI2 source 18MHz clock)
	LLPD::spi_master_init( SPI_NUM::SPI_2, SPI_BAUD_RATE::APB1CLK_DIV_BY_2, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::FIRST,
				SPI_DUPLEX::FULL, SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_8 );
	LLPD::usart_log( USART_NUM::USART_3, "spi initialized..." );

	// audio timer setup (for 40 kHz sampling rate at 72 MHz system clock)
	LLPD::tim6_counter_setup( 0, 1800, 40000 );
	LLPD::tim6_counter_enable_interrupts();
	LLPD::usart_log( USART_NUM::USART_3, "tim6 initialized..." );

	// DAC setup
	LLPD::dac_init( true );
	LLPD::usart_log( USART_NUM::USART_3, "dac initialized..." );

	// Op Amp setup
	LLPD::gpio_analog_setup( OP_AMP_PORT, OP_AMP_INVERT_PIN );
	LLPD::gpio_analog_setup( OP_AMP_PORT, OP_AMP_OUTPUT_PIN );
	LLPD::gpio_analog_setup( OP_AMP_PORT, OP_AMP_NON_INVERT_PIN );
	LLPD::opamp_init();
	LLPD::usart_log( USART_NUM::USART_3, "op amp initialized..." );

	// audio timer start
	LLPD::tim6_counter_start();
	LLPD::usart_log( USART_NUM::USART_3, "tim6 started..." );

	// ADC setup (note, this must be done after the tim6_counter_start() call since it uses the delay function)
	LLPD::rcc_pll_enable( RCC_CLOCK_SOURCE::INTERNAL, false, RCC_PLL_MULTIPLY::NONE );
	LLPD::gpio_analog_setup( EFFECT1_ADC_PORT, EFFECT1_ADC_PIN );
	LLPD::gpio_analog_setup( EFFECT2_ADC_PORT, EFFECT2_ADC_PIN );
	LLPD::gpio_analog_setup( EFFECT3_ADC_PORT, EFFECT3_ADC_PIN );
	LLPD::gpio_analog_setup( AUDIO_IN_PORT, AUDIO_IN_PIN );
	LLPD::adc_init( ADC_CYCLES_PER_SAMPLE::CPS_2p5 );
	LLPD::adc_set_channel_order( 4, EFFECT1_ADC_CHANNEL, EFFECT2_ADC_CHANNEL, EFFECT3_ADC_CHANNEL, AUDIO_IN_CHANNEL );
	adcSetupComplete = true;
	LLPD::usart_log( USART_NUM::USART_3, "adc initialized..." );

	// pushbutton setup
	LLPD::gpio_digital_input_setup( EFFECT1_BUTTON_PORT, EFFECT1_BUTTON_PIN, GPIO_PUPD::PULL_UP );
	LLPD::gpio_digital_input_setup( EFFECT2_BUTTON_PORT, EFFECT2_BUTTON_PIN, GPIO_PUPD::PULL_UP );

	// EEPROM setup and test
	std::vector<Eeprom_CAT24C64_AddressConfig> eepromAddressConfigs;
	eepromAddressConfigs.emplace_back( EEPROM1_ADDRESS );
	eepromAddressConfigs.emplace_back( EEPROM2_ADDRESS );
	Eeprom_CAT24C64_Manager eeproms( I2C_NUM::I2C_2, eepromAddressConfigs );
	// TODO comment the verification lines out if you're using the eeprom for persistent memory
	SharedData<uint8_t> eepromValsToWrite = SharedData<uint8_t>::MakeSharedData( 3 );
	eepromValsToWrite[0] = 64; eepromValsToWrite[1] = 23; eepromValsToWrite[2] = 17;
	eeproms.writeToMedia( eepromValsToWrite, 45 );
	eeproms.writeToMedia( eepromValsToWrite, 45 + Eeprom_CAT24C64::EEPROM_SIZE );
	SharedData<uint8_t> eeprom1Verification = eeproms.readFromMedia( 3, 45 );
	SharedData<uint8_t> eeprom2Verification = eeproms.readFromMedia( 3, 45 + Eeprom_CAT24C64::EEPROM_SIZE );
	if ( eeprom1Verification[0] == 64 && eeprom1Verification[1] == 23 && eeprom1Verification[2] == 17 &&
			eeprom2Verification[0] == 64 && eeprom2Verification[1] == 23 && eeprom2Verification[2] == 17 )
	{
		LLPD::usart_log( USART_NUM::USART_3, "eeproms verified..." );
	}
	else
	{
		LLPD::usart_log( USART_NUM::USART_3, "WARNING!!! eeproms failed verification..." );
	}

	// SRAM setup and test
	std::vector<Sram_23K256_GPIO_Config> spiGpioConfigs;
	spiGpioConfigs.emplace_back( SRAM1_CS_PORT, SRAM1_CS_PIN );
	spiGpioConfigs.emplace_back( SRAM2_CS_PORT, SRAM2_CS_PIN );
	spiGpioConfigs.emplace_back( SRAM3_CS_PORT, SRAM3_CS_PIN );
	spiGpioConfigs.emplace_back( SRAM4_CS_PORT, SRAM4_CS_PIN );
	Sram_23K256_Manager srams( SPI_NUM::SPI_2, spiGpioConfigs );
	SharedData<uint8_t> sramValsToWrite = SharedData<uint8_t>::MakeSharedData( 3 );
	sramValsToWrite[0] = 25; sramValsToWrite[1] = 16; sramValsToWrite[2] = 8;
	srams.writeToMedia( sramValsToWrite, 45 );
	srams.writeToMedia( sramValsToWrite, 45 + Sram_23K256::SRAM_SIZE );
	srams.writeToMedia( sramValsToWrite, 45 + Sram_23K256::SRAM_SIZE * 2 );
	srams.writeToMedia( sramValsToWrite, 45 + Sram_23K256::SRAM_SIZE * 3 );
	SharedData<uint8_t> sram1Verification = srams.readFromMedia( 3, 45 );
	SharedData<uint8_t> sram2Verification = srams.readFromMedia( 3, 45 + Sram_23K256::SRAM_SIZE );
	SharedData<uint8_t> sram3Verification = srams.readFromMedia( 3, 45 + Sram_23K256::SRAM_SIZE * 2 );
	SharedData<uint8_t> sram4Verification = srams.readFromMedia( 3, 45 + Sram_23K256::SRAM_SIZE * 3 );
	if ( sram1Verification[0] == 25 && sram1Verification[1] == 16 && sram1Verification[2] == 8 &&
			sram2Verification[0] == 25 && sram2Verification[1] == 16 && sram2Verification[2] == 8 &&
			sram3Verification[0] == 25 && sram3Verification[1] == 16 && sram3Verification[2] == 8 &&
			sram4Verification[0] == 25 && sram4Verification[1] == 16 && sram4Verification[2] == 8 )
	{
		LLPD::usart_log( USART_NUM::USART_3, "srams verified..." );
	}
	else
	{
		LLPD::usart_log( USART_NUM::USART_3, "WARNING!!! srams failed verification..." );
	}

	// SD Card setup and test TODO this currently isn't working correctly so it's been commented out
	/*
	LLPD::gpio_output_setup( SDCARD_CS_PORT, SDCARD_CS_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( SDCARD_CS_PORT, SDCARD_CS_PIN, true );
	SDCard sdCard( SPI_NUM::SPI_2, SDCARD_CS_PORT, SDCARD_CS_PIN, true );
	sdCard.initialize();
	LLPD::usart_log( USART_NUM::USART_3, "sd card initialized..." );
	// TODO comment the verification lines out if you're using the sd card for persistent memory
	SharedData<uint8_t> sdCardValsToWrite = SharedData<uint8_t>::MakeSharedData( 3 );
	sdCardValsToWrite[0] = 23; sdCardValsToWrite[1] = 87; sdCardValsToWrite[2] = 132;
	sdCard.writeToMedia( sdCardValsToWrite, 54 );
	SharedData<uint8_t> retVals3 = sdCard.readFromMedia( 3, 54 );
	if ( retVals3[0] == 23 && retVals3[1] == 87 && retVals3[2] == 132 )
	{
		LLPD::usart_log( USART_NUM::USART_3, "sd card verified..." );
	}
	else
	{
		LLPD::usart_log( USART_NUM::USART_3, "WARNING!!! sd card failed verification..." );
	}
	*/

	// display buffer
	uint8_t displayBuffer[(SH1106_LCDWIDTH * SH1106_LCDHEIGHT) / 8] = { 0 };

	// clear display buffer
	for ( unsigned int byte = 0; byte < (SH1106_LCDHEIGHT * SH1106_LCDWIDTH) / 8; byte++ )
	{
		displayBuffer[byte] = 0x00;
	}

	// OLED setup
	LLPD::gpio_output_setup( OLED_CS_PORT, OLED_CS_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( OLED_CS_PORT, OLED_CS_PIN, true );
	LLPD::gpio_output_setup( OLED_DC_PORT, OLED_DC_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( OLED_DC_PORT, OLED_DC_PIN, true );
	LLPD::gpio_output_setup( OLED_RESET_PORT, OLED_RESET_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( OLED_RESET_PORT, OLED_RESET_PIN, true );
	Oled_SH1106 oled( SPI_NUM::SPI_2, OLED_CS_PORT, OLED_CS_PIN, OLED_DC_PORT, OLED_DC_PIN, OLED_RESET_PORT, OLED_RESET_PIN );
	oled.begin();
	oled.displayFullRowMajor( displayBuffer );
	LLPD::usart_log( USART_NUM::USART_3, "oled initialized..." );

	LLPD::usart_log( USART_NUM::USART_3, "Gen_FX_SYN setup complete, entering while loop -------------------------------" );

	while ( true )
	{
		if ( ! LLPD::gpio_input_get(EFFECT1_BUTTON_PORT, EFFECT1_BUTTON_PIN) )
		{
			LLPD::usart_log( USART_NUM::USART_3, "BUTTON 1 PRESSED" );
		}

		if ( ! LLPD::gpio_input_get(EFFECT2_BUTTON_PORT, EFFECT2_BUTTON_PIN) )
		{
			LLPD::usart_log( USART_NUM::USART_3, "BUTTON 2 PRESSED" );
		}

		LLPD::usart_log_int( USART_NUM::USART_3, "POT 1 VALUE: ", LLPD::adc_get_channel_value(EFFECT1_ADC_CHANNEL) );
		LLPD::usart_log_int( USART_NUM::USART_3, "POT 2 VALUE: ", LLPD::adc_get_channel_value(EFFECT2_ADC_CHANNEL) );
		LLPD::usart_log_int( USART_NUM::USART_3, "POT 3 VALUE: ", LLPD::adc_get_channel_value(EFFECT3_ADC_CHANNEL) );
	}
}

extern "C" void TIM6_DAC_IRQHandler (void)
{
	if ( ! LLPD::tim6_isr_handle_delay() ) // if not currently in a delay function,...
	{
		if ( adcSetupComplete )
		{
			LLPD::adc_perform_conversion_sequence();
			uint16_t adcVal = LLPD::adc_get_channel_value( ADC_CHANNEL::CHAN_4 );

			LLPD::dac_send( adcVal );
		}
	}

	LLPD::tim6_counter_clear_interrupt_flag();
}

extern "C" void USART3_IRQHandler (void)
{
	// loopback test code for usart recieve
	uint16_t data = LLPD::usart_receive( USART_NUM::USART_3 );
	LLPD::usart_transmit( USART_NUM::USART_3, data );
}
