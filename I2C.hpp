#ifndef __I2C_H__
#define __I2C_H__
/*
 * I2C.h
 *
 *  Kim Lilliesterna (c) 2019
 *  based on original created by kolban
 */

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

#include <sys/types.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

#include <esp_system.h>

// check for esp8266 or esp32 arch
#define ARCH_ESP8266 0
#define ARCH_ESP32 1

#ifdef CONFIG_ESP8266_XTAL_FREQ
#define ARCH  ARCH_ESP8266
#else
#define ARCH  ARCH_ESP32
#endif
/**
 * @brief C++ class interface to I2C functions.
 *
 * the traditonal way of esp32freertos implementations is
 * setup driver config
 * install driver with config
 * create commandlink list
 * add [START] token to cmdlist
 * add [ADRESS] and RW flag to cmdlist
 * if writing add byte array cand count to cmdlist
 * add [STOP] Token
 * start transission
 *
 * the same flow can be done here with:
 * begin()
 * write(...) / read(...);
 * end();
 *
 * Another caveate and the principal reason for this lib is that the low level i2c driver is not
 * thread safe so this tries to overcome this by the use of Semaphores
 * one is used to protect the initialization of the class itself and one for each port
 *
 * the constructor will check if there is a driver installed for the selected port and configure and install one
 * if not. see implementation...
 *
 * The clock speed is set on the port and is common for all instances on the same port
 * on the todo list is to be able to change it on the fly...
 *
 */
class I2C {
public:
typedef struct Info_S {
		SemaphoreHandle_t	sema;		///< Semaphore for this port
		i2c_config_t		conf;		///< I2C config 
		uint8_t				count;		///< Usage count, buss will be deallocated when it reaches 0
	} Info_t;

	static const gpio_num_t DEFAULT_SDA_PIN = GPIO_NUM_14;	///<default SDA pin.
	static const gpio_num_t DEFAULT_CLK_PIN = GPIO_NUM_2;	///<default Clock pin.
	static const uint32_t DEFAULT_CLK_SPEED = 400000;		///<default Clock speed.
	static const TickType_t MAX_SEM_WAIT = pdMS_TO_TICKS(100); ///< max time to wait for semaphore beeing released
#if ARCH == ARCH_ESP32
	// obly ESP32 hass a concept of clock speed
	uint32_t clk_speed; 									///< instance clock speed, for future use
#endif
																///might need tadjustmen based depening on clock and max transmit length
	/**
	 * @brief constructor
	 **/
	I2C();
	I2C(uint8_t address, i2c_port_t portNum = I2C_NUM_0, gpio_num_t sdaPin = DEFAULT_SDA_PIN, gpio_num_t sclPin = DEFAULT_CLK_PIN, uint32_t clkSpeed = DEFAULT_CLK_SPEED,  bool pullup = true);

	/**
	 * @brief destructor
	 **/

	~I2C();


	/**
 	* @brief Get last error code
 	*
 	* @return esp_error_code
 	*/
	esp_err_t getErro() const { return address; };

	/**
 	* @brief Get the address of the %I2C slave against which we are working.
 	*
 	* @return The address of the %I2C slave.
 	*/
	uint8_t getAddress() const { return address; };

	/**
	 * @brief set new I2C address;
	 * @param address new I2C device address
	 **/
	void setAddress(uint8_t address);

	/**
	 * @brief Start new transmission
	 *
	 *   set up new command link and add start and address to it
	 * @param rw read or write direction (I2C_MASTER_WRITE/ I2C_MASTER_READ)
	 * @paeam ack to ack or not to ack
	 **/
	esp_err_t begin(i2c_rw_t rw,bool ack);

	/**
	 * @brief close up command link and start I2C transmission
	 **/

	esp_err_t end();
	/**
	 * @brief add START to command link
	 **/
	esp_err_t start();

	/**
	 * @brief add STOP to command link
	 **/
	esp_err_t stop();

	/**
	 * @brief read one bytes from address
	 *
	 * |START |i2C Address[W]|reg Address|START|I2C Adress[R]| Read byte|STOP|
	 *
	 *  @param reg register addres to read from
	 *  @param inData pointer to destination buffer
	 *  @prarm ack
	 **/
	esp_err_t readReg(uint8_t reg, uint8_t *inData, bool ack);

	/**
	 * @brief read lenght bytes from address 
	 *
	 * |START |i2C Address[W]|reg Address|START|I2C Adress[R]| Read byte|Read byte|...|STOP|
	 *
	 *  @param reg register addres to read from
	 *  @param inData pointer to destination buffer
	 *  @prarm ack
	 **/
	esp_err_t readReg(uint8_t reg, uint8_t *inData, size_t lenght, bool ack);

	/**
	 * @brief read length bytes 
	 * @param bytes pointer to destination buffer
	 * @param length number of bytes to read
	 * @ack	ack each byte
	 **/
	esp_err_t read(uint8_t* bytes, size_t length, bool ack = true);

	/**
	 * @brief read byte 
	 * @param byte pointer to destination
	 * @param	ack each byte
	 **/
	esp_err_t read(uint8_t* byte, bool ack = true);

	/**
	 * @brief write one byte
	 * @param byte byte data to write
	 * @param ack
	 **/

	esp_err_t write(uint8_t byte, bool ack = true);
	/**
	 * @brief write length bytes
	 * @param bytes pointer to data to write
	 * @param ack
	 **/
	esp_err_t write(uint8_t* bytes, size_t length, bool ack = true);

	/**
	 * @brief write one byte at reg
	 * @param reg register to write to
	 * @param byte byte data to write
	 * @param ack
	 **/

	esp_err_t writeReg(uint8_t reg, uint8_t byte, bool ack = true);
	/**
	 * @brief write length bytes to reg
	 * @param reg register to write to
	 * @param bytes pointer to data to write
	 * @param ack expect ack not
	 **/
	esp_err_t writeReg(uint8_t reg, uint8_t* bytes, size_t length, bool ack = true);

	/**
	 * @brief scan bus for devices, dumps list on stdout
	 **/
	void scan();
	
	/**
	 * @brief check for presense of device 
	 * @param address to check for;
	 **/

	bool slavePresent(uint8_t address);

private:
	uint8_t		  		address;		///< I2C address of device
	i2c_cmd_handle_t	cmdLink;		///< pointer to comand linked list
	bool			 	directionKnown;	///< Used to decide if address needs to be sent
	i2c_port_t 			port;			///< I2C port number i.e also handle to low level driver
	esp_err_t			err;			///< last error if anny
	/**
	 * @brief , delete cmdLink and release port semaphore
	 */
	void error();
public:

static Info_t info[I2C_NUM_MAX]; 
static SemaphoreHandle_t initSem;


};

#endif 
