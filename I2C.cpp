/*
 * I2C.cpp
 *
 *  Kim Lilliesterna (c) 2019
 *  based on original created by kolban
 */
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <sys/types.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <stdint.h>
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include "I2C.hpp"

static const char* TAG = "I2C";

I2C::Info_t I2C::info[I2C_NUM_MAX]; 

SemaphoreHandle_t I2C::initSem=NULL;

/**
 * @brief Create / Initialize the I2C interface.
 *
 * @param [in] address The address of the slave device.
 * @param [in] sdaPin The pin to use for SDA data.
 * @param [in] sclPin The pin to use for SCL clock.
 * @param [in] Clock speed
 * @param [in] portNum 0 or 1
 * @param [in] should internal pullup be used on SDA and SCL pins 
 * @return N/A.
 */
I2C::I2C( uint8_t theAddress, i2c_port_t thePort, gpio_num_t theSdaPin, gpio_num_t theSclPin, uint32_t theClockSpeed, bool thePullup) {
	ESP_LOGI(TAG, "I2C(portNum=%d, address=%#0x, sda=%d, scl=%d, clockSpeed=%d)", thePort, theAddress, theSdaPin, theSclPin, theClockSpeed);
	assert(thePort < I2C_NUM_MAX);

	// Check if driver is already installed for this port

	ESP_LOGD(TAG,"Checking initSem");
	
	if (initSem == NULL) {
		ESP_LOGD(TAG,"Creating initSem");
		initSem = xSemaphoreCreateMutex();
		xSemaphoreTake(initSem,  portMAX_DELAY);
		for (int i=0;i<I2C_NUM_MAX;i++) {
			I2C::info[i].sema=NULL;
			I2C::info[i].count=0;
		}
	} else { 
		ESP_LOGD(TAG,"Sem already created, trying to take it\n");
		if(xSemaphoreTake(initSem,  portMAX_DELAY) != pdPASS) {
			ESP_LOGE(TAG,"Failed to obtain init Semaphore");
			return;
		}
	}

	err=ESP_OK;
	ESP_LOGD(TAG,"Checking count for port %d =%d ",thePort,info[thePort].count);

	if(I2C::info[thePort].count>0) { // we already have an instace of this, dont reinstall
		ESP_LOGD(TAG,"Not reinstalling I2C driver on port %d",thePort);
	} else {
		ESP_LOGD(TAG,"First one for port %d",thePort);
		info[thePort].conf.mode             = I2C_MODE_MASTER;
		info[thePort].conf.sda_io_num       = theSdaPin;
		info[thePort].conf.scl_io_num       = theSclPin;
		info[thePort].conf.sda_pullup_en    = thePullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
		info[thePort].conf.scl_pullup_en    = thePullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
		info[thePort].conf.clk_stretch_tick = 300;
// esp 32 inly
// esp 8266 and its ilk does not have i2c hardware so no clockspeed 
#if ARCH == ARCH_ESP32
		info[thePort].conf.master.clk_speed =  theClockSpeed;
#endif
		// set upp i2c low level config on port
		ESP_LOGD(TAG,"Trying to configure port and install driver\n");
		if ((err = i2c_param_config(thePort, &(info[thePort].conf)))!= ESP_OK) {
				ESP_LOGE(TAG, "Failed i2c_param_config: rc=%d", err);
		}

#if ARCH == ARCH_ESP32
		if ((err = i2c_driver_install(thePort, I2C_MODE_MASTER, 0, 0, 0) )!= ESP_OK) {
#else
		if ((err = i2c_driver_install(thePort, I2C_MODE_MASTER) )!= ESP_OK) {
#endif
			ESP_LOGE(TAG, "Failed to install Driver: rc=%d ", err );
		}

		ESP_LOGD(TAG,"Creating port mutex\n");
		info[thePort].sema 	= xSemaphoreCreateMutex();
		info[thePort].count++;				// Increment instance counter
	}
	// set up instance vars
	address = theAddress;
	port = thePort;

#if ARCH == ARCH_ESP32
	clk_speed = theClockSpeed;
#endif
	
	info[thePort].count++;				// Increment instance counter

	ESP_LOGD(TAG,"Releasing init sem\n");
	xSemaphoreGive(initSem);
} // init

I2C::~I2C() 
{
	xSemaphoreTake(initSem,  portMAX_DELAY);	// Dont really care if we fail here ...
	ESP_LOGD(TAG,"I2C Destructor, port%d,count%d\n",port,info[port].count);
	info[port].count--;
	if (info[port].count <= 0) {				// is this the last instance for this port ?
		ESP_LOGD(TAG,"last instance removing sem and driver\n");
		info[port].count = 0;
		vSemaphoreDelete(info[port].sema);		// Drop the semaphore for the port
		i2c_driver_delete(port);				//Drop the low level driver for the port
	}
	xSemaphoreGive(initSem);
}

/**
 * Error handler, for now just remove the command link chain
 * and release the port semaphore
 **/
void I2C::error() {
	ESP_LOGI(TAG,"Error: Deleting cmdLink and realsing port Semapore\n");
	i2c_cmd_link_delete(cmdLink);
	xSemaphoreGive( info[port].sema );	// release semaphore
}


/**
 * @brief Set the address of the %I2C slave against which we will be working.
 *
 * @param [in] address The address of the %I2C slave.
 */
void I2C::setAddress(uint8_t address) {
	this->address = address;
} // setAddress

/**
 * @brief Add an %I2C start request to the command stream.
 * @return N/A.
 */
esp_err_t I2C::start() {
	ESP_LOGD(TAG, "start()");
	if((err = i2c_master_start(cmdLink))!=ESP_OK) {
		ESP_LOGE(TAG, "FAILED to add i2c_master_start: rc=%d ", err);
		error();
	}
	return err;
} // start


/**
 * @brief Add an %I2C stop request to the command stream.
 * @return N/A.
 */
esp_err_t I2C::stop() {
	ESP_LOGD(TAG, "stop()");
	if ((err = i2c_master_stop(cmdLink)) != ESP_OK) {
		ESP_LOGE(TAG, "FAILED to add i2c_master_stop: rc=%d ", err);
		error();
	}
	return err;
} // stop


/**
 * Begin a new transmission
 */
// TODO: begin version that also sets clock speed...
esp_err_t I2C::begin(i2c_rw_t rw,bool ack) {
	ESP_LOGD(TAG, "begin(Address: %#0x,rw: %#0x, ack:%#0x)",address,rw,ack);
	xSemaphoreTake( info[port].sema,  MAX_SEM_WAIT );		// guarante that nobody else interfeeres
	cmdLink = ::i2c_cmd_link_create();						// Create a new comand link
	esp_err_t err=ESP_OK;
	ESP_LOGV(TAG,"begin() cmdLink: %p",(void *)cmdLink);
	if ( ((err =   i2c_master_start(cmdLink)) != ESP_OK) 
		|| ((err = i2c_master_write_byte(cmdLink, (address << 1) | rw, ack)) != ESP_OK)) {
		ESP_LOGE(TAG, "FAIL: begin rc=%d", err);
	}
	return err;
}

/**
 * Send stop and start transmission
 **/
esp_err_t I2C::end() {
	ESP_LOGD(TAG, "end(cmdLink: %p)",(void *)cmdLink);
	if(  ((err = 	i2c_master_stop(cmdLink)) != ESP_OK)
		|| ((err = i2c_master_cmd_begin(port, cmdLink, 1000 / portTICK_PERIOD_MS)) != ESP_OK) ) {
			ESP_LOGE(TAG, "Failed: Adding STOP and start transmission rc=%d ", err);
	}
	i2c_cmd_link_delete(cmdLink);
	xSemaphoreGive( info[port].sema );	// release semaphore
	return err;
} // endTransaction

/**
 * Read bytes from the slave.
 */

esp_err_t I2C::read(uint8_t *bytes, size_t length, bool ack) {
	ESP_LOGD(TAG, "read(size=%d, ack=%d)", length, ack);
	if ((err = i2c_master_read(cmdLink, bytes, length, ack?I2C_MASTER_ACK:I2C_MASTER_LAST_NACK))!= ESP_OK) {
		ESP_LOGE(TAG, "Failed adding i2c_master_read: rc=%d ", err);
		error(); // clean up
	}
	return err;
} // read


/**
 * @brief Read a single byte from the slave.
 */
esp_err_t I2C::read(uint8_t *byte, bool ack) {
	ESP_LOGD(TAG, "read(size=1, ack=%d)", ack);
	if((err=i2c_master_read_byte(cmdLink, byte, ack?I2C_MASTER_ACK:I2C_MASTER_NACK))!=ESP_OK) {
		ESP_LOGE(TAG, "Failed adding i2c_master_read: rc=%d ", err);
		error(); //clean up
	}
	return err;

} // readByte

/**
 * Read register ie write register address ,START, I2C device address, read one byte
 */
esp_err_t I2C::readReg(uint8_t reg, uint8_t *inData, bool ack)
{
	//START |i2C Address[W]|reg Address|START|I2C Adress[R]| Read byte|...|STOP

	if(((begin(I2C_MASTER_WRITE,true))!=ESP_OK)							// START write i2C address + W bit
		|| ((err=i2c_master_write_byte(cmdLink, reg, ack))!=ESP_OK)		// write Reg address
		|| ((err=i2c_master_start(cmdLink))!=ESP_OK)					// new START
		|| ((err=i2c_master_write_byte(cmdLink, (address << 1) | I2C_MASTER_READ, ack))!=ESP_OK) // i2C addres + R
		|| ((err=i2c_master_read_byte(cmdLink, inData, I2C_MASTER_LAST_NACK))!=ESP_OK)	// Read byte
		|| ((err=end())!=ESP_OK)										// STOP and begin trans
	) {
		ESP_LOGE(TAG,"Faild read register 0x%0x  err=%d ",reg,err);
		error();
	}
	return err;
}

/**
 * Read from register address ie write register addres ,START, read  length bytes
 */
esp_err_t I2C::readReg(uint8_t reg, uint8_t *inData, size_t length,bool ack)
{
	ESP_LOGD(TAG, "read(size=1, ack=%d)", ack);
	if(((begin(I2C_MASTER_WRITE,true))!=ESP_OK)							// START write i2C address + W bit
		|| ((err=i2c_master_write_byte(cmdLink, reg, ack))!=ESP_OK)	// write Reg address
		|| ((err=i2c_master_start(cmdLink))!=ESP_OK)					// new START
		|| ((err=i2c_master_write_byte(cmdLink, (address << 1) | I2C_MASTER_READ, ack))!=ESP_OK) // i2C addres + R
		|| ((err=i2c_master_read(cmdLink, inData, length, I2C_MASTER_LAST_NACK))!=ESP_OK)	// Read bytes
		|| ((err=end())!=ESP_OK)										// STOP and begin trans
	) {
		ESP_LOGE(TAG,"Faild read %d bytes from register 0x%0x  err=%d ",length,reg,err);
		error();
	}
	return err;
}
/**
 * @brief Write a single byte to the %I2C slave.
 *
 * @param[in] byte The byte to write to the slave.
 * @param[in] ack Whether or not an acknowledgment is expected from the slave.
 * @return N/A.
 */
esp_err_t I2C::write(uint8_t byte, bool ack) {
		ESP_LOGD(TAG, "write(val=%#0x, ack=%d)", byte, ack);
		if((err = i2c_master_write_byte(cmdLink, byte, ack)) != ESP_OK) {
			ESP_LOGE(TAG, "i2c_master_write_byte: rc=%d", err);
			error();
		}
		return err;
} // write byte


/**
 * @brief Write a sequence of byte to the %I2C slave.
 *
 * @param [in] bytes The sequence of bytes to write to the %I2C slave.
 * @param [in] length The number of bytes to write.
 * @param [in] ack Whether or not an acknowledgment is expected from the slave.
 * @return N/A.
 */
esp_err_t I2C::write(uint8_t *bytes, size_t length, bool ack) {
	ESP_LOGD(TAG, "write(Bytes %p, length=%d, ack=%d)", (void *)bytes,length, ack);
	ESP_LOG_BUFFER_HEXDUMP(TAG, bytes, length, ESP_LOG_VERBOSE);

	if ((err=i2c_master_write(cmdLink, bytes, length, ack))!=ESP_OK) {
		ESP_LOGE(TAG, "i2c_master_write_byte failed: rc=%d ", err);
		error();
	}
	return err;
} // write


/**
 * @brief Scan the I2C bus looking for devices.
 */
void I2C::scan() {
	ESP_LOGD(TAG,"Data Pin: %d, Clock Pin: %d\n", info[port].conf.sda_io_num, info[port].conf.scl_io_num);
	ESP_LOGD(TAG,"     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	ESP_LOGD(TAG,"00:         ");
	for (uint8_t i = 3; i < 0x78; i++) {
		if (i % 16 == 0) {
			ESP_LOGD(TAG,"\n%.2x:", i);
		}
		if (slavePresent(i)) {
			ESP_LOGD(TAG," %.2x", i);
		} else {
			ESP_LOGD(TAG," --");
		}
	}
	ESP_LOGD(TAG,"\n");
} // scan


/**
 * @brief Determine if the slave is present and responding.
 * @param [in] The address of the slave.
 * @return True if the slave is present and false otherwise.
 */
bool I2C::slavePresent(uint8_t address) {

	i2c_cmd_handle_t tmpCmd =NULL;
	ESP_LOGI(TAG,"Checking for slave at address %0x\n",address );
	xSemaphoreTake( info[port].sema,  MAX_SEM_WAIT );		// guarante that nobody else interfeeres
	ESP_LOGD(TAG,"after semaphore\n" );

	tmpCmd = i2c_cmd_link_create();
	if(tmpCmd == NULL) {
		ESP_LOGE(TAG,"Failed to create cmd link\n");
		return false;
	}
	ESP_LOGD(TAG,"after link create\n" );
	if((err=i2c_master_start(tmpCmd))!=ESP_OK) {
		ESP_LOGE(TAG,"Failed to create cmd link: %d\n",err);
	}else if ((err=i2c_master_write_byte(tmpCmd, (address << 1) | I2C_MASTER_WRITE, true ))!=ESP_OK) {
		ESP_LOGE(TAG,"Failed to write byte: %d\n",err);
	}else if ((err=i2c_master_stop(tmpCmd))!=ESP_OK) {
		ESP_LOGE(TAG,"Failed write Stop: %d\n",err);
	} else if ((err = i2c_master_cmd_begin(port, tmpCmd, 100 / portTICK_PERIOD_MS))!=ESP_OK) {
		ESP_LOGE(TAG,"Failed Begin transmission: %d\n",err);
	}
	if(err==ESP_OK)
		ESP_LOGI(TAG,"Slave found\n");
	else
		ESP_LOGI(TAG,"Slave NOT found\n");


	/*
	if(((err=i2c_master_start(tmpCmd))!=ESP_OK)
	 ||((i2c_master_write_byte(tmpCmd, (address << 1) | I2C_MASTER_WRITE, true ))!=ESP_OK)
	 ||((i2c_master_stop(tmpCmd))!=ESP_OK)
	 ){
		ESP_LOGE(TAG,"Faild presense check err=%d ",err );
	} else {
		ESP_LOGE(TAG,"Begin Transmission");
		err = i2c_master_cmd_begin(port, tmpCmd, 100 / portTICK_PERIOD_MS);
	}
	*/

	ESP_LOGD(TAG,"Releasing port sem and deleting cmd_link\n");
	if(tmpCmd !=NULL) {
		i2c_cmd_link_delete(tmpCmd);
	}
	xSemaphoreGive( info[port].sema );	// release semaphore
	return err == 0;  // Return true if the slave is present and false otherwise.
	} // slavePresent

