//Import All Required Libraries.
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>


//Define  a strtuct data structure for sharing the data between the threads.
struct thread_data
{
   float tempc;
   float tempf;
   float humd;
   double prus;  
};


//Registers Address for controlling LCD.
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00

//LCD Backlight Control
#define BACKLIGHT 0x08
#define ENABLE 0x04

//Function to write the data into the LCD Display.
void lcd_write(int file, uint8_t value, uint8_t mode) {
   uint8_t high_nibble = value & 0xF0; 
   uint8_t low_nibble = (value << 4) & 0xF0; 

   uint8_t data[4] = {
      high_nibble | mode | BACKLIGHT | ENABLE,
      high_nibble | mode | BACKLIGHT,
      low_nibble | mode | BACKLIGHT | ENABLE,
      low_nibble | mode | BACKLIGHT
   };

   if (write(file, data, 4) != 4) 
   {
      printf("Unable to write the data to the display.\n");
   }

   usleep(5000); 
}

//Function to intialize the LCD.
void lcd_init(int file) 
{
   lcd_write(file, 0x03, 0); 
   lcd_write(file, 0x03, 0); 
   lcd_write(file, 0x03, 0); 
   lcd_write(file, 0x02, 0); 

   lcd_write(file, LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS, 0);
   lcd_write(file, LCD_DISPLAYCONTROL | LCD_DISPLAYON, 0);
   lcd_write(file, LCD_CLEARDISPLAY, 0);
   lcd_write(file, LCD_ENTRYMODESET | LCD_RETURNHOME, 0);
}

//Function to take an argumnet from the display to write to LCD.
void lcd_display_string(int file, const char *string) 
{
   for (int i = 0; string[i] != '\0'; i++) 
   {
      lcd_write(file, string[i], 1);
   }
}

//Function to clear the LCD display
void lcd_clear(int file) 
{
   lcd_write(file, LCD_CLEARDISPLAY, 0);
   usleep(2000); 
}


//Inilitailze Mutexes.
pthread_mutex_t mutex_watch = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_display = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_pres = PTHREAD_MUTEX_INITIALIZER;



//Thread reading in temp and humidity from BME280 sensor. 
void *bme280_read(void *arg) 
{
   struct thread_data *data = (struct thread_data *)arg;
   while (1) 
   {
      int file;
	   char *bus = "/dev/i2c-2";  //Detect the sensor from i2cdetect -r 2.
	   if((file = open(bus, O_RDWR)) < 0) 
	   {
		   printf("ERROR unable to open the BME280 Sensor.\n");  //If unable to open the sensor.
		   exit(1);
	   }
	
      ioctl(file, I2C_SLAVE, 0x76);   //setting sensor at 0x76 as slave component.

      //Read the data from the sensors read address 0x88.
      pthread_mutex_lock(&mutex_watch);
      char read_d[1] = {0x88};
      write(file, read_d, 1);   //Write to the file to read from that address.
      char data_read[24] = {0};   //We get an 24 bit data req to calculate temp, humidity at 0x88.
      if(read(file, data_read, 24) != 24)
      {
         printf("Unable to read the data at 0x88.\n");
         exit(1);
      }
      pthread_mutex_unlock(&mutex_watch);

      //Calculating TEMPERATURE Coeffecient from the data.
      uint16_t temp_1 = (uint16_t)(data_read[0] | (data_read[1] << 8)); //using the LSB 0-5 from the data we read.
      int16_t temp_2 = (int16_t)(data_read[2] | (data_read[3] << 8));
      int16_t temp_3 = (int16_t)(data_read[4] | (data_read[5] << 8));



      //Calculating HUMIDITY Coeffecient from the data.
      //Read the calibration data needed to calculate HUMIDITY and TEMPREATURE.
      pthread_mutex_lock(&mutex_watch);
      read_d[0] = 0xA1;
      write(file, read_d, 1);
      char data_read2[8] = {0};
      read(file, data_read2, 1);
      int calh_1 = data_read2[0];

      read_d[0] = 0xE1;
      write(file, read_d, 1);
      read(file, data_read, 7);
      pthread_mutex_unlock(&mutex_watch);

      int calh_2 = (data_read[0] + data_read[1] * 256);
      if(calh_2 > 32767)
      {
         calh_2 -= 65536;
      }

      int calh_3 = data_read[2] & 0xFF ;

      int calh_4 = (data_read[3] * 16 + (data_read[4] & 0xF));
      if(calh_4 > 32767)
      {
         calh_4 -= 65536;
      }

      int calh_5 = (data_read[4] / 16) + (data_read[5] * 16);
      if(calh_5 > 32767)
      {
         calh_5 -= 65536;
      }

      int calh_6 = data_read[6];
      if(calh_6 > 127)
      {
         calh_6 -= 256;
      }

      //Handling the oversampling of humidity data.
      pthread_mutex_lock(&mutex_watch);
      char config[2] = {0};
      config[0] = 0xF2;
      config[1] = 0x01;
      write(file, config, 2);
      
      //Handling the oversampling of temp data.
      config[0] = 0xF4;
      config[1] = 0x27;
      write(file, config, 2);

      //Setting the configuration options.
      config[0] = 0xF5;
      config[1] = 0xA0;
      write(file, config, 2);

      //Data read out to avoid mixing of the data between different measurements.
      read_d[0] = 0xF7;
      write(file, read_d, 1);
      read(file, data_read2, 8);
      pthread_mutex_unlock(&mutex_watch);

      //TEMPERATURE CALCULATIONS.
      long temp_d = ((long)(data_read2[3] * 65536 + ((long)(data_read2[4] * 256) + (long)(data_read2[5] & 0xF0)))) / 16;
      float tval_1 = (((float)temp_d) / 16384.0 - ((float)temp_1) / 1024.0) * ((float)temp_2);
      float tval_2 = ((((float)temp_d) / 131072.0 - ((float)temp_1) / 8192.0) * (((float)temp_d)/131072.0 - ((float)temp_1)/8192.0)) * ((float)temp_3);
      float tval_3 = (long)(tval_1 + tval_2);
      float temp_cel = (tval_1 + tval_2) / 5120.0;
      float temp_farh = temp_cel * 1.8 + 32;


      //HUMIDITY CALCULATIONS.
      long humd_d = (data_read2[6] * 256 + data_read2[7]);
      float hval_1 = (((float)tval_3) - 76800.0);
      hval_1 = (humd_d - (calh_4 * 64.0 + calh_5 / 16384.0 * hval_1)) * (calh_2 / 65536.0 * (1.0 + calh_6 / 67108864.0 * hval_1 * (1.0 + calh_3 / 67108864.0 * hval_1)));
      float humidity = hval_1 * (1.0 -  calh_1 * hval_1 / 524288.0);

      //updating the values.
      pthread_mutex_lock(&mutex_watch);
      data->tempc = temp_cel;
      data->tempf = temp_farh;
      data->humd = humidity;
      pthread_mutex_unlock(&mutex_watch);

      close(file);
      usleep(3000000);
   }  
}


//Thread reading in pressure from BMP280 sensor. 
void *bmp280_read(void *arg) 
{
   struct thread_data *data = (struct thread_data *)arg;
   while (1) 
   {
      int file2;
	   char *bus = "/dev/i2c-2";  //Detect the sensor from i2cdetect -r 2.
	   if((file2 = open(bus, O_RDWR)) < 0) 
	   {
		   printf("ERROR unable to open the BMP280 Sensor.\n");  //If unable to open the sensor.
		   exit(1);
	   }
	
      ioctl(file2, I2C_SLAVE, 0x77);  //setting sensor at 0x77 as slave component.

      //Read the data from the sensors read address 0x88.
      pthread_mutex_lock(&mutex_pres);
      char read_d[1] = {0x88};
      write(file2, read_d, 1);     //Write to the file2 to read from that address.
      char data_read[24] = {0};   //We get an 24 bit data req to calculate pressure at 0x88.
      if(read(file2, data_read, 24) != 24)
      {
         printf("Unable to read the data at 0x88.\n");
         exit(1);
      }
      pthread_mutex_unlock(&mutex_pres);


      // Calculating PRESSURE Coeffecient from the data.
      int calp_1 = data_read[7] * 256 + data_read[6];

      int calp_2  = data_read[9] * 256 + data_read[8];
      if(calp_2 > 32767)
      {
         calp_2 -= 65536;
      }

      int calp_3 = data_read[11]* 256 + data_read[10];
      if(calp_3 > 32767)
      {
         calp_3 -= 65536;
      }

      int calp_4 = data_read[13]* 256 + data_read[12];
      if(calp_4 > 32767)
      {
         calp_4 -= 65536;
      }

      int calp_5 = data_read[15]* 256 + data_read[14];
      if(calp_5 > 32767)
      {
         calp_5 -= 65536;
      }

      int calp_6 = data_read[17]* 256 + data_read[16];
      if(calp_6 > 32767)
      {
         calp_6 -= 65536;
      }

      int calp_7 = data_read[19]* 256 + data_read[18];
      if(calp_7 > 32767)
      {
         calp_7 -= 65536;
      }

      int calp_8 = data_read[21]* 256 + data_read[20];
      if(calp_8 > 32767)
      {
         calp_8 -= 65536;
      }

      int calp_9 = data_read[23]* 256 + data_read[22];
      if(calp_9 > 32767)
      {
         calp_9 -= 65536;
      }
         
      
      //Setting the configuration options.
      pthread_mutex_lock(&mutex_pres);
      char config[2] = {0};
      config[0] = 0xF5;
      config[1] = 0xA0;
      write(file2, config, 2);
      sleep(1);
      
      //Data read out to avoid mixing of the data between different measurements.
      read_d[0] = 0xF7;
      write(file2, read_d, 1);
      read(file2, data_read, 8);
      pthread_mutex_unlock(&mutex_pres);
      
      //PRESSURE CALCULATIONS.
      long pres_d = (((long)data_read[0] * 65536) + ((long)data_read[1] * 256) + (long)(data_read[2] & 0xF0)) / 16;
      double pval_1 = (((double)pres_d) / 16384.0 - ((double)calp_1) / 1024.0) * ((double)calp_2);
      double pval_2 = ((((double)pres_d) / 131072.0 - ((double)calp_1) / 8192.0) *(((double)pres_d)/131072.0 - ((double)calp_1)/8192.0)) * ((double)calp_3);
      double pval_3 = (long)(pval_1 + pval_2);
      pval_1 = ((double)pval_3 / 2.0) - 64000.0;
      pval_2 = pval_1 * pval_1 * ((double)calp_6) / 32768.0;
      pval_2 = pval_2 + pval_1 * ((double)calp_5) * 2.0;
      pval_2 = (pval_2 / 4.0) + (((double)calp_4) * 65536.0);
      pval_1 = (((double) calp_3) * pval_1 * pval_1 / 524288.0 + ((double) calp_2) * pval_1) / 524288.0;
      pval_1 = (1.0 + pval_1 / 32768.0) * ((double)calp_1);
      double pres = 1048576.0 - (double)pres_d;
      pres = (pres - (pval_2 / 4096.0)) * 6250.0 / pval_1;
      pval_1 = ((double) calp_9) * pres * pres / 2147483648.0;
      pval_2 = pres * ((double) calp_8) / 32768.0;
      double pressure = (pres + (pval_1 + pval_2 + ((double)calp_7)) / 16.0) / 100;
      
      //Updating the pressure value.
      pthread_mutex_lock(&mutex_pres);
      data->prus = pressure;
      pthread_mutex_unlock(&mutex_pres);
     
      close(file2);
      usleep(3000000);
   }
        
} 


//Thread to display the temp, humdity, pressure.
void *display(void *arg) 
{
   struct thread_data *data = (struct thread_data *)arg;
   while (1) 
   {
      int fd = open("/dev/i2c-2", O_RDWR);
      if (fd < 0) 
      {
         printf("ERROR unable to open the DISPLAY Sensor.\n");  //If unable to open the sensor.
		   exit(1);
      }

      ioctl(fd, I2C_SLAVE, 0x27);  //setting sensor at 0x27 as slave component.

      //Initialize the LCD display.
      lcd_init(fd);

      //Time for initialization.
      usleep(10000);

      char value[20];

      pthread_mutex_lock(&mutex_display);
      //Temp in C.
      lcd_clear(fd);
      lcd_display_string(fd, "Temp (C):");
      sprintf(value, "%.2f", data->tempc);
      lcd_display_string(fd, value);
      sleep(3);
      //Temp in F.
      lcd_clear(fd);
      lcd_display_string(fd, "Temp (F):");
      sprintf(value, "%.2f", data->tempf);
      lcd_display_string(fd, value);
      sleep(3);
      //Humidity.
      lcd_clear(fd);
      lcd_display_string(fd, "HUMID(RH):");
      sprintf(value, "%.2f", data->humd);
      lcd_display_string(fd, value);
      sleep(3);
      //Pressure.
      lcd_clear(fd);
      lcd_display_string(fd, "PRESSURE(Pa):");
      sprintf(value, "%.2f", data->prus);
      lcd_display_string(fd, value);
      sleep(3);
      pthread_mutex_unlock(&mutex_display);

      close(fd);
   }

}



//C program main function.
int main() {

   struct thread_data thread_data;
   thread_data.tempc = 0.0;
   thread_data.tempf = 0.0;
   thread_data.humd = 0.0;
   thread_data.prus = 0.0;
   //Define Threads for handling each of the buttons and display.
   pthread_t bme280_thread, bmp280_thread, display_thread;

   pthread_attr_t bme280_thread_attr, bmp280_thread_attr, display_thread_attr;

   struct sched_param bme280_thread_param, bmp280_thread_param, display_thread_param;

   pthread_attr_init(&bme280_thread_attr);
   pthread_attr_init(&bmp280_thread_attr);
   pthread_attr_init(&display_thread_attr);

   pthread_attr_getschedparam(&bme280_thread_attr, &bme280_thread_param);
   pthread_attr_getschedparam(&bmp280_thread_attr, &bmp280_thread_param);
   pthread_attr_getschedparam(&display_thread_attr, &display_thread_param);

   //Assign Priority for the threads.
   bme280_thread_param.sched_priority = 10;
   bmp280_thread_param.sched_priority = 10;
   display_thread_param.sched_priority = 5;

   //Create the Threads.
   pthread_create(&bme280_thread, NULL, bme280_read, &thread_data);
   pthread_create(&bmp280_thread, NULL, bmp280_read, &thread_data);
   pthread_create(&display_thread, NULL, display, &thread_data);

   pthread_join(bme280_thread, NULL);
   pthread_join(bmp280_thread, NULL);
   pthread_join(display_thread,NULL);

   pthread_attr_destroy(&bme280_thread_attr);
   pthread_attr_destroy(&bmp280_thread_attr);
   pthread_attr_destroy(&display_thread_attr);

   return 0;
}