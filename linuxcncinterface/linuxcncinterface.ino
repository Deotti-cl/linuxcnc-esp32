/*
 UDPSendReceiveString:
 This sketch receives UDP message strings, prints them to the serial port
 and sends an "acknowledge" string back to the sender

 A Processing sketch is included at the end of file that can be used to send
 and received messages for testing with a computer.

 created 21 Aug 2010
 by Michael Margolis

 This code is in the public domain.
 */


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_ipc.h>
#include <stdlib.h>
#include <gpio.h>
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
///#include <mcpwm.h>
//#include <WiFiUdp.h>
#include <esp_timer.h>
#include <rmt.h>
#include <string.h>
#include <ESP32Encoder.h>


ESP32Encoder encoder;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
 
/*###########################Ethernet definitions ######################################*/
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
IPAddress sub(255, 255, 255, 0); 
unsigned int localPort = 27181;      // local port to listen on
// buffers for receiving and sending data
char packetBuffer[1024];  // buffer to hold incoming packet,
EthernetUDP Udp;
//WiFiUDP Udp;

char bufferin[1024] ;
char bufferout[1024] ;
/*##################################i nterupt function ################################*/
unsigned long timeus;
volatile int interrupts;
int totalInterrupts;//char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; 
hw_timer_t * timer = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/*######################### STEPPER variables ################################*/

int16_t steps_0_dir;
int16_t steps_0;
int16_t steps_1_dir;
int16_t steps_1;
int16_t steps_2_dir;
int16_t steps_2;
int16_t steps_3_dir;
int16_t steps_3;
int16_t steps_4_dir;
int16_t steps_4;


int16_t steps_0_executed;
int16_t steps_1_executed;
int16_t steps_2_executed;
int16_t steps_3_executed;
int16_t steps_4_executed;
int16_t steps_5_executed;



 
struct old {
  
int32_t old_pos_0;
int32_t old_pos_1;
int32_t old_pos_2;
int32_t old_pos_3;
int32_t old_pos_4;
int32_t old_pos_5;
}olds;






int8_t dir0=1;
int8_t dir1=1;
int8_t dir2=1;
int8_t dir3=1;
int8_t dir4=1;
int8_t dir5=1;


struct axes  {
 int32_t fb0;
 int32_t fb1;
 int32_t fb2;
 int32_t fb3;
 int32_t fb4;
 int32_t fb5;
 int32_t fb6;
 int32_t fb7;
 int32_t fb8;
 unsigned int io;
int64_t enc0;

};       

axes fdbrt;
//struct axes mayaxes ;

struct pos  {
 int32_t pos0;
 int32_t pos1;
 int32_t pos2;
 int32_t pos3;
 int32_t pos4;
 int32_t pos5;
 int32_t pos6;
 int32_t pos7;
 int32_t pos8;
 uint32_t io;
 int64_t analog;

}cmd;       

axes fdb={0, 0, 0, 0, 0, 0};

int packetSize;
long enc0;
long enc1;
long enc2;
long enc3;
int count=0;
int sema=0;
//////////////////////////////////task nucleo 2 ////////////////////////

void LoopOnProCpu(void *arg) {
    (void)arg;
int    packetSize = Udp.parsePacket();
if (packetSize) {
    Udp.write(bufferin,sizeof(fdb)); 
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.endPacket();
  
   
    Udp.read(packetBuffer,sizeof(cmd));
    memcpy(&cmd,packetBuffer,sizeof(cmd));
    
    memcpy(&bufferin, &fdb, sizeof(fdb)); 
    
   // if (fdb.enc0 < 0){
   //  Serial.println (fdb.fb0); 
   // }
   
   // if there's data available, read a packet
  
//Serial.println (cmd.pos5);
fdb={fdbrt.fb0,fdbrt.fb1,fdbrt.fb2,fdbrt.fb3,100,enc0};
//fdb={cmd.pos0,cmd.pos1,cmd.pos2,cmd.pos3,3};




//Serial.println (testch1);

////////////////////////////////////////////////////////
steps_0_executed=0;
steps_0_dir=cmd.pos0 - fdbrt.fb0;
if(steps_0_dir > 0 && dir0 == -1 ){
   REG_WRITE(GPIO_OUT_W1TC_REG, BIT12);//GPIO4 LOW (clear);
   dir0=1;
   steps_0_dir=0;
   
 }
 if(steps_0_dir < 0 && dir0 == 1 ){
  
   REG_WRITE(GPIO_OUT_W1TS_REG, BIT12);//GPIO4 HIGH (clear);
   dir0=-1;
   steps_0_dir=0;
  
 }
steps_0=abs(steps_0_dir);
/////////////////////////////////////////////////////
steps_1_executed=0;
steps_1_dir=cmd.pos1 - fdbrt.fb1;
if(steps_1_dir > 0 && dir1 != 1 ){
   REG_WRITE(GPIO_OUT_W1TC_REG, BIT13);//GPIO4 LOW (clear);
    steps_1_dir=0;
   dir1=1;
 }
 if(steps_1_dir < 0 && dir1 != -1 ){
  
   REG_WRITE(GPIO_OUT_W1TS_REG, BIT13);//GPIO4 HIGH (clear);
    steps_1_dir=0;
   dir1=-1;
 }
steps_1=abs(steps_1_dir);
/////////////////////////////////////////////////////
steps_2_executed=0;
steps_2_dir=cmd.pos2 - fdbrt.fb2;
if(steps_2_dir > 0 && dir2 != 1 ){
    steps_2_dir=0;
   REG_WRITE(GPIO_OUT_W1TC_REG, BIT14);//GPIO2 LOW (clear);
  dir2=1;
 }
 if(steps_2_dir < 0 && dir2 != -1 ){
   steps_2_dir=0;
   REG_WRITE(GPIO_OUT_W1TS_REG, BIT14);//GPIO2 LOW (clear);
  dir2=-1;
 }
steps_2=abs(steps_2_dir);
////////////////////////////////////////////////////
steps_3_executed=0;
steps_3_dir=cmd.pos3 - fdbrt.fb3;
if(steps_3_dir > 0 && dir3 != 1 ){
   REG_WRITE(GPIO_OUT_W1TC_REG, BIT14);//GPIO4 LOW (clear);
    steps_3_dir=0;
   dir3=1;
 }
 if(steps_3_dir < 0 && dir3 != -1 ){
  
   REG_WRITE(GPIO_OUT_W1TS_REG, BIT14);//GPIO4 HIGH (clear);
    steps_3_dir=0;
   dir3=-1;
 }
steps_3=abs(steps_3_dir); 
    
    //Udp.read(packetBuffer,50);
    
count=count+1;
if (count>= 1000){

//Serial.println ("RECIVED");
//Serial.println (cmd.pos0);
//Serial.println (fdbrt.fb0);
//Serial.println (steps_0_dir);

count=0;
}
}
}



////////////////////////////////////////////////////////
/*######################### RMT configuration function  ################################*/
void initRMT() {
    rmt_item32_t rmtItem[2];
    rmt_config_t rmtConfig;
    rmtConfig.rmt_mode = RMT_MODE_TX;
    rmtConfig.clk_div = 20;
    rmtConfig.mem_block_num = 2;
    rmtConfig.tx_config.loop_en = false;
    rmtConfig.tx_config.carrier_en = 0;
    rmtConfig.tx_config.carrier_freq_hz = 0;
    rmtConfig.tx_config.carrier_duty_percent = 50;
    rmtConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    rmtConfig.tx_config.idle_output_en = true;


    rmtConfig.mem_block_num = 2;
    rmtConfig.tx_config.loop_en = false;
    rmtConfig.tx_config.carrier_en = 0;
    rmtConfig.tx_config.carrier_freq_hz = 0;

//#ifdef STEP_PULSE_DELAY
  // rmtItem[0].duration0 = 4 * 2;
//#else
    rmtItem[0].duration0 = 2;
//#endif
    rmtItem[0].duration1 = 4 * 2;
    rmtItem[1].duration0 = 0;
    rmtItem[1].duration1 = 0;
//AXIS 0 Config
   
    rmt_set_source_clk(RMT_CHANNEL_0, RMT_BASECLK_APB);
    rmtConfig.channel = RMT_CHANNEL_0;
    rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num = GPIO_NUM_2;
    rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);


//AXIS 1 Config

    rmt_set_source_clk(RMT_CHANNEL_1, RMT_BASECLK_APB);
    rmtConfig.channel = RMT_CHANNEL_1;
    rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num = GPIO_NUM_4;
    rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);


//AXIS 2 Config

    rmt_set_source_clk(RMT_CHANNEL_2, RMT_BASECLK_APB);
    rmtConfig.channel = RMT_CHANNEL_2;
    rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num = GPIO_NUM_16;
    rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);

//AXIS 3 Config

    rmt_set_source_clk(RMT_CHANNEL_3, RMT_BASECLK_APB);
    rmtConfig.channel = RMT_CHANNEL_3;
    rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num = GPIO_NUM_16;
    rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);

//AXIS 4 Config

    rmt_set_source_clk(RMT_CHANNEL_4, RMT_BASECLK_APB);
    rmtConfig.channel = RMT_CHANNEL_4;
    rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num = GPIO_NUM_16;
    rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);

//AXIS 5 Config

    rmt_set_source_clk(RMT_CHANNEL_5, RMT_BASECLK_APB);
    rmtConfig.channel = RMT_CHANNEL_5;
    rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num = GPIO_NUM_16;
    rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);

}




//IRAM_ATTR static void onTime()(){
static void IRAM_ATTR onTime() {

 
 portENTER_CRITICAL_ISR(&timerMux);
if (steps_0_executed < steps_0) {
 RMT.conf_ch[RMT_CHANNEL_0].conf1.mem_rd_rst = 1;
 RMT.conf_ch[RMT_CHANNEL_0].conf1.tx_start = 1;
steps_0_executed++;
fdbrt.fb0=fdbrt.fb0+dir0;
}


if (steps_1_executed < steps_1) {
 RMT.conf_ch[RMT_CHANNEL_1].conf1.mem_rd_rst = 1;
 RMT.conf_ch[RMT_CHANNEL_1].conf1.tx_start = 1;
steps_1_executed++;
fdbrt.fb1=fdbrt.fb1+dir1;

}
if (steps_2_executed < steps_2) {
 RMT.conf_ch[RMT_CHANNEL_2].conf1.mem_rd_rst = 1;
 RMT.conf_ch[RMT_CHANNEL_2].conf1.tx_start = 1;
steps_2_executed++;
fdbrt.fb2=fdbrt.fb2+dir2;

}
/*
if (steps_3_executed < steps_3) {
 RMT.conf_ch[RMT_CHANNEL_3].conf1.mem_rd_rst = 1;
 RMT.conf_ch[RMT_CHANNEL_3].conf1.tx_start = 1;
steps_3_executed++;
fdbrt.fb3=fdbrt.fb3+dir3;
}




if (steps_0_executed==steps_0 && steps_1_executed==steps_1 
    && steps_2_executed==steps_2 && steps_3_executed==steps_3)
    { 
     //fdbrt.fb0=fdbrt.fb0+steps_0_dir;
     
     // timerAlarmDisable(timer);
      //timerEnd(timer);
    //timer = NULL;
    }
//*/
portEXIT_CRITICAL_ISR(&timerMux);
}









void setup() {

 
 
 //pinMode(4,OUTPUT);
 pinMode(12,OUTPUT);
 pinMode(13,OUTPUT);
 //REG_WRITE(GPIO_ENABLE_REG, BIT4);//Define o GPIO2 como saída
 //REG_WRITE(GPIO_ENABLE_REG, BIT15);//Define o GPIO2 como saída
 //REG_WRITE(GPIO_ENABLE_REG, BIT17);//Define o GPIO2 como saída
 
///////////////////////   encoder ///////////////////////////////

// Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;

  // Attache pins for use as encoder pins
  encoder.attachHalfQuad(15, 17);


///////////////////////////////////////////////////////////////
 
 
 
 initRMT();
 
 /***************************************** Main timer stepper *****************/ 
 
//hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


  // Configure Prescaler to 80, as our timer runs @ 80Mhz
  // Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
  timer = timerBegin(0, 80, true);                
  timerAttachInterrupt(timer, &onTime, true);    
  // Fire Interrupt every 1m ticks, so 1s
  timerAlarmWrite(timer,4, true);      
  timerAlarmEnable(timer);



// You can use Ethernet.init(pin) to configure the CS pin
  
  Ethernet.init(5);   // MKR ETH shield
  
//*********************************//
//Ethernet.setCsPin(5); // set Pin 3 for CS
//Ethernet.setRstPin(27); // set Pin 4 for RST
//*********************************//



//pinMode(15, OUTPUT);
//pinMode(2, OUTPUT);
  // start the Ethernet
  Ethernet.begin(mac, ip, sub);

  // Open serial communications and wait for port to open:
 Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
   }
  }
 if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);
}

IPAddress remote = Udp.remoteIP();
//////////////// LOOP////////////////////////////////////////////
void loop() {


//fdbrt.fb0=cmd.pos0;
//fdbrt.fb1=cmd.pos1;
//fdbrt.fb2=cmd.pos2;


//Serial.println(enc0);
//Serial.println(steps_0);      
 //timerAlarmWrite(timer, 1000, true);      
 // timerAlarmEnable(timer);
//
 

memcpy(&olds,&cmd, sizeof(cmd));
//fdb={888.8,444.4,111.1,0.555,3};
//delayMicroseconds(20);
enc0=encoder.getCount();
enc1=encoder.getCount();
enc2=encoder.getCount();
enc3=encoder.getCount();






 esp_ipc_call(PRO_CPU_NUM, LoopOnProCpu, NULL);    
}
