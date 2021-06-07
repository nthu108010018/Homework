#include"mbed.h"
#include "bbcar.h"
#include "bbcar_rpc.h"
#include "stdlib.h"

Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BufferedSerial pc(USBTX, USBRX);

BBCar car(pin5, pin6, servo_ticker);
BufferedSerial uart(D1,D0); //tx,rx
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread thread;
void angle_cal(float angle);
char recv[1];
char buf[32];

int main(){
   uart.set_baud(9600);
   int i = 0;
   while(1){
      /*car.goStraight(200);
      ThisThread::sleep_for(1s);
      car.stop();
      ThisThread::sleep_for(1s);*/
      if(uart.readable()){
            
            uart.read(recv, sizeof(recv));
            //printf("%s", recv);
            if(recv[0] == '\n'){
                float angle = atof(buf);
                printf("%f\r\n", angle);
                queue.call(&angle_cal, angle);
                thread.start(callback(&queue, &EventQueue::dispatch_forever));
                for(int j = 0; j++; j<32){
                    buf[j] = '0';
                }
                i = 0;
            }
            else{
                buf[i] = recv[0];
                i++;
            }
            
            
            /*queue.call(&angle_cal, 0);
            thread.start(callback(&queue, &EventQueue::dispatch_forever));*/
            //pc.write(recv, sizeof(recv));
      }
    
   }
}

void angle_cal(float angle){
   
    if(angle>30){
        car.turn(100, 0.3);
        ThisThread::sleep_for(1s);
        car.stop();
        //printf("turn right\r\n");
    }
    else if(angle<-30){
        car.turn(100, -0.3);
        ThisThread::sleep_for(1s);
        car.stop();
        //printf("turn left\r\n");
    }
    else{
        car.goStraight(100);
        ThisThread::sleep_for(1s);
        car.stop();
        //printf("go straight\r\n");
    }
}